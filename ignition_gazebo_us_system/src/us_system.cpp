#include <us_system/us_system.hpp>
#include <vtkMatrix4x4.h>

#include <ignition/plugin/Register.hh>
#include <ignition/sensors/SensorFactory.hh>

#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/CustomSensor.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/Util.hh>
#include <gz/rendering/Utils.hh>

#include <iomanip>

using namespace ignition;
using namespace gazebo;
using namespace systems;

us_system::us_system(){}
us_system::~us_system(){}

void us_system::Configure( const ignition::gazebo::Entity& entity,
			   const std::shared_ptr<const sdf::Element>& sdf,
			   ignition::gazebo::EntityComponentManager& ecm,
			   ignition::gazebo::EventManager&){

  model = ignition::gazebo::Model(entity);
  
}

void us_system::PreUpdate( const ignition::gazebo::UpdateInfo& info,
			   ignition::gazebo::EntityComponentManager& ecm ){

  // Get all the custom sensors
  ecm.EachNew<gz::sim::components::CustomSensor,
           gz::sim::components::ParentEntity>
    ([&](const gz::sim::Entity &entity,
         const gz::sim::components::CustomSensor *custom,
         const gz::sim::components::ParentEntity *parent)->bool{
      
      // Get sensor's scoped name without the world
      auto sensorScopedName = gz::sim::removeParentScope(gz::sim::scopedName(entity, ecm, "::", false), "::");

      sdf::Sensor data = custom->Data();
      data.SetName(sensorScopedName);
      
      ignition::sensors::SensorFactory sensorFactory;
      sensor = sensorFactory.CreateSensor<USSensor>(data);
      if (nullptr == sensor){
        ignerr << "Failed to create sensor [" << sensorScopedName << "]" << std::endl;
        return false;
      }

      // Here, the sensor was detected. The PlusLib paths should be set.
      else{
	
	ignmsg << "PlusDir: " << sensor->plusdir << std::endl;
	ignmsg << "PlusConfig: " << sensor->plusconfig << std::endl;
	  
	plusConfig = vtkPlusConfig::New();

	plusConfig->SetOutputDirectory(sensor->plusdir);
	plusConfig->SetImageDirectory(sensor->plusdir);
	plusConfig->SetDeviceSetConfigurationDirectory(sensor->plusdir);
	
	vtkPlusLogger::Instance()->SetLogLevel(vtkPlusLogger::LOG_LEVEL_TRACE);

	vtkSmartPointer<vtkXMLDataElement> configRootElement=vtkSmartPointer<vtkXMLDataElement>::New();
	if(PlusXmlUtils::ReadDeviceSetConfigurationFromFile(configRootElement, sensor->plusconfig.c_str()) == PLUS_FAIL){
	  ignerr << "FAILED to configure" << std::endl;
	  exit(EXIT_FAILURE);
	}
	
	transformRepository = vtkSmartPointer<vtkPlusTransformRepository>::New();
	if(transformRepository->ReadConfiguration(configRootElement) != PLUS_SUCCESS){
	  ignerr << "Failed to read transforms for transform repository!" << std::endl;
	  exit(EXIT_FAILURE);
	}
	
	usSimulator = vtkSmartPointer<vtkPlusUsSimulatorAlgo>::New();
	if(usSimulator->ReadConfiguration(configRootElement) != PLUS_SUCCESS){
	  ignerr << "Failed to read US simulator configuration!" << std::endl;
	  exit(EXIT_FAILURE);
	}
	
	usSimulator->SetTransformRepository(transformRepository);
	
      }
      return true;
    });
  
}

void us_system::PostUpdate( const ignition::gazebo::UpdateInfo&,
			    const ignition::gazebo::EntityComponentManager& ecm){
  
  ignition::math::Pose3d probepose;
  ecm.Each<gz::sim::components::Name, gz::sim::components::ParentEntity>
    ([&](const gz::sim::Entity &entity,
         const gz::sim::components::Name *name,
         const gz::sim::components::ParentEntity *parent)->bool{

      std::optional<std::string> data = ecm.ComponentData<gz::sim::components::Name>( entity );
      if( data->find("probe_link") != std::string::npos ){
	
	std::optional<ignition::math::Pose3d> pose = ecm.ComponentData<gz::sim::components::Pose>( entity );

	probepose = worldPose( entity, ecm );
      }
      
      return true;
      
    });
  
  //PlusTrackedFrame* frame = trackedFrameList->GetTrackedFrame(0);
  
  // Begin of the messy part. No (very little) idea about the coordinate frame convention of PlusTK.
  // This used to work for the probe in the xml file but now broken
  
  vtkMatrix4x4* Rtoffset = vtkMatrix4x4::New();
  Rtoffset->SetElement(0,0,-1); Rtoffset->SetElement(0,1, 0); Rtoffset->SetElement(0,2, 0);
  Rtoffset->SetElement(1,0, 0); Rtoffset->SetElement(1,1,-1); Rtoffset->SetElement(1,2, 0);
  Rtoffset->SetElement(2,0, 0); Rtoffset->SetElement(2,1, 0); Rtoffset->SetElement(2,2, 1);

  vtkMatrix4x4* Rtprobe = vtkMatrix4x4::New();
  Rtprobe->SetElement(1,1, -1);
  Rtprobe->SetElement(2,2, -1);
  Rtprobe->SetElement(0,3,-probepose.Pos().Z()*1000);
  Rtprobe->SetElement(1,3, probepose.Pos().X()*1000);
  Rtprobe->SetElement(2,3,-probepose.Pos().Y()*1000);
  
  vtkMatrix4x4* Rtplus = vtkMatrix4x4::New();
  vtkMatrix4x4::Multiply4x4( Rtoffset, Rtprobe, Rtplus );

  vtkMatrix4x4* Rtign = vtkMatrix4x4::New();
  vtkMatrix4x4::Multiply4x4( Rtoffset, Rtplus, Rtign );

  Rtign->SetElement(0,0,Rtplus->GetElement(0,0));
  Rtign->SetElement(0,1,Rtplus->GetElement(0,1));
  Rtign->SetElement(0,2,Rtplus->GetElement(0,2));
  Rtign->SetElement(1,0,Rtplus->GetElement(1,0));
  Rtign->SetElement(1,1,Rtplus->GetElement(1,1));
  Rtign->SetElement(1,2,Rtplus->GetElement(1,2));
  Rtign->SetElement(2,0,Rtplus->GetElement(2,0));
  Rtign->SetElement(2,1,Rtplus->GetElement(2,1));
  Rtign->SetElement(2,2,Rtplus->GetElement(2,2));

  PlusTransformName igsioname("ReferenceToImage");
  PlusTrackedFrame frame;
  frame.SetFrameTransform(igsioname, Rtign );
  frame.SetFrameTransformStatus(igsioname, TOOL_OK );
  
  if(transformRepository->SetTransforms(frame) != PLUS_SUCCESS){
    ignerr << "Failed to set repository transforms from tracked frame!" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  usSimulator->Modified();
  usSimulator->Update();
  vtkImageData* simOutput = usSimulator->GetOutput();
  int *dims = simOutput->GetDimensions();

  int width = dims[0];
  int height = dims[1];
  gz::msgs::PixelFormatType msgsPixelFormat = gz::msgs::PixelFormatType::L_INT8;

  sensor->image.set_width(width);
  sensor->image.set_height(height);
  sensor->image.set_step(width * ignition::rendering::PixelUtil::BytesPerPixel(ignition::rendering::PixelFormat::PF_L8));
  sensor->image.set_pixel_format_type(msgsPixelFormat);

  auto frame_key = sensor->image.mutable_header()->add_data();
  frame_key->set_key("frame_id");
  frame_key->add_value("probe_link");
  sensor->image.set_data((char*) simOutput->GetScalarPointer(), width*height);

  sensor->Update(std::chrono::steady_clock::duration());
  
}

IGNITION_ADD_PLUGIN( us_system,
		     ignition::gazebo::System,
		     us_system::ISystemConfigure,
		     us_system::ISystemPreUpdate,
		     us_system::ISystemPostUpdate )

