To build

```mkdir -p ~/ros/ign_us_system/src/```

```git clone https://github.com/simonleonard/ign_us_system.git ~/ros/ign_us_system/src```

```cd ~/ros/ign_us_system```

```colcon build --metas src/ign_us_system/igsio src/ign_us_system/PlusLib```

To run

```cd ~/ros/ign_us_system```

```source install/setup.bash```

```ros2 launch  ur5_us_bringup  ismr_ur_bringup.launch.py```

To move

```ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/spacenav/twist```
