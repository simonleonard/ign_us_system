/*=Plus=header=begin======================================================
  Program: Plus
  Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
  See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __PLUSLOGGER_H
#define __PLUSLOGGER_H

#include "vtkPlusCommonExport.h"

#include "vtkObject.h"
#include "vtkOutputWindow.h"
#include <fstream>
#include <sstream>

class vtkPlusRecursiveCriticalSection;

/*!
  \class vtkPlusConsoleOutputWindow
  \brief This is a special output class for VTK logs that enforces VTK to
  log to the console instead of displaying them in a pop-up window.
  \sa vtkPlusLoggerOutputWindow
  \ingroup PlusLibCommon
*/
class vtkPlusConsoleOutputWindow : public vtkOutputWindow
{
public:
  /*!
    The new method of vtkOutputWindow would create a vtkWin32OutputWindow.cxx
    isntance, which displays logs in a pop-up window. Instead, we directly call
    the constructor of vtkOutputWindow, which forces to create a vtkOutputWindow instance
    that writes log messages on the console.
  */
  static vtkPlusConsoleOutputWindow* New()
  { return new vtkPlusConsoleOutputWindow; }
};

/*!
  \class vtkPlusLoggerOutputWindow
  \brief This is a special output class for forwarding VTK logs
  to the Plus logging system instead of displaying them in a pop-up window
  or directly on the console.
  \sa vtkPlusConsoleOutputWindow
  \ingroup PlusLibCommon
*/
class vtkPlusLoggerOutputWindow : public vtkOutputWindow
{
public:
  vtkTypeMacro(vtkPlusLoggerOutputWindow, vtkOutputWindow);
  static vtkPlusLoggerOutputWindow* New();
  virtual void PrintSelf(ostream& os, vtkIndent indent);

  /*! Display VTK message as Plus LOG_INFO text */
  virtual void DisplayText(const char* text);
  /*! Display VTK message as Plus LOG_ERROR text */
  virtual void DisplayErrorText(const char* text);
  /*! Display VTK message as Plus LOG_WARNING text */
  virtual void DisplayWarningText(const char* text);
  /*! Display VTK message as Plus LOG_WARNING text */
  virtual void DisplayGenericWarningText(const char* text);
  /*! Display VTK message as Plus LOG_DEBUG text */
  virtual void DisplayDebugText(const char* text);

protected:
  vtkPlusLoggerOutputWindow();
  virtual ~vtkPlusLoggerOutputWindow();

  /*! VTK error messages contain multiple lines. This method replaces newline characters by another separator character */
  void ReplaceNewlineBySeparator(std::string& str);

private:
  vtkPlusLoggerOutputWindow(const vtkPlusLoggerOutputWindow&);  // Not implemented.
  void operator=(const vtkPlusLoggerOutputWindow&);  // Not implemented.
};

/*!
  \class vtkPlusLogger
  \brief This singleton class provides logging into file and/or the console
  with adjustable verbosity.
  \ingroup PlusLibCommon
*/
class vtkPlusCommonExport vtkPlusLogger : public vtkObject
{
public:
  enum LogEvents
  {
    MessageLogged         = 92300,
    WideMessageLogged     = 92301
  };

public:
  /*! Logging level to control the verbosity of the logging */
  enum LogLevelType
  {
    /*!
      Indicates an error. Whenever a method fails and returns with
      PLUS_FAIL it shall log a message with this error level before
      returning. Logging of the error may be omitted when a method
      returns with PLUS_FAIL because a called method returned with
      PLUS_FAIL.
    */
    LOG_LEVEL_ERROR = 1,
    /*!
      Indicates a potential error: there is a chance that an
      error occurred, but from the available data it
      cannot be determined for sure.
    */
    LOG_LEVEL_WARNING = 2,
    /*! Used for logging information that may be useful for a user. */
    LOG_LEVEL_INFO = 3,
    /*! Used for logging information that may be useful for a developer. */
    LOG_LEVEL_DEBUG = 4,
    /*! Detailed debugging information. */
    LOG_LEVEL_TRACE = 5,
    /*! Default logging level to be applied when it is not set explicitly by the application. */
    LOG_LEVEL_DEFAULT = LOG_LEVEL_INFO,
    /*!
      If this value is used for SetLogLevel then the current log level
      will be kept.
    */
    LOG_LEVEL_UNDEFINED = 100
  };

  static int UnlimitedLogMessages() { return -1; };

  /*!  Get a pointer to the single existing object instance */
  static vtkPlusLogger* Instance();

  /*! Convert a string into a log level type */
  static LogLevelType GetLogLevelType(const std::string& logLevelString);

  /*!
    Add a new message to the log. Instead of using this method directly it is advisable
    to use the convenience macros: LOG_ERROR, LOG_WARNING, LOG_INFO, LOG_DEBUG, LOG_TRACE
    \param level Level of the current message
    \param msg Text of the log message
    \param fileName Name of the file where the message comes from
    \param lineNumber Line number within the file where the message comes from
    \param optionalPrefix a prefix to append after the timestamp, but before the logged message
  */
  void LogMessage(LogLevelType level, const char* msg, const char* fileName, int lineNumber, const char* optionalPrefix = NULL);
  void LogMessage(LogLevelType level, const wchar_t* msg, const char* fileName, int lineNumber, const wchar_t* optionalPrefix = NULL);
  void LogMessage(LogLevelType level, const std::string& msg, const char* fileName, int lineNumber, const std::string& optionalPrefix = "");
  void LogMessage(LogLevelType level, const std::wstring& msg, const char* fileName, int lineNumber, const std::wstring& optionalPrefix = L"");

  /*!
    Add a new message to the log without line number of filename
    \param level Level of the current message
    \param msg Text of the log message
    \param optionalPrefix a prefix to append after the timestamp, but before the logged message
  */
  void LogMessage(LogLevelType level, const std::string& msg, const std::string& optionalPrefix = "");
  void LogMessage(LogLevelType level, const std::wstring& msg, const std::wstring& optionalPrefix = L"");

  /*! Get the current log level. Messages that has a higher level than the current log level are ignored. */
  int GetLogLevel();
  /*! Get the current log level. Messages that has a higher level than the current log level are ignored. */
  std::string GetLogLevelString();
  /*! Set the current log level. Messages that has a higher level than the current log level are ignored. */
  void SetLogLevel(int logLevel);

  /*!
    Helper function to print a progress bar on the console
    \param percent Percentage completed. Minimum value is 0, maximum value is 100.
  */
  static void PrintProgressbar(int percent);

  /*!
    Set the name of the file where the messages will be logged to.
    Until a file name is set the messages are cached in memory and are written to
    file when the file name is specified.
  */
  void SetLogFileName(const char* logfilename);
  /*! Get the name of the file where the messages are logged to */
  std::string GetLogFileName();

protected:
  vtkPlusLogger();
  ~vtkPlusLogger();

  /*! Writes the messages that are cached in memory to the log file and clears the cache. */
  void Flush();

private:
  vtkPlusLogger(vtkPlusLogger const&);
  vtkPlusLogger& operator=(vtkPlusLogger const&);

  /*! Pointer to the singleton instance */
  static vtkPlusLogger*   m_pInstance;
  /*! Log level used for controlling the verbosity of the logging */
  int                     m_LogLevel;
  /*! Cache for storing messages that have not yet been written to file */
  std::wostringstream     m_LogStream;
  /*! Stream object of the log output file */
  std::wofstream          m_FileStream;
  /*! Name of the log output file */
  std::string             m_LogFileName;

  /*!
    Critical section that is used to serialize output of messages.\
    It is necessary because the logging object may be used in multiple
    threads simultaneously.
  */
  vtkPlusRecursiveCriticalSection* m_CriticalSection;
};

#endif
