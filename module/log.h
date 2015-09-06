#pragma once

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <vector>
#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>
#include "version.h"

using namespace log4cxx;

namespace module
{
namespace logger
{
// useful macro
#ifndef C99
#	define LOG_INFO(param, args...)              Log::GetInstance().Printf(param, ##args)
#	define LOG_WARN(param, args...)              Log::GetInstance().PrintWarn(param, ##args)
#	define LOG_WARN_WITH_NO(param, args...)      Log::GetInstance().PrintWarnWithNo(param, ##args)
#	define LOG_ERROR(param, args...)             Log::GetInstance().PrintError(param, ##args)
#	define LOG_ERROR_WITH_NO(param, args...)     Log::GetInstance().PrintErrorWithNo(param, ##args)
#	define LOG_CRITICAL(param, args...)          Log::GetInstance().PrintCriticalError(param, ##args)
#	define LOG_CRITICAL_WITH_NO(param, args...)  Log::GetInstance().PrintCriticalErrorWithNo(param, ##args)
#	define LOG_DEBUG(param, args...)             Log::GetInstance().Debug(param, ##args)
#else
#	define LOG_INFO(param, ...)              Log::GetInstance().Printf(param, __VA_ARGS__)
#	define LOG_WARN(param, ...)              Log::GetInstance().PrintWarn(param, __VA_ARGS__)
#	define LOG_WARN_WITH_NO(param, ...)      Log::GetInstance().PrintWarnWithNo(param, __VA_ARGS__)
#	define LOG_ERROR(param, ...)             Log::GetInstance().PrintError(param, __VA_ARGS__)
#	define LOG_ERROR_WITH_NO(param, ...)     Log::GetInstance().PrintErrorWithNo(param, __VA_ARGS__)
#	define LOG_CRITICAL(param, ...)          Log::GetInstance().PrintCriticalError(param, __VA_ARGS__)
#	define LOG_CRITICAL_WITH_NO(param, ...)  Log::GetInstance().PrintCriticalErrorWithNo(param, __VA_ARGS__)
#	define LOG_DEBUG(param, ...)             Log::GetInstance().Debug(param, __VA_ARGS__)
#endif

class Log
{
private:
	static LoggerPtr logger;
	static Log log;
	static LoggerPtr Initialize();

	long maxNameLen;
	char* name;

protected:
	void GetProcName();

	Log();
	virtual ~Log();

public:
	static Log& GetInstance()
	{
		return log;
	}

	// INFO
	void Printf(const char* str, ...);
	// WARN
	void PrintWarn(const char* str, ...);
	void PrintWarnWithNo(const char* str, ...);
	// ERROR
	void PrintError(const char* str, ...);
	void PrintErrorWithNo(const char* str, ...);
	// CRITICAL
	void PrintCriticalError(const char* str, ...);
	void PrintCriticalErrorWithNo(const char* str, ...);
	// DEBUG
	void Debug(const char* str, ...);
};
}
}
