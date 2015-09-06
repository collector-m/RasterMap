#include "log.h"
#include <string.h>
#include <unistd.h>

#define SELF_FILE "/proc/self/exe"
#define DEFAULT_NAME "module"

namespace module
{
namespace logger
{
Log Log::log;
LoggerPtr Log::logger = Log::Initialize();

LoggerPtr Log::Initialize()
{
	PropertyConfigurator::configure("../log4cxx.properties");
	return Logger::getRootLogger();
}

Log::Log()
{
	maxNameLen = pathconf("/", _PC_NAME_MAX);
	name = new char[maxNameLen + 1];
	memset(name, 0, maxNameLen + 1);
	GetProcName();
}

Log::~Log()
{
	delete[] name;
}

void Log::GetProcName()
{
	long maxPathLen = pathconf("/", _PC_PATH_MAX);
	char* selfname = new char[maxPathLen];
	memset(selfname, 0, sizeof(selfname));

	if(readlink(SELF_FILE, selfname, maxPathLen) != -1)
	{
		char* start = strrchr(selfname, '/');
		if(start != NULL)
			strncpy(name, start + 1, maxNameLen);
		else
			strncpy(name, selfname, maxNameLen);
//		printf("log name = %s\n", name);
	}
	else
		strncpy(name, DEFAULT_NAME, maxNameLen);

	delete[] selfname;
}

void Log::Printf(const char* str, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, str);
	vsprintf(buf, str, list);
//	printf("[INFO]%s\n", buf);
	sprintf(tmp, "[%s] %s", this->name, buf);
	if(logger != NULL)
	{
		LOG4CXX_INFO(logger, tmp);
	}
	va_end(list);
}

void Log::PrintWarn(const char* str, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, str);
	vsprintf(buf, str, list);
//	printf("[INFO]%s\n", buf);
	sprintf(tmp, "[%s] %s", this->name, buf);
	if(logger != NULL)
	{
		LOG4CXX_WARN(logger, tmp);
	}
	va_end(list);
}

void Log::PrintWarnWithNo(const char* str, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, str);
	vsprintf(buf, str, list);
	sprintf(tmp, "[%s] %s - %s", this->name, buf, strerror(errno));
	if(logger != NULL)
	{
		LOG4CXX_WARN(logger, tmp);
	}
	va_end(list);
}

void Log::PrintErrorWithNo(const char* str, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, str);
	vsprintf(buf, str, list);
	sprintf(tmp, "[%s] %s - %s", this->name, buf, strerror(errno));
	if(logger != NULL)
	{
		LOG4CXX_ERROR(logger, tmp);
	}
	va_end(list);
}

void Log::PrintError(const char* str, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, str);
	vsprintf(buf, str, list);
	sprintf(tmp, "[%s] %s", this->name, buf);
	if(logger != NULL)
	{
		LOG4CXX_ERROR(logger, tmp);
	}
	va_end(list);
}

void Log::PrintCriticalErrorWithNo(const char* str, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, str);
	vsprintf(buf, str, list);
	sprintf(tmp, "[%s] %s - %s", this->name, buf, strerror(errno));
	if(logger != NULL)
	{
		LOG4CXX_FATAL(logger, tmp);
	}
	va_end(list);
}

void Log::PrintCriticalError(const char* str, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, str);
	vsprintf(buf, str, list);
	sprintf(tmp, "[%s] %s", this->name, buf);
	if(logger != NULL)
	{
		LOG4CXX_FATAL(logger, tmp);
	}
	va_end(list);
}

void Log::Debug(const char* format, ...)
{
	char buf[2000], tmp[2000];
	va_list list;
	va_start(list, format);
	vsprintf(buf, format, list);
	sprintf(tmp, "[%s] %s", this->name, buf);
	if(logger != NULL)
	{
		LOG4CXX_DEBUG(logger, tmp);
	}
	va_end(list);
}
}
}
