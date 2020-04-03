#ifndef C_LOG_SINGLE
#define C_LOG_SINGLE

#include "Poco/Logger.h"
#include "Poco/NumberFormatter.h"
#include "Poco/FileChannel.h"
#include "Poco/FormattingChannel.h"
#include "Poco/PatternFormatter.h"
#include "Poco/LogStream.h"
#include "Poco/File.h"
#include "Poco/AutoPtr.h"
#include "Poco/AsyncChannel.h"

//#define  DEBUG_MEMLEAK 1

#ifdef DEBUG_MEMLEAK
//内存泄漏诊断
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define new new( _CLIENT_BLOCK, __FILE__, __LINE__)
#endif

#define LOGSINGLE_FATAL(fmt, file, line, ...) \
{																				\
		Poco::Logger* 	pLogger = CLogSingle::getLogger();						\
		if (pLogger->fatal())													\
		{																		\
			pLogger->fatal(Poco::format(fmt, __VA_ARGS__), file, line);			\
		}																		\
}																				

#define LOGSINGLE_ERROR(fmt, file, line, ...) \
{																				\
		Poco::Logger*	pLogger = CLogSingle::getLogger();						\
		if (pLogger->error())													\
		{																		\
			pLogger->error(Poco::format(fmt, __VA_ARGS__), file, line); 		\
		}																		\
}																				

#define LOGSINGLE_WARNING(fmt, file, line, ...) \
{																				\
		Poco::Logger*	pLogger = CLogSingle::getLogger();						\
		if (pLogger->warning())													\
		{																		\
			pLogger->warning(Poco::format(fmt, __VA_ARGS__), file, line); 		\
		}																		\
}				

#define LOGSINGLE_INFORMATION(fmt, file, line, ...) \
{																				\
		Poco::Logger*	pLogger = CLogSingle::getLogger();						\
		if (pLogger->information())												\
		{																		\
			pLogger->information(Poco::format(fmt, __VA_ARGS__), file, line); 	\
		}																		\
}		


class CLogSingle
{
public:
	static void logFatal(const std::string &msg ,const char* file, int line);

#if 0
	template <typename... Args>
	static void logFatal(const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Poco::Logger* 	pLogger = getLogger();
		
		if (pLogger->error())
		{
			pLogger->fatal(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}
#endif

	static void logError(const std::string &msg, const char* file, int line);

#if 0
	template <typename... Args>
	static void logError(const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Poco::Logger* 	pLogger = getLogger();
		
		if (pLogger->error())
		{
			pLogger->error(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}
#endif

	static void logWarning(const std::string &msg, const char* file, int line);

#if 0 
	template <typename... Args>
	static void logWarning(const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Poco::Logger* 	pLogger = getLogger();
		
		if (pLogger->warning())
		{
			pLogger->warning(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}
#endif
	static void logInformation(const std::string &msg, const char* file, int line);

#if 0
	template <typename... Args>
	static void logInformation( const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Poco::Logger* 	pLogger = getLogger();
		
		if (pLogger->information())
		{
			pLogger->information(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}
#endif
	static void logDump(const std::string &msg, const void* buffer, std::size_t length);

#if 0
	template <typename... Args>
	static void logPoint( const std::string &fmt, Args&&... args)
	{

		Poco::Logger* 	pLogger = getPointsLogger();
		if (pLogger->information())
		{
			pLogger->information(Poco::format(fmt, std::forward<Args>(args)...));
		}

	}
#endif

	static void logPoint(int p);
	static void logPoint(const std::string& line);

	static void setLogLevel(int nLevel, bool logpoints = false);
	static void initLogger();
	static void closeLogger();

	static Poco::Logger* getLogger();
private:
	CLogSingle();
	virtual ~CLogSingle();


	static Poco::Logger* getPointsLogger();
};

#endif
