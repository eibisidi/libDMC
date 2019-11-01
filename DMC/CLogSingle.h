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

using namespace std;
using namespace Poco;

class CLogSingle
{
public:
	static void logFatal(const std::string &msg ,const char* file, int line);

	template <typename... Args>
	static void logFatal(const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Logger* 	pLogger = getLogger();
		
		if (pLogger->error())
		{
			pLogger->fatal(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}

	static void logError(const std::string &msg, const char* file, int line);

	template <typename... Args>
	static void logError(const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Logger* 	pLogger = getLogger();
		
		if (pLogger->error())
		{
			pLogger->error(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}

	static void logWarning(const std::string &msg, const char* file, int line);
		
	template <typename... Args>
	static void logWarning(const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Logger* 	pLogger = getLogger();
		
		if (pLogger->warning())
		{
			pLogger->warning(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}

	static void logInformation(const std::string &msg, const char* file, int line);
		
	template <typename... Args>
	static void logInformation( const std::string &fmt, const char* file, int line, Args&&... args)
	{
		Logger* 	pLogger = getLogger();
		
		if (pLogger->information())
		{
			pLogger->information(Poco::format(fmt, std::forward<Args>(args)...), file, line);
		}

	}

	static void logDump(const std::string &msg, const void* buffer, std::size_t length);

	template <typename... Args>
	static void logPoint( const std::string &fmt, Args&&... args)
	{

		Logger* 	pLogger = getPointsLogger();
		if (pLogger->information())
		{
			pLogger->information(Poco::format(fmt, std::forward<Args>(args)...));
		}

	}

	static void logPoint(int p);

	static void setLogLevel(int nLevel);
	static void initLogger();
	static void closeLogger();
private:
	CLogSingle();
	virtual ~CLogSingle();


	static Poco::Logger* getLogger();
	static Poco::Logger* getPointsLogger();

};

#endif
