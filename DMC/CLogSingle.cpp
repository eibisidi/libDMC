#include "CLogSingle.h"
#include <fstream>

Logger* pLogger = NULL;
//Logger* pPointsLogger = NULL;
std::fstream		ofs;


Logger* CLogSingle::getLogger()
{
	if (!pLogger)
	{	
		string		loggerName = "DMC.log";

		AutoPtr<FileChannel> pChannel(new FileChannel);
			
		pChannel->setProperty("path", 		loggerName);
		pChannel->setProperty("rotation", "12 M");
		pChannel->setProperty("archive", "timestamp");
		pChannel->setProperty("compress", "true");
		pChannel->setProperty("purgeAge", "10days"); 
		//pChannel->setProperty("purgeCount", "2");
		
		AutoPtr<PatternFormatter> pPF(new PatternFormatter);
		pPF->setProperty("pattern", "%Y-%m-%d %H:%M:%S:%i %p (%U:%u): %t");
		pPF->setProperty("times", "local");
		AutoPtr<FormattingChannel> pFC(new FormattingChannel(pPF, pChannel));

		AutoPtr<AsyncChannel> pAC(new AsyncChannel(pFC));

		
		Poco::Logger& logger = Logger::get(loggerName);
		logger.setChannel(pAC);

		pLogger = &logger;
	}

	return pLogger;
}

#if 0
Logger* CLogSingle::getPointsLogger()
{
	if (!pPointsLogger)
	{	
		string		loggerName = "DMCPOINTS.log";

		Poco::File  pointlog(loggerName);
		if (pointlog.exists())
			pointlog.remove();


		AutoPtr<FileChannel> pChannel(new FileChannel);
			
		pChannel->setProperty("path", 		loggerName);
		pChannel->setProperty("rotation", "12 M");
		pChannel->setProperty("archive", "timestamp");
		pChannel->setProperty("compress", "true");
		pChannel->setProperty("purgeAge", "10days"); 

		AutoPtr<AsyncChannel> pAC(new AsyncChannel(pChannel));
		
		Poco::Logger& logger = Logger::get(loggerName);
		logger.setChannel(pChannel);
			
		pPointsLogger = &logger;
	}

	return pPointsLogger;
}
#endif

void CLogSingle::logFatal(const std::string &msg ,const char* file, int line)
{
	Logger* 	pLogger = getLogger();

	if (pLogger->fatal())
	{
		pLogger->fatal(msg, file, line);
	}
}

void CLogSingle::logError(const std::string &msg ,const char* file, int line)
{
	Logger* 	pLogger = getLogger();

	if (pLogger->error())
	{
		pLogger->error(msg, file, line);
	}
}

void CLogSingle::logWarning(const std::string &msg ,const char* file, int line)
{

	Logger* 	pLogger = getLogger();

	if (pLogger->warning())
	{
		pLogger->warning(msg, file, line);
	}
}

void CLogSingle::logInformation(const std::string &msg ,const char* file, int line)
{

	Logger* 	pLogger = getLogger();

	if (pLogger->information())
	{
		pLogger->information(msg, file, line);
	}
}

void CLogSingle::logDump(const std::string &msg, const void* buffer, std::size_t length)
{
	Logger* 	pLogger = getLogger();
	pLogger->dump(msg, buffer, length, Message::PRIO_FATAL);
}


 void CLogSingle::logPoint(int p)
{
#if 0
	Logger* 	pLogger = getPointsLogger();
	
	if (pLogger->information())
	{
		pLogger->information(format("%d", p));
	}
#endif
	if (ofs.is_open())
		ofs << p << "    ";
}


void CLogSingle::logPoint(const std::string &line)
{
#if 0
	Logger*	 pLogger = getPointsLogger();

	if (pLogger->information())
	{
	 	pLogger->information(line);
	}
#endif
	if (ofs.is_open())
		ofs << line;
}


void CLogSingle::setLogLevel(int nLevel, bool logpoints)
{
	Logger* 	pLogger = getLogger();

	if (nLevel < Poco::Message::Priority::PRIO_FATAL
		|| nLevel > Poco::Message::Priority::PRIO_TRACE
		)
	{
		nLevel = 0; //turn off
	}

	pLogger->setLevel(nLevel);

	//开启规划点记录
	if (logpoints)
		ofs.open("DMCPOINTS.log",std::fstream::out | std::fstream::trunc);

	//Logger* 	pPointLogger = getPointsLogger();
	//pPointLogger->setLevel(Poco::Message::Priority::PRIO_TRACE);
}


void CLogSingle::initLogger()
{
	setLogLevel(Poco::Message::Priority::PRIO_TRACE);	//初始化记录所有信息

	if (ofs.is_open())
		ofs.close();
}

void CLogSingle::closeLogger()
{
	if (pLogger)
	{
		pLogger->getChannel()->close();
		pLogger = NULL;
	}

#if 0
	if (pPointsLogger)
	{
		pPointsLogger->getChannel()->close();
		pPointsLogger = NULL;
	}
#endif
	if (ofs.is_open())
		ofs.close();
}


CLogSingle::CLogSingle()
{

}

CLogSingle::~CLogSingle()
{
}

