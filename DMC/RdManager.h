#ifndef RD_MANAGER
#define RD_MANAGER



#include "NEXTWUSBLib_12B.h"
#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include "SlaveState.h"

#include <map>

class RdWrState
{
public:
	unsigned int lastFifoFull;	//ECM返回的fifofull记录数，如检测到发生改变，代表写入速度过快
	bool 		 consecutive;
	unsigned int fifoRemain;

	RdWrState()
		:lastFifoFull(0),
		 consecutive(false),
		 fifoRemain(0)
	{
	}
};


class RdManager : public Poco::Runnable
{
public:
	static RdManager & instance();

	virtual void run();
	void addIoSlave(int slaveidx);
	void start();
	void cancel();				//停止线程

	virtual ~RdManager();
	RdWrState					rdWrState;
	unsigned int getIoInput(short slaveidx);

	
private:
	RdManager();
	void clear();

	Poco::Thread		m_thread;
	bool				m_canceled;				//线程停止
	int 				m_towrite;


	std::map<int, IoSlaveState> ioState;							//Io模块输入、输出值

};


#endif
