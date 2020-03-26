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
	unsigned int lastFifoFull;	//ECM���ص�fifofull��¼�������⵽�����ı䣬����д���ٶȹ���
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
	void cancel();				//ֹͣ�߳�

	virtual ~RdManager();
	RdWrState					rdWrState;
	unsigned int getIoInput(short slaveidx);

	
private:
	RdManager();
	void clear();

	Poco::Thread		m_thread;
	bool				m_canceled;				//�߳�ֹͣ
	int 				m_towrite;


	std::map<int, IoSlaveState> ioState;							//Ioģ�����롢���ֵ

};


#endif
