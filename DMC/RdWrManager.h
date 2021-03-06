#ifndef RDWR_MANAGER
#define RDWR_MANAGER

#include <map>

#include "NEXTWUSBLib_12B.h"
#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include "SlaveState.h"
#include "GarbageCollector.h"

//减速停止信息
class DeclStopInfo
{
public:
	int			slaveIdx;
	double 		decltime;			//减速时间
	bool		valid;				//减速目标位置是否有效
	int			endpos;				//减速目标位置，valid = true时有效

	DeclStopInfo()
	{
		slaveIdx = 0;
		decltime = 0.1;
		valid 	 = false;
		endpos	 = 0;
	}
};

//减速停止信息
class AdjustInfo
{
public:
	int 		dVel;		//速率变化 +加速  -减速
	size_t		remainCount;
	
	AdjustInfo()
	{
		reset();
	}

	void reset()
	{
		dVel 			= 0;
		remainCount 	= 0;
	}
};

class RdWrState
{
public:
	unsigned int lastFifoFull;	//ECM返回的fifofull记录数，如检测到发生改变，代表写入速度过快
	int 		 readCount;

	RdWrState()
		:lastFifoFull(0),
		 readCount(0)
	{
	}
};

struct SeqLock{
	volatile unsigned int seq;
	SeqLock()
	{
		seq = 0;
	}

	void lock()
	{
		seq++;
		MemoryBarrier();
	}

	void unlock()
	{
		MemoryBarrier();
		seq++;
	}

};
	
struct CmdQueue
{
	Item 			*head;
	Item 			*tail;
	Item			*cur;
	size_t			count;		//当前队列中数目
	bool			keeprun;	//是否持续运动
	CmdQueue()
	{
		head = tail = NULL;
		cur	 = NULL;
		count = 0;
		keeprun = false;
	}
};

class RdWrManager : public Poco::Runnable
{
public:
	RdWrManager();
	virtual void run();
	void addIoSlave(int slaveidx);
	void start();
	void cancel();				//停止线程

	~RdWrManager();

	void pushItems(Item **itemLists, size_t rows, size_t cols, bool keep);
	void pushItemsSync(Item **itemLists, size_t rows, size_t cols, bool keep);
	size_t peekQueue(int slaveidx);
	void declStop(int slaveidx, DeclStopInfo *stopInfo);
	void declStopSync(DeclStopInfo **stopInfos, size_t cols);
	void setAdjust(short axis, short deltav, size_t cycles);

	void setIoOutput(short slaveidx, unsigned int output);
	unsigned int getIoOutput(short slaveidx);
	unsigned int getIoInput(short slaveidx);
	
private:
	void clear();
	int popItems(transData *cmdData, size_t count);

	Poco::Thread		m_thread;
	bool				m_canceled;				//线程停止
	bool				m_consecutive;			//连续Write模式？
	int 				m_towrite;
	GarbageCollector	m_garbageCollector;

	std::map<int, CmdQueue> 	tosend;								//待发送	命令队列				
	DeclStopInfo				*tostop[DEF_MA_MAX];				//待减速停止
	transData					lastSent[DEF_MA_MAX];				//记录上次发送CSP命令
	AdjustInfo					adjusts[DEF_MA_MAX];				//调速
	RdWrState					rdWrState;
	std::map<int, IoSlaveState> ioState;							//Io模块输入、输出值

	Poco::Mutex					coreMutex;							//Main Core Mutext To Guard each queueMutex
	SeqLock						seqLock[DEF_MA_MAX];
};
#endif

