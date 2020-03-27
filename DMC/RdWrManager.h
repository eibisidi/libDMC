#ifndef RDWR_MANAGER
#define RDWR_MANAGER

#include <map>
#include <deque>

#include "NEXTWUSBLib_12B.h"
#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include "SlaveState.h"

class Item
{
public:
	int			index;
	transData 	cmdData;
	Item		*next;
	//add extra
	Item()
	{
		index = 0;
		cmdData.CMD = 0;
		cmdData.Parm = 0;
		cmdData.Data1 = 0;
		cmdData.Data2 = 0;
		next  = NULL;
		//cmdData.Data3 = 0;
	}
};

//减速停止信息
class DeclStopInfo
{
public:
	double 		decltime;			//减速时间
	bool		valid;				//减速目标位置是否有效
	int			endpos;				//减速目标位置，valid = true时有效

	DeclStopInfo()
	{
		decltime = 0.1;
		valid 	 = false;
		endpos	 = 0;
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
	unsigned int seq;
	SeqLock()
	{
		seq = 0;
	}

	void lock()
	{
		seq++;
	}

	void unlock()
	{
		seq++;
	}

};
	
struct CmdQueue
{
	Item 			*head;
	Item 			*tail;
	Item			*cur;
	
	CmdQueue()
	{
		head = tail = NULL;
		cur	 = NULL;
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

	void pushItems(const Item *items, size_t rows, size_t cols);
	void pushItemsSync(const Item *items, size_t rows, size_t cols);
	size_t peekQueue(int slaveidx);
	void declStop(int slaveidx, DeclStopInfo *stopInfo);

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

	typedef std::deque<Item> ItemQueue;


	std::map<int, CmdQueue> 	tosend;								//待发送	命令队列
	std::map<int, DeclStopInfo*>	tostop;							//待减速停止
	transData					lastSent[DEF_MA_MAX];				//记录上次发送命令
	RdWrState					rdWrState;
	std::map<int, IoSlaveState> ioState;							//Io模块输入、输出值

	Poco::Mutex		coreMutex;						//Main Core Mutext To Guard each queueMutex
	Poco::Mutex		queueMutex[DEF_MA_MAX];

	SeqLock		seqLock[DEF_MA_MAX];
};

#endif

