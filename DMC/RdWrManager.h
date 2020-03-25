#ifndef RDWR_MANAGER
#define RDWR_MANAGER

#include <map>
#include <deque>

#include "NEXTWUSBLib_12B.h"
#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"

class Item
{
public:
	int			index;
	transData 	cmdData;
	//add extra
	Item()
	{
		index = 0;
		cmdData.CMD = 0;
		cmdData.Parm = 0;
		cmdData.Data1 = 0;
		cmdData.Data2 = 0;
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
	unsigned int lastRemain;	//上一次调用ECMUSBRead，返回的FIFO空闲数，每次调用ECMUSBRead后更新
	unsigned int lastFifoFull;	//ECM返回的fifofull记录数，如检测到发生改变，代表写入速度过快
	int 		 readCount;
	int			 flag1;			//1 空闲数正在下降  2:空闲数正在上升

	RdWrState()
		:lastRemain(0),
		 lastFifoFull(0),
		 readCount(0),
		 flag1(0)
	{
	}
};

class RdWrManager : public Poco::Runnable
{
public:
	RdWrManager();
	virtual void run();
	void start();
	void cancel();				//停止线程

	~RdWrManager();

	void pushItems(const Item *items, size_t rows, size_t cols);
	void pushItemsSync(const Item *items, size_t rows, size_t cols);
	size_t peekQueue(int slaveidx);
	void declStop(int slaveidx, DeclStopInfo *stopInfo);

private:
	void clear();
	int popItems(transData *cmdData, size_t count);

	Poco::Thread		m_thread;
	bool				m_canceled;				//线程停止
	bool				m_consecutive;			//连续Write模式？
	int 				m_towrite;

	typedef std::deque<Item> ItemQueue;
	typedef bool		 	QueueFlag;

	std::map<int, ItemQueue*> 	tosend;								//待发送	命令队列
	std::map<int, DeclStopInfo*>	tostop;							//待减速停止
	transData					lastSent[DEF_MA_MAX];				//记录上次发送命令
	RdWrState					rdWrState;

	Poco::Mutex		coreMutex;						//Main Core Mutext To Guard each queueMutex
	Poco::Mutex		queueMutex[DEF_MA_MAX];
};

#endif

