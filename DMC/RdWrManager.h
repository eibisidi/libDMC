#ifndef RDWR_MANAGER
#define RDWR_MANAGER

#include <map>
#include <deque>
#include <vector>

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

class RdWrManager : public Poco::Runnable
{
public:
	static RdWrManager & instance();
	virtual void run();
	void start();
	void cancel();				//停止线程

	~RdWrManager();
	void setIdle();
	void pushItems(Item *items, int rows, int cols);
	int peekQueue(int slaveidx);
	void declStop(int slaveidx, DeclStopInfo *stopInfo);

private:
	RdWrManager();
	void clear();
	int popItems(transData *cmdData);

	Poco::Thread		m_thread;
	Poco::Mutex  		m_mutex;
	Poco::Condition 	m_condition;			//条件变量
	bool				m_idle;
	bool				m_canceled;				//线程停止

	typedef std::deque<Item> ItemQueue;

	enum QueueState{
		QUEUE_IDLE = 0,
		QUEUE_BUSY,
	};

	QueueState					queueState[DEF_MA_MAX];
	std::map<int, ItemQueue*> 	tosend;		//待发送	命令队列
	std::map<int, DeclStopInfo*>	tostop;		//待减速停止
	transData					lastSent[DEF_MA_MAX];		//记录上次发送命令
};

#endif

