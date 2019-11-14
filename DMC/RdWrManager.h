#ifndef RDWR_MANAGER
#define RDWR_MANAGER

#include <map>
#include <queue>
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
};

class RdWrManager : public Poco::Runnable
{
public:
	static RdWrManager & instance();
	virtual void run();
	void start();
	~RdWrManager();
	void setIdle();
		void pushItems(const std::vector<Item> &items);
private:
	RdWrManager();
	void popItems(transData *cmdData);

	void setBusy();

	Poco::Thread		m_thread;
	Poco::Mutex  		m_mutex;
	Poco::Condition 	m_condition;			//条件变量
	bool				m_idle;

	bool				m_queueaccess;


	typedef std::queue<Item> ItemQueue;

	enum QueueState{
		QUEUE_IDLE,
		QUEUE_BUSY,
	};

	QueueState					queueState[42];
	std::map<int, ItemQueue*> 	tosend;		//待发送	
};

#endif

