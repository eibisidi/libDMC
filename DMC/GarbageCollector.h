#ifndef GARBAGE_COLLECTOR
#define GARBAGE_COLLECTOR

#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"
#include "NEXTWUSBLib_12B.h"

#include <list>

class Item
{
public:
	int			index;
	transData 	cmdData;
	Item		*next;
	Item		*prev;
	//add extra
	Item()
	{
		index = 0;
		cmdData.CMD = 0;
		cmdData.Parm = 0;
		cmdData.Data1 = 0;
		cmdData.Data2 = 0;
		//cmdData.Data3 = 0;
		next  = this;
		prev  = this;
	}
};


class GarbageCollector : public Poco::Runnable
{
public:
	GarbageCollector();
	virtual void run();
	void start();
	void cancel();				//停止线程

	void toss(Item *trash);
	~GarbageCollector();
private:
	void clear();

	Poco::Thread		m_thread;
	bool				m_canceled;				//线程停止
	Poco::Mutex			m_mutex;
	Poco::Condition		m_condition;
	std::list<Item*> 	m_trashes;
};


#endif
