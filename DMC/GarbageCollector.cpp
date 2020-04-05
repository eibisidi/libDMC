#include "GarbageCollector.h"

GarbageCollector::GarbageCollector()
{
	clear();
}

GarbageCollector::~GarbageCollector()
{
}

void GarbageCollector::start()
{
	m_thread.setPriority(Poco::Thread::PRIO_NORMAL);
	m_thread.start(*this);
}

void GarbageCollector::cancel()
{
	m_mutex.lock();
	m_canceled = true;
	m_condition.signal();
	m_mutex.unlock();

	m_thread.join();	//等待run函数返回
	
	clear();
}

void GarbageCollector::toss(Item *trash)
{
	m_mutex.lock();
	m_trashes.push_back(trash);
	m_condition.signal();
	m_mutex.unlock();
}

void GarbageCollector::clear()
{
	for(std::list<Item *>::iterator iter = m_trashes.begin();
			iter != m_trashes.end();
			++iter)
	{
		Item * toDel = *iter, *nextDel;
		while(toDel)
		{
			nextDel = toDel->next;
			delete toDel;
			toDel = nextDel;
		}
	}

	m_trashes.clear();
}

void GarbageCollector::run()
{	
	while(!m_canceled)
	{		
		m_mutex.lock();
		while(m_trashes.empty())
		{
			m_condition.wait(m_mutex);

			if (m_canceled)
			{
				m_mutex.unlock();
				return;
			}
		}
		
		Item * trash = m_trashes.front();
		m_trashes.pop_front();
		m_mutex.unlock();
		
		//Nuke this trash
		Item * toDel = trash, *nextDel;
		while(toDel)
		{
			nextDel = toDel->next;
			delete toDel;
			toDel = nextDel;
		}
		
	}

}


