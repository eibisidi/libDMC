#ifndef RDWR_MANAGER
#define RDWR_MANAGER

#include <map>
#include <deque>
#include <atomic> 


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

//����ֹͣ��Ϣ
class DeclStopInfo
{
public:
	double 		decltime;			//����ʱ��
	bool		valid;				//����Ŀ��λ���Ƿ���Ч
	int			endpos;				//����Ŀ��λ�ã�valid = trueʱ��Ч

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
	RdWrManager();
	virtual void run();
	void start();
	void cancel();				//ֹͣ�߳�

	~RdWrManager();
	void setIdle();
	void pushItems(Item *items, int rows, int cols);
	int peekQueue(int slaveidx);
	void declStop(int slaveidx, DeclStopInfo *stopInfo);

private:
	void clear();
	int popItems(transData *cmdData, size_t count);

	Poco::Thread		m_thread;
	Poco::Mutex  		m_mutex;
	Poco::Condition 	m_condition;			//��������
	bool				m_idle;
	bool				m_canceled;				//�߳�ֹͣ

	typedef std::deque<Item> ItemQueue;
	typedef std::atomic<bool> QueueFlag;

	QueueFlag					queueFlags[DEF_MA_MAX];				//�����е�ǰ״̬��true��æ, false����
	std::map<int, ItemQueue*> 	tosend;								//������	�������
	std::map<int, DeclStopInfo*>	tostop;							//������ֹͣ
	transData					lastSent[DEF_MA_MAX];				//��¼�ϴη�������
};

#endif

