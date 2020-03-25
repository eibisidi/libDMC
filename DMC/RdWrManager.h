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

class RdWrState
{
public:
	unsigned int lastRemain;	//��һ�ε���ECMUSBRead�����ص�FIFO��������ÿ�ε���ECMUSBRead�����
	unsigned int lastFifoFull;	//ECM���ص�fifofull��¼�������⵽�����ı䣬����д���ٶȹ���
	int 		 readCount;
	int			 flag1;			//1 �����������½�  2:��������������

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
	void cancel();				//ֹͣ�߳�

	~RdWrManager();

	void pushItems(const Item *items, size_t rows, size_t cols);
	void pushItemsSync(const Item *items, size_t rows, size_t cols);
	size_t peekQueue(int slaveidx);
	void declStop(int slaveidx, DeclStopInfo *stopInfo);

private:
	void clear();
	int popItems(transData *cmdData, size_t count);

	Poco::Thread		m_thread;
	bool				m_canceled;				//�߳�ֹͣ
	bool				m_consecutive;			//����Writeģʽ��
	int 				m_towrite;

	typedef std::deque<Item> ItemQueue;
	typedef bool		 	QueueFlag;

	std::map<int, ItemQueue*> 	tosend;								//������	�������
	std::map<int, DeclStopInfo*>	tostop;							//������ֹͣ
	transData					lastSent[DEF_MA_MAX];				//��¼�ϴη�������
	RdWrState					rdWrState;

	Poco::Mutex		coreMutex;						//Main Core Mutext To Guard each queueMutex
	Poco::Mutex		queueMutex[DEF_MA_MAX];
};

#endif

