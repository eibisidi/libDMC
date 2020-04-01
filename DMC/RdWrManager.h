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
	unsigned int lastFifoFull;	//ECM���ص�fifofull��¼�������⵽�����ı䣬����д���ٶȹ���
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
		//mb()
		seq++;
	}

	void unlock()
	{
		//mb()
		seq++;
	}

};
	
struct CmdQueue
{
	Item 			*head;
	Item 			*tail;
	Item			*cur;
	size_t			count;		//��ǰ��������Ŀ
	CmdQueue()
	{
		head = tail = NULL;
		cur	 = NULL;
		count = 0;
	}

	
};

class RdWrManager : public Poco::Runnable
{
public:
	RdWrManager();
	virtual void run();
	void addIoSlave(int slaveidx);
	void start();
	void cancel();				//ֹͣ�߳�

	~RdWrManager();

	void pushItems(Item **itemLists, size_t rows, size_t cols);
	void pushItemsSync(Item **itemLists, size_t rows, size_t cols);
	size_t peekQueue(int slaveidx);
	void declStop(int slaveidx, DeclStopInfo *stopInfo);

	void setIoOutput(short slaveidx, unsigned int output);
	unsigned int getIoOutput(short slaveidx);
	unsigned int getIoInput(short slaveidx);
	
private:
	void clear();
	int popItems(transData *cmdData, size_t count);

	Poco::Thread		m_thread;
	bool				m_canceled;				//�߳�ֹͣ
	bool				m_consecutive;			//����Writeģʽ��
	int 				m_towrite;

	typedef std::deque<Item> ItemQueue;


	std::map<int, CmdQueue> 	tosend;								//������	�������				
	DeclStopInfo				*tostop[DEF_MA_MAX];				//������ֹͣ
	transData					lastSent[DEF_MA_MAX];				//��¼�ϴη�������
	RdWrState					rdWrState;
	std::map<int, IoSlaveState> ioState;							//Ioģ�����롢���ֵ

	Poco::Mutex					coreMutex;							//Main Core Mutext To Guard each queueMutex
	SeqLock						seqLock[DEF_MA_MAX];
};

#endif

