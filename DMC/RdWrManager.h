#ifndef RDWR_MANAGER
#define RDWR_MANAGER

#include <map>
#include <list>

#include "NEXTWUSBLib_12B.h"
#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include "SlaveState.h"
#include "GarbageCollector.h"

#define DEF_BATCHWRITE		(5)				//ȱʡ��������д�����
#define DEF_FIFOLW			(140)			//ȱʡFIFO LowWater

#define RDWR_TIMING 1

//����ֹͣ��Ϣ
class DeclStopInfo
{
public:
	int			slaveIdx;
	double 		decltime;			//����ʱ��
	bool		valid;				//����Ŀ��λ���Ƿ���Ч
	int			endpos;				//����Ŀ��λ�ã�valid = trueʱ��Ч

	DeclStopInfo()
	{
		slaveIdx = 0;
		decltime = 0.1;
		valid 	 = false;
		endpos	 = 0;
	}
};

//����ֹͣ��Ϣ
class AdjustInfo
{
public:
	int 		dVel;		//���ʱ仯 +����  -����
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
	unsigned int lastFifoFull;	//ECM���ص�fifofull��¼�������⵽�����ı䣬����д���ٶȹ���
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
	size_t			count;		//��ǰ��������Ŀ
	bool			keeprun;	//�Ƿ�����˶�
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
	void cancel();				//ֹͣ�߳�

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
	void	setRdParas(unsigned int batchwrite, unsigned int fifolw);
private:
	void clear();
	int popItems(transData *cmdData, size_t count);

	Poco::Thread		m_thread;
	bool				m_canceled;				//�߳�ֹͣ
	bool				m_consecutive;			//����Writeģʽ��
	int					m_flag;					//FIFO empty check state flag
	unsigned int		m_boostcount;
	int 				m_towrite;
	GarbageCollector	m_garbageCollector;

	unsigned int		BATCH_WRITE;
	unsigned int		FIFO_LOWATER;

	std::map<int, CmdQueue> 	tosend;								//������	�������				
	DeclStopInfo				*tostop[DEF_MA_MAX];				//������ֹͣ
	transData					lastSent[DEF_MA_MAX];				//��¼�ϴη���CSP����
	AdjustInfo					adjusts[DEF_MA_MAX];				//����
	RdWrState					rdWrState;
	std::map<int, IoSlaveState> ioState;							//Ioģ�����롢���ֵ

	Poco::Mutex					coreMutex;							//Main Core Mutext To Guard each queueMutex
	SeqLock						seqLock[DEF_MA_MAX];

	double 						frequency;							//��ʱ��Ƶ�� 

#ifdef RDWR_TIMING
	struct Ts
	{	
		LARGE_INTEGER	t0;
		LARGE_INTEGER	t1;
		
		Ts()
		{
			t0.QuadPart = 0;
			t1.QuadPart = 0;
		}
	};
	
	struct RoundTs
	{
		LARGE_INTEGER 	t0;
		LARGE_INTEGER 	t1;
		LARGE_INTEGER   t2;
		LARGE_INTEGER 	t3;
		LARGE_INTEGER   t4;
		LARGE_INTEGER 	t5;
		LARGE_INTEGER   t6;
		std::list<Ts>   reads;
		std::list<Ts>	writes;
		RoundTs()
		{
			t0.QuadPart = 0;
			t1.QuadPart = 0;
			t2.QuadPart = 0;
			t3.QuadPart = 0;
			t4.QuadPart = 0;
			t5.QuadPart = 0;
			t6.QuadPart = 0;
		}
	};

#define MAX_RECORD_ROUNDS (20)
	Ts					tmpreadts;
	Ts					tmpwritets;
    RoundTs			  	tmproundts;
	std::list<RoundTs> 	record;
	void	dumpRecord() const;
#endif
};
#endif

