#include "RdWrManager.h"
#include "CLogSingle.h"
#include "DmcManager.h"


#define FIFO_REMAIN(respData) 	((respData)[0].Data2 & 0xFFFF)	//FIFO剩余空间
#define FIFO_FULL(respData)		((respData)[0].Data2 >> 16)		//FIFO满的次数
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)

#define BATCH_WRITE		(24)
#define FIFO_LOWATER	(32)			
#define ECM_FIFO_SIZE	(0xA0)				//ECM内部FIFO数目

RdWrManager::RdWrManager()
{
	m_idle = true;
}

RdWrManager::~RdWrManager()
{
}

RdWrManager & RdWrManager::instance()
{
	static RdWrManager _inst;
	return _inst;
}

void RdWrManager::start()
{
	m_thread.setPriority(Poco::Thread::PRIO_HIGHEST);
	m_thread.start(*this);
}

void RdWrManager::popItems(transData *cmdData)
{	

#if 0
	LARGE_INTEGER frequency;								//计时器频率 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    //计时器频率   

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif

	m_mutex.lock();	
	
	for(std::map<int, ItemQueue *>::iterator iter = tosend.begin();
				iter!= tosend.end();
				++iter)
	{
		if (QUEUE_IDLE == queueState[iter->first])
		{
			queueState[iter->first] = QUEUE_BUSY;
			ItemQueue *que = iter->second;
			if (!que->empty())
			{
				cmdData[iter->first] =  (iter->second)->front().cmdData;
				(iter->second)->pop();
			}

			queueState[iter->first] = QUEUE_IDLE;
		}
	}

	m_mutex.unlock();
#if 0
	QueryPerformanceCounter(&timeEnd); 
	elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 
	printf("time elapsed = %f\n", elapsed);
#endif
}

void RdWrManager::pushItems(const std::vector<Item> &items)
{
	int index = items[0].index;

	while(true)
	{
		m_mutex.lock();	
		if (QUEUE_BUSY == queueState[index])
		{
			m_mutex.unlock();
			Poco::Thread::sleep(1);
		}
		else
		{
			queueState[index] = QUEUE_BUSY;
			m_mutex.unlock();
			break;
		}
	}


	//printf("pushItems lock aquired.\n");
	for (size_t i = 0; i < items.size(); ++i)
	{
		int index = items[i].index;
		if (0 == tosend.count(index))
			tosend[index] = new ItemQueue;
		tosend[index]->push(items[i]);
	}

	m_mutex.lock();	
	queueState[index] = QUEUE_IDLE;
	m_idle = false;
	m_condition.signal();
	m_mutex.unlock();

}

void RdWrManager::setBusy()
{
	m_mutex.lock();	
	m_idle = false;
	m_condition.signal();
	m_mutex.unlock();
}

void RdWrManager::setIdle()
{
	m_mutex.lock();	
	m_idle = true;
	m_mutex.unlock();
}



void RdWrManager::run()
{
	int 					towrite = BATCH_WRITE;
	transData	cmdData[DEF_MA_MAX];
	transData	respData[DEF_MA_MAX];

	//初始化FIFO状态
	if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
	{
		CLogSingle::logFatal("ECMUSBRead failed.", __FILE__, __LINE__);
		return;
	}

	unsigned int last_remain = 160;
	unsigned int last_fifofull = FIFO_FULL(respData);
	
	int	flag1 = 0;
	
	while(true)
	{	
		m_mutex.lock();
		while(m_idle)
			m_condition.wait(m_mutex);
		m_mutex.unlock();
		//printf("RdWrManager signaled.\n");

		
		while(towrite--)
		{
			memset(cmdData, 0, sizeof(cmdData));
			popItems(cmdData);
			if (!ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
				printf("Write Error \n");
		}

		do{
			if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
				printf("Read Error \n");

			if (FIFO_FULL(respData) != last_fifofull)
			{
				printf("FIFO FULL Error \n");
				throw;
			}

			//更新响应数据
			DmcManager::instance().setRespData(respData);

			switch(flag1)
			{
				case 0:
					if (FIFO_REMAIN(respData) < last_remain)
					{
						flag1 = 1;
					}
					break;
				case 1:
					if (FIFO_REMAIN(respData) > last_remain)
					{
						flag1 = 2;
					}
					break;
				case 2:
					if (FIFO_REMAIN(respData) > FIFO_LOWATER)
					{
						flag1 = 0;
						last_remain = FIFO_REMAIN(respData);
						towrite = BATCH_WRITE;
						goto LABEL;
					}
					break;
				default:
					throw;

			}
			
			last_remain = FIFO_REMAIN(respData);

			if (flag1 && FIFO_REMAIN(respData) == ECM_FIFO_SIZE)
			{
				printf("FIFO EMPTY Error \n");
				throw;
			}
		}while(true);

LABEL:
		//printf("to write = %d, last_remain=%d, fifo_remain=%d.\n", towrite, last_remain, FIFO_REMAIN(respData));
		//printf("min = %f, max=%f, average=%f, curpos=0x%x.\n", min, max, (total/160), curpos);
		//printf("curpos=0x%x.\n", curpos);
		;
	};
}

