#include "RdWrManager.h"
#include "CLogSingle.h"
#include "DmcManager.h"


#define FIFO_REMAIN(respData) 	((respData)[0].Data2 & 0xFFFF)	//FIFO剩余空间
#define FIFO_FULL(respData)		((respData)[0].Data2 >> 16)		//FIFO满的次数
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)

#define BATCH_WRITE		(20)
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

int RdWrManager::popItems(transData *cmdData)
{	

#if 0
	LARGE_INTEGER frequency;								//计时器频率 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    //计时器频率   

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif
	int activeItems = 0;
	int slaveidx;
	m_mutex.lock();	
	
	for(std::map<int, ItemQueue *>::iterator iter = tosend.begin();
				iter!= tosend.end();
				++iter)
	{
		slaveidx = iter->first;
		if (QUEUE_IDLE == queueState[slaveidx])
		{
			queueState[slaveidx] = QUEUE_BUSY;
			///////////////////////////////////////////////////////////////////////////////////////////

			if (0 == tostop.count(slaveidx))
			{
				ItemQueue *que = iter->second;
				if (!que->empty())
				{
					lastSent[slaveidx] = cmdData[slaveidx] =  (iter->second)->front().cmdData;
					(iter->second)->pop_front();
				}
			}
			else
			{//停止该从站
				int unsents = (iter->second)->size();
				
				if (unsents > 0)
				{
					int estimate = (int)(tostop[slaveidx]->decltime * CYCLES_PER_SEC) + 1;					//预计停止周期数量
					int	q0, q1;
					if (unsents > 2 && estimate < unsents)
					{//重新计算一条减速曲线
						q0 = (iter->second)->front().cmdData.Data1;
						(iter->second)->pop_front();
						q1 = (iter->second)->front().cmdData.Data1;

						(iter->second)->clear();		//清除当前队列中所有待发送CSP目标位置
	
						int     vel0 = (q1 > q0) ? (q1 - q0) : (q0 - q1);		//初始运动速率
						double  dec  = 1.0 * vel0 / estimate;					//减速度
						int     dir  = (q1 > q0) ? 1 : -1;						//运动方向
						double  vel  = vel0;
						int 	q    = q0;
						
						Item    item;
						item.index 			= slaveidx;
						item.cmdData.CMD 	= CSP;
						item.cmdData.Data1	= q0;

						lastSent[slaveidx] = cmdData[slaveidx] =  item.cmdData;
						
						while (vel > 0)
						{
							vel = vel - dec;
							q 	= int(q + dir * vel);

							item.cmdData.Data1 = q;
							(iter->second)->push_back(item);
						}
					}
					
					tostop[slaveidx]->valid = true;
					if ((iter->second)->size() > 0)
						tostop[slaveidx]->endpos = (iter->second)->back().cmdData.Data1;
					else
						tostop[slaveidx]->endpos = q0;
				}
				else
				{//并未运动
					if (lastSent[slaveidx].CMD == CSP)
					{
						tostop[slaveidx]->valid = true;
						tostop[slaveidx]->endpos = lastSent[slaveidx].Data1;
					}
					else{
						tostop[slaveidx]->valid = false;
					}
				}

				//移除该停止指令 
				tostop.erase(slaveidx);
			}
			////////////////////////////////////////////////////////////////////////////////////////////
			queueState[slaveidx] = QUEUE_IDLE;
		}
	}

	m_mutex.unlock();

	return activeItems;
#if 0
	QueryPerformanceCounter(&timeEnd); 
	elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 
	printf("time elapsed = %f\n", elapsed);
#endif
}

//每行代表一个周期，每列代表一个从站
void RdWrManager::pushItems(Item *items, int rows, int cols)
{
	//一行为一个周期，一列为一个轴
	int c;
	int slaveidx;
	while(true)
	{
		m_mutex.lock();	

		for(c = 0; c < cols; ++c)
		{
			slaveidx = items[c].index;
			if (QUEUE_BUSY == queueState[slaveidx])
			{
				break;
			}
		}

		if (c == cols)
		{
			for(c = 0; c < cols; ++c)
			{
				slaveidx = items[c].index;
				queueState[slaveidx] = QUEUE_BUSY;
			}
			m_mutex.unlock();
			break;
		}
		
		m_mutex.unlock();
		Poco::Thread::sleep(1);
	}

	for (size_t i = 0; i < rows * cols; ++i)
	{
		slaveidx = items[i].index;
		if (0 == tosend.count(slaveidx))
			tosend[slaveidx] = new ItemQueue;
		tosend[slaveidx]->push_back(items[i]);
	}

	//每个队列置为空闲
	m_mutex.lock();	
	for(c = 0; c < cols; ++c)
	{
		slaveidx = items[c].index;
		queueState[slaveidx] = QUEUE_IDLE;
	}
	m_idle = false;
	m_condition.signal();
	m_mutex.unlock();
}

int RdWrManager::peekQueue(int slaveidx)
{
	int queuecount = 0;
	while(true)
	{
		m_mutex.lock(); 
		if (QUEUE_IDLE == queueState[slaveidx])
		{
			break;
		}
		m_mutex.unlock();
		Poco::Thread::sleep(1);
	}

	if (tosend.count(slaveidx))
		queuecount = tosend[slaveidx]->size();
	m_mutex.unlock();

	return queuecount;
}

//减速停止
void RdWrManager::declStop(int slaveidx, DeclStopInfo *stopInfo)
{
	m_mutex.lock(); 
	tostop[slaveidx] = stopInfo;
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
	int 		towrite = BATCH_WRITE;
	transData	cmdData[DEF_MA_MAX];
	transData	respData[DEF_MA_MAX];
		
	//初始化FIFO状态
	if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
	{
		CLogSingle::logFatal("ECMUSBRead failed.", __FILE__, __LINE__);
		return;
	}

	unsigned int last_remain = ECM_FIFO_SIZE;
	unsigned int last_fifofull = FIFO_FULL(respData);
	
	int	flag1 = 0;
	
	while(true)
	{	
		m_mutex.lock();
		while(m_idle)
			m_condition.wait(m_mutex);
		m_mutex.unlock();

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
		//if (noneActiveCount > 5)
		//	m_idle = true;
	};
}

