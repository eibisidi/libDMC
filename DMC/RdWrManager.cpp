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
	clear();
}

RdWrManager::~RdWrManager()
{
}

void RdWrManager::start()
{
	m_thread.setPriority(Poco::Thread::PRIO_HIGHEST);
	m_thread.start(*this);
}

void RdWrManager::cancel()				//停止线程
{
	m_canceled = true;

	setBusy();			//如果空闲，唤醒run，避免无法退出
	
	m_thread.join();	//等待run函数返回
	clear();
}

void RdWrManager::clear()
{
	m_idle = true;
	m_canceled = false;

	for(std::map<int, ItemQueue *>::iterator iter = tosend.begin();
					iter!= tosend.end();
					++iter)
	{
		if (iter->second)
		{
			delete (iter->second);
			iter->second = NULL;
		}
	}
	tosend.clear();
	tostop.clear();
	memset(lastSent, 0, sizeof(lastSent));
}

int RdWrManager::popItems(transData *cmdData , size_t cmdcount)
{	
#ifdef TIMING
	static double longest = 0;
	static double shortest = 1000000;
	static double total = 0;
	static int	  count = 0;
	LARGE_INTEGER frequency;									//计时器频率 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    	//计时器频率   

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif

	int activeItems = 0;
	int slaveidx;
	
	memset(cmdData, 0, sizeof(transData)* cmdcount);

	coreMutex.lock();//获取队列大锁
	
	for(std::map<int, ItemQueue *>::iterator iter = tosend.begin();
				iter!= tosend.end();
				++iter)
	{
		slaveidx = iter->first;
		if (true == queueMutex[slaveidx].tryLock())	
		{
			///////////////////////////////////////////////////////////////////////////////////////////
			if (0 == tostop.count(slaveidx))
			{
				ItemQueue *que = iter->second;
				if (que && !que->empty())
				{
					lastSent[slaveidx] = cmdData[slaveidx] =  (iter->second)->front().cmdData;
					(iter->second)->pop_front();
				}
			}
			else
			{//停止该从站
				size_t unsents = (iter->second)->size();
				
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
			queueMutex[slaveidx].unlock();
		}
	}

	coreMutex.unlock();//释放队列大锁

#ifdef TIMING

	QueryPerformanceCounter(&timeEnd); 
	elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 

	if (elapsed > 1000)
		(void)elapsed;
	
	total += elapsed;
	if(elapsed > longest)
		longest = elapsed;
	if (elapsed < shortest)
		shortest = elapsed;
	++count;
	printf("popItems elapsed = %f, longest=%f, shortest=%f, average=%f.\n", elapsed, longest, shortest, total/count);
#endif


	return activeItems;

}

void RdWrManager::pushItems(const Item *items, size_t rows, size_t cols)
{
	for(size_t  c = 0; c < cols; ++c)
	{
		int	slaveidx = items[c].index;
		queueMutex[slaveidx].lock();

		if (0 == tosend.count(slaveidx))
			tosend[slaveidx] = new ItemQueue;

		for(size_t r = 0; r < rows; ++r)
		{
			tosend[slaveidx]->push_back(items[r*cols + c]);
		}
		
		queueMutex[slaveidx].unlock();
	}
}


//同步插入命令队列：适用于多轴运动、多轴回原点，每行命令出现在一次Ecm_write调用中
void RdWrManager::pushItemsSync(const Item *items, size_t rows, size_t cols)
{
	if (cols == 1)
	{//单轴运动，获得小锁
		int		slaveidx = items[0].index;
		queueMutex[slaveidx].lock();
		
		for (size_t i = 0; i < rows * cols; ++i)
		{
			if (0 == tosend.count(slaveidx))
				tosend[slaveidx] = new ItemQueue;
			tosend[slaveidx]->push_back(items[i]);
		}

		queueMutex[slaveidx].unlock();
	}
	else//多轴插补
	{
		//一行为一个周期，一列为一个轴
		int c = 0;
		int slaveidx;

		//等待当前所有队列全部消耗完
		while(c < cols)
		{
			for(; c < cols; ++c)
			{				
				slaveidx = items[c].index;
				if (tosend.count(slaveidx)>0
					&& (!tosend[slaveidx]->empty()))
					break; //当前队列仍有待发送数据
			}
			Poco::Thread::sleep(1);
		}

		//所有队列均空
		c = 0;
		while(c < cols)
		{
			coreMutex.lock(); //获得大锁
	
			for( ; c < cols; ++c)
			{
				slaveidx = items[c].index;
				if (false == queueMutex[slaveidx].tryLock())
					break; //当前队列正忙
			}
			
			coreMutex.unlock();
			Poco::Thread::sleep(1);
		}

		//已经获得所有的队列锁
		
	//////////////////////////////////////////////////////////////////////////////////////
		for (size_t i = 0; i < rows * cols; ++i)
		{
			slaveidx = items[i].index;
			if (0 == tosend.count(slaveidx))
				tosend[slaveidx] = new ItemQueue;
			tosend[slaveidx]->push_back(items[i]);
		}
	///////////////////////////////////////////////////////////////////////////////////////
	
		//释放所有的队列锁
		coreMutex.lock(); 
		for(c = 0; c < cols; ++c)
		{
			slaveidx = items[c].index;
			queueMutex[slaveidx].unlock();
		}
		coreMutex.unlock();
	}
}

size_t RdWrManager::peekQueue(int slaveidx)
{
	size_t queuecount = 0;

	queueMutex[slaveidx].lock();

	if (tosend.count(slaveidx))
		queuecount = tosend[slaveidx]->size();
	queueMutex[slaveidx].unlock();

	return queuecount;
}

//减速停止
void RdWrManager::declStop(int slaveidx, DeclStopInfo *stopInfo)
{
	queueMutex[slaveidx].lock(); 
	tostop[slaveidx] = stopInfo;
	queueMutex[slaveidx].unlock();
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

	std::map<int, ItemQueue *>::iterator iter = tosend.begin();
	for(; iter!= tosend.end(); ++iter)
	{
		if (iter->second 
			 && !(iter->second)->empty())
		{
			break;
		}
	}

	if (iter == tosend.end()) m_idle = true;				//队列恰好为空，线程置为空闲
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
		LOGSINGLE_FATAL("ECMUSBRead failed.%s", __FILE__, __LINE__, std::string(""));
		return;
	}

	rdWrState.lastRemain 	= ECM_FIFO_SIZE;
	rdWrState.lastFifoFull	= FIFO_FULL(respData);
	rdWrState.readCount		= 0;
	rdWrState.flag1			= 0;
	
	bool	bRet;
	
	while(!m_canceled)
	{	
		m_mutex.lock();
		while(m_idle)
			m_condition.wait(m_mutex);
		m_mutex.unlock();

		while(towrite--)
		{
			popItems(cmdData, DEF_MA_MAX);

			do{
				bRet =  ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData));
				if (!bRet)
					printf("Write Error \n");
			}while(!bRet);
		}

		rdWrState.readCount 	= 0;
		do{
			do {
				bRet = ECMUSBRead((unsigned char*)respData, sizeof(respData));
				if (!bRet)
					printf("Read Error \n");
			}while(!bRet);

			++rdWrState.readCount;
			if (FIFO_FULL(respData) != rdWrState.lastFifoFull)
			{
				printf("FIFO FULL Error \n");
				LOGSINGLE_FATAL("FIFO full.flag1=%d, readCount=%d, lastRemain=%?d, fifoRemain=%?d.", __FILE__, __LINE__,
								rdWrState.flag1, rdWrState.readCount, rdWrState.lastRemain, FIFO_REMAIN(respData));
				rdWrState.lastFifoFull = FIFO_FULL(respData);	//update lastFifoFull and keep running!!!
			}

			//更新响应数据
			DmcManager::instance().setRespData(respData);

			switch(rdWrState.flag1)
			{
				case 0:
					if (FIFO_REMAIN(respData) < rdWrState.lastRemain)
					{
						rdWrState.flag1 = 1;
					}
					break;
				case 1://剩余FIFO数开始增加
					if (FIFO_REMAIN(respData) > rdWrState.lastRemain)
					{
						if (FIFO_REMAIN(respData) > FIFO_LOWATER)
						{
							rdWrState.flag1 = 3;
							rdWrState.lastRemain = FIFO_REMAIN(respData);
							goto SEND;
						}
						rdWrState.flag1 = 2;
					}
					break;
				case 2://剩余FIFO数超过低值
					if (FIFO_REMAIN(respData) > FIFO_LOWATER)
					{
						rdWrState.flag1 = 3;
						rdWrState.lastRemain = FIFO_REMAIN(respData);
						goto SEND;
					}
					break;
				default:
					LOGSINGLE_FATAL("Unexpected case.%s", __FILE__, __LINE__, std::string(""));
			}
			
			rdWrState.lastRemain = FIFO_REMAIN(respData);

			if (rdWrState.flag1 && FIFO_REMAIN(respData) == ECM_FIFO_SIZE)
			{
				printf("FIFO EMPTY Error \n");
				LOGSINGLE_FATAL("FIFO empty.flag1=%d, readCount=%d, lastRemain=%?d, fifoRemain=%?d.", __FILE__, __LINE__,
								rdWrState.flag1, rdWrState.readCount, rdWrState.lastRemain, FIFO_REMAIN(respData));
				//keep running!!!
			}
		}while(rdWrState.readCount < 300);	//防止出现死循环，连续读取之后跳出，todo?

SEND:
		//printf("last_remain=%d, fifo_remain=%d, readcount = %d, flag1=%d.\n", rdWrState.lastRemain, FIFO_REMAIN(respData), rdWrState.readCount, rdWrState.flag1);
		rdWrState.flag1 = 0;
		towrite = BATCH_WRITE;			//todo，是否修改为根据FIFO Remain增加的数量动态调整下一次写入量，而不是采用固定值？
	};

	LOGSINGLE_INFORMATION("RdWrManager Thread canceled.%s", __FILE__, __LINE__, std::string(""));
}

