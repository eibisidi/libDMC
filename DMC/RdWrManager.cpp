#include "RdWrManager.h"
#include "CLogSingle.h"
#include "DmcManager.h"

#define FIFO_REMAIN(respData) 	((respData)[0].Data2 & 0xFFFF)	//FIFO剩余空间
#define FIFO_FULL(respData)		((respData)[0].Data2 >> 16)		//FIFO满的次数
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)

#define BATCH_WRITE		(5)
#define FIFO_LOWATER	(150)			
#define ECM_FIFO_SIZE	(0xA0)				//ECM内部FIFO数目

RdWrManager::RdWrManager()
{
	clear();
}

RdWrManager::~RdWrManager()
{
}

void RdWrManager::addIoSlave(int slaveidx)
{
	ioState[slaveidx] = IoSlaveState();
}

void RdWrManager::start()
{
	m_thread.setPriority(Poco::Thread::PRIO_HIGHEST);
	m_thread.start(*this);
}

void RdWrManager::cancel()				//停止线程
{
	m_canceled = true;
	
	m_thread.join();	//等待run函数返回
	
	clear();
}

void RdWrManager::clear()
{
	m_canceled 		= false;
	m_consecutive	= false;
	m_towrite		= 2;

	tosend.clear();
	tostop.clear();
	memset(lastSent, 0, sizeof(lastSent));
	ioState.clear();

	for(std::map<int, CmdQueue>::iterator iter = tosend.begin();
				iter!= tosend.end();
				++iter)
	{
		CmdQueue &queue = iter->second;

		if (queue.tail)
		{
			queue.tail->next = NULL;
			while(queue.head)
			{
				Item * toDel 	= queue.head;
				queue.head		= queue.head->next;
				delete toDel;
			}
		}
	}
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
	
	int 	slaveidx;
	bool	con			= false;
	
	memset(cmdData, 0, sizeof(transData)* cmdcount);

	coreMutex.lock();//获取队列大锁

	for(std::map<int, CmdQueue>::iterator iter = tosend.begin();
				iter!= tosend.end();
				++iter)
	{
		transData localCmd;
		Item *localCur = NULL, *localNext=NULL, *localHead=NULL;
		memset(&localCmd, 0, sizeof(transData));
		slaveidx = iter->first;
		CmdQueue &queue = iter->second;


		if (tostop[slaveidx])
		{
			DeclStopInfo *dcInfo = tostop[slaveidx];
			int estimate = (int)(dcInfo->decltime * CYCLES_PER_SEC) + 1;
			if (queue.cur != NULL 
				&& queue.cur->next != queue.head)
			{
				int q0 = queue.cur->cmdData.Data1;
				int q1 = (queue.cur->next)->cmdData.Data1;

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

				queue.tail->next	= NULL;

				Item 	*toOverWrite = queue.cur;
				Item	*newTail = queue.tail;
				int 	deltaCount = 0;
				while (vel > 0)
				{
					vel = vel - dec;
					q	= int(q + dir * vel);

					toOverWrite->cmdData.CMD 	= CSP;
					toOverWrite->cmdData.Data1	= q;
					newTail						= toOverWrite;
					toOverWrite = toOverWrite->next;
					if (toOverWrite->next == queue.head
						&& vel > 0)
					{
						Item * newItem = new Item;
						toOverWrite->next 	= newItem;
						newItem->prev		= toOverWrite;
						newItem->next		= queue.head;
						
						toOverWrite  	= newItem;
						++deltaCount;
					}
				}

				//释放内存
				if (!deltaCount 
					&& toOverWrite->next != queue.head)
				{
					queue.tail->next	= NULL;
					Item * toDel = toOverWrite, *nextDel;
					while(toDel)
					{
						nextDel = toDel->next;
						delete toDel;
						toDel = nextDel;
						deltaCount--;
					}
				}

				queue.tail			= newTail;
				queue.tail->next	= queue.head;
				queue.count		   += deltaCount;
				
				tostop[slaveidx]->valid = true;
				tostop[slaveidx]->endpos = queue.tail->cmdData.Data1;
			}
			else
			{
				//并未运动
				if (lastSent[slaveidx].CMD == CSP)
				{
					tostop[slaveidx]->valid = true;
					tostop[slaveidx]->endpos = lastSent[slaveidx].Data1;
				}
				else{
					tostop[slaveidx]->valid = false;
				}
			}
		}
		else
		{
			unsigned int begin_seq,end_seq;
			begin_seq = seqLock[slaveidx].seq;
			
			if (queue.cur)
			{
				localCur = queue.cur;
				localNext= localCur->next;
				localHead= queue.head;
				localCmd = (queue.cur)->cmdData;
			}
			
			end_seq   = seqLock[slaveidx].seq;

			if (!((begin_seq & 1) ||( end_seq != begin_seq)))
			{
				if (localCur)
				{
					cmdData[slaveidx] = localCmd;
					if (localNext == localHead)
						queue.cur = NULL;
					else
						queue.cur = localNext;
				}
			}
		}

		if (CSP == cmdData[slaveidx].CMD)
			con = true;
	}

	coreMutex.unlock();//释放队列大锁

	if (false == m_consecutive
		&& true == con)
	{
		m_consecutive	= true;
		m_towrite = BATCH_WRITE - 1;
	}

	if (true == m_consecutive
		&& false == con)
	{
		m_consecutive	= false;
		m_towrite = 1;
	}
		
	//修改IO命令
	for(std::map<int, IoSlaveState>::const_iterator iter = ioState.begin();
			iter != ioState.end();
			++iter)
	{
		if (m_towrite)
		{
			cmdData[iter->first].CMD 	= IO_WR;
			cmdData[iter->first].Data1	= (iter->second).getOutput();
		}
		else
			cmdData[iter->first].CMD = IO_RD;
	}

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
	return 0;
}

void RdWrManager::pushItems(Item **itemLists, size_t rows, size_t cols)
{
	for(size_t  c = 0; c < cols; ++c)
	{
		Item * toAppend = itemLists[c];
		int	slaveidx 	= toAppend->index;

		seqLock[slaveidx].lock();

		if (tosend[slaveidx].head == NULL)
		{
			tosend[slaveidx].head 	= toAppend;
			tosend[slaveidx].tail 	= toAppend->prev;
			tosend[slaveidx].cur 	= toAppend;
			tosend[slaveidx].count	= rows;
		}
		else
		{//append to tosend list, chain it up
			(tosend[slaveidx].head)->prev 	= toAppend->prev;	
			(tosend[slaveidx].tail)->next 	= toAppend;
			(toAppend->prev)->next			= (tosend[slaveidx].head);
			toAppend->prev					= (tosend[slaveidx].tail);

			(tosend[slaveidx].tail) 	   	= (tosend[slaveidx].head)->prev;
			tosend[slaveidx].count			+= rows;
			
			if (tosend[slaveidx].cur == NULL)					//旧的数据已经发送光
				tosend[slaveidx].cur = toAppend;
		}

		//释放已经发送过的内存
		(tosend[slaveidx].cur)->prev 	= tosend[slaveidx].tail;
		(tosend[slaveidx].tail)->next 	= tosend[slaveidx].cur;

		Item *todel;
		while(tosend[slaveidx].head != tosend[slaveidx].cur)
		{
			todel = tosend[slaveidx].head;
			tosend[slaveidx].head 			= todel->next;
			todel->next = NULL;
			todel->prev = NULL;
			delete todel;
			--tosend[slaveidx].count;
		}

		seqLock[slaveidx].unlock();
	}
}


//同步插入命令队列：适用于多轴运动、多轴回原点，每行命令出现在一次Ecm_write调用中
void RdWrManager::pushItemsSync(Item **itemLists, size_t rows, size_t cols)
{
	if (cols == 1)
	{//单轴运动，获得小锁
		pushItems(itemLists, rows, cols);
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
				slaveidx = itemLists[c]->index;
				if (tosend.count(slaveidx)>0
					&& (tosend[slaveidx].cur))
					break; //当前队列仍有待发送数据
			}
		}

		//所有队列均空
		coreMutex.lock(); //获得大锁
		for(c = 0; c < cols; ++c)
		{
			slaveidx = itemLists[c]->index;
			seqLock[slaveidx].lock();
		}
		coreMutex.unlock(); //获得大锁
		//已经获得所有的队列锁
		
	//////////////////////////////////////////////////////////////////////////////////////

	for(size_t	c = 0; c < cols; ++c)
	{
		Item * toAppend = itemLists[c];
		int	slaveidx 	= toAppend->index;

		Item *todel;
		if (tosend[slaveidx].tail)
		{
			(tosend[slaveidx].tail)->next = NULL;
			while(tosend[slaveidx].head)
			{
				todel = tosend[slaveidx].head;
				tosend[slaveidx].head 			= todel->next;
				delete todel;
			}
		}
		
		//empty and memory deleted
		tosend[slaveidx].head 	= toAppend;
		tosend[slaveidx].tail 	= toAppend->prev;
		tosend[slaveidx].cur 	= toAppend;
		tosend[slaveidx].count	= rows;
	}

	///////////////////////////////////////////////////////////////////////////////////////
	
		//释放所有的队列锁
		coreMutex.lock(); 
		for(c = 0; c < cols; ++c)
		{
			slaveidx = itemLists[c]->index;
			seqLock[slaveidx].unlock();
		}
		coreMutex.unlock();
	}
}

size_t RdWrManager::peekQueue(int slaveidx)
{
	size_t queuecount = 0;

	if (tosend.count(slaveidx))
		queuecount = (tosend[slaveidx].cur) ? 1: 0;

	return queuecount;
}

//减速停止
void RdWrManager::declStop(int slaveidx, DeclStopInfo *stopInfo)
{
	queueMutex[slaveidx].lock(); 
	tostop[slaveidx] = stopInfo;
	queueMutex[slaveidx].unlock();
}

void RdWrManager::setIoOutput(short slaveidx, unsigned int output)
{
	ioState[slaveidx].setOutput(output);
}

unsigned int RdWrManager::getIoOutput(short slaveidx)
{
	return ioState[slaveidx].getOutput();
}

unsigned int RdWrManager::getIoInput(short slaveidx)
{
	return ioState[slaveidx].getInput();
}

void RdWrManager::run()
{
	transData	cmdData[DEF_MA_MAX];
	transData	respData[DEF_MA_MAX];
		
	//初始化FIFO状态
	if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
	{
		LOGSINGLE_FATAL("ECMUSBRead failed.%s", __FILE__, __LINE__, std::string(""));
		return;
	}

	rdWrState.lastFifoFull	= FIFO_FULL(respData);
	rdWrState.readCount		= 0;
	
	bool	bRet;
	
	while(!m_canceled)
	{	
		while(m_towrite--)
		{
			popItems(cmdData, DEF_MA_MAX);

			do{
				bRet =  ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData));
				if (!bRet)
					LOGSINGLE_FATAL("Write Error.", __FILE__, __LINE__, std::string(""));
			}while(!bRet);
		}

		rdWrState.readCount 	= 0;
		do{
			do {
				bRet = ECMUSBRead((unsigned char*)respData, sizeof(respData));
				if (!bRet)
					LOGSINGLE_FATAL("Read Error.", __FILE__, __LINE__, std::string(""));
			}while(!bRet);

			++rdWrState.readCount;
			if (FIFO_FULL(respData) != rdWrState.lastFifoFull)
			{
				LOGSINGLE_FATAL("FIFO full.readCount=%d, fifoRemain=%?d.", __FILE__, __LINE__, rdWrState.readCount,  FIFO_REMAIN(respData));
			}

			//更新输入值
			for(std::map<int, IoSlaveState>::iterator iter = ioState.begin();
						iter != ioState.end();
						++iter)
			{
				if (IO_RD == respData[iter->first].CMD)
					(iter->second).setInput(respData[iter->first].Data1);
			}

			DmcManager::instance().setRespData(respData);

			if (m_consecutive)
			{
				if (FIFO_REMAIN(respData) == ECM_FIFO_SIZE)
				{
					LOGSINGLE_FATAL("readCount=%d, fifoRemain=%?d.", __FILE__, __LINE__, rdWrState.readCount,  FIFO_REMAIN(respData));
					goto SEND;
				}
				if (FIFO_REMAIN(respData) > FIFO_LOWATER)
				{
					goto SEND;
				}
			}
			else
			{
				if (FIFO_REMAIN(respData)>=ECM_FIFO_SIZE)
				{
					goto SEND;
				}
			}
		}while(true);

SEND:
		if (m_consecutive)
			m_towrite = BATCH_WRITE;			//todo，是否修改为根据FIFO Remain增加的数量动态调整下一次写入量，而不是采用固定值？
		else
			m_towrite = 2;
	};

	LOGSINGLE_INFORMATION("RdWrManager Thread canceled.%s", __FILE__, __LINE__, std::string(""));
}

