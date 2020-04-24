#include "RdWrManager.h"
#include "CLogSingle.h"
#include "DmcManager.h"
#include "GarbageCollector.h"

#define FIFO_REMAIN(respData) 	((respData)[0].Data2 & 0xFFFF)	//FIFOʣ��ռ�
#define FIFO_FULL(respData)		((respData)[0].Data2 >> 16)		//FIFO���Ĵ���
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)
	
#define ECM_FIFO_SIZE	(0xA0)				//ECM�ڲ�FIFO��Ŀ

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
	m_garbageCollector.start();
	m_thread.setOSPriority(THREAD_PRIORITY_TIME_CRITICAL);
	m_thread.start(*this);
}

void RdWrManager::cancel()				//ֹͣ�߳�
{
	m_canceled = true;
	
	m_thread.join();	//�ȴ�run��������
	
	clear();
	m_garbageCollector.cancel();
}

void RdWrManager::clear()
{
	m_canceled 		= false;
	m_consecutive	= false;
	m_flag			= 0;
	m_boostcount	= 0;
	m_towrite		= 2;

	BATCH_WRITE		= DEF_BATCHWRITE;
	FIFO_LOWATER	= DEF_FIFOLW;

	memset(tostop, 0, sizeof(tostop));
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
	tosend.clear();
}

int RdWrManager::popItems(transData *cmdData , size_t cmdcount)
{	
#ifdef TIMING
	static double longest = 0;
	static double shortest = 1000000;
	static double total = 0;
	static int	  count = 0;
	LARGE_INTEGER frequency;									//��ʱ��Ƶ�� 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    	//��ʱ��Ƶ��   

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif
	
	int 	slaveidx;
	bool	con			= false;
	
	memset(cmdData, 0, sizeof(transData)* cmdcount);

	if (coreMutex.tryLock())//��ȡ���д���
	{
		for(std::map<int, CmdQueue>::iterator iter = tosend.begin();
			iter!= tosend.end();
			++iter)
		{
			transData localCmd;
			Item *localCur = NULL, *localNext=NULL, *localHead=NULL;
			memset(&localCmd, 0, sizeof(transData));
			slaveidx = iter->first;
			CmdQueue &queue = iter->second;
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
			if ((begin_seq & 1) ||( end_seq != begin_seq)) 
				continue;	/*��һ������*/

			if (queue.keeprun)
			{//�����˶�
				if (localCur)
				{//�����˶����ٽ׶�
					cmdData[slaveidx] = localCmd;
					if (localNext == localHead)
						queue.cur = NULL;
					else
						queue.cur = localNext;
				}
				else
				{//�Ѿ��������˶��׶�
					if (tostop[slaveidx])
					{//����
						DeclStopInfo *dcInfo = tostop[slaveidx];
						int estimate = (int)(dcInfo->decltime * CYCLES_PER_SEC) + 1;

						int v  		 = queue.tail->cmdData.Data1 - (queue.tail->prev)->cmdData.Data1;	//�����ٶ�
						int q0 		 = lastSent[slaveidx].Data1 + v;
						int 	vel0 = (v > 0) ? (v) : (-v);											//��ʼ�˶�����
						double	dec  = 1.0 * vel0 / estimate;											//���ٶ�
						int 	dir  = (v > 0) ? 1 : -1;												//�˶�����
						double	vel  = vel0;
						int 	q	 = q0;

						Item	item;
						item.index			= slaveidx;
						item.cmdData.CMD	= CSP;
						item.cmdData.Data1	= q0;

						cmdData[slaveidx] =  item.cmdData;

						Item	*toOverWrite = queue.head;
						Item	*newTail 	= queue.tail;
						int 	deltaCount 	= 0;
						int		newCount    = 0;
						while (vel > 0)
						{
							vel = vel - dec;
							q	= int(q + dir * vel);

							toOverWrite->cmdData.CMD	= CSP;
							toOverWrite->cmdData.Data1	= q;
							newTail	= toOverWrite;
							++newCount;
							toOverWrite = toOverWrite->next;
							if (toOverWrite == queue.head
								&& vel > 0)
							{
								Item * newItem = new Item;

								queue.tail->next	= newItem;
								newItem->prev		= queue.tail;
								newItem->next		= queue.head;
								queue.tail			= newItem;			//����tail
								queue.head->prev	= newItem;

								toOverWrite 		= newItem;
								++deltaCount;
							}
						}

						//�ͷ��ڴ�
						if (!deltaCount 
							&& toOverWrite != queue.head)
						{
							queue.tail->next	= NULL;
							m_garbageCollector.toss(toOverWrite);
						}

						queue.cur			= queue.head;		//��ͷ��ʼ����
						queue.tail			= newTail;
						queue.tail->next	= queue.head;
						queue.head->prev	= queue.tail;
						queue.count 	  	= newCount;
						queue.keeprun		= false;			//�л�Ϊ�������˶�

						tostop[slaveidx]->valid = true;
						tostop[slaveidx]->endpos = queue.tail->cmdData.Data1;
						tostop[slaveidx] = NULL;
					}
					else
					{//�����˶������ٽ׶�
						int q1 = queue.tail->cmdData.Data1;
						int q0 = (queue.tail->prev)->cmdData.Data1;
						int v  = q1 - q0;
						if (adjusts[slaveidx].remainCount)
						{
							--adjusts[slaveidx].remainCount;
							int dVel = adjusts[slaveidx].dVel;
							if (v > 0)
							{
								if (dVel >= 0)
									v += dVel;
								else
									v -= dVel;
							}
							else
							{
								if (dVel >= 0)
									v -= dVel;
								else
									v += dVel;
							}
						}
						cmdData[slaveidx].CMD 	= CSP;
						cmdData[slaveidx].Data1	= lastSent[slaveidx].Data1 + v;
					}
				}
			}
			else
			{//�ǳ����˶�
				if (tostop[slaveidx])
				{
					DeclStopInfo *dcInfo = tostop[slaveidx];
					int estimate = (int)(dcInfo->decltime * CYCLES_PER_SEC) + 1;
					if (queue.cur != NULL 
						&& queue.cur->next != queue.head)
					{
						int q0 = queue.cur->cmdData.Data1;
						int q1 = (queue.cur->next)->cmdData.Data1;

						int     vel0 = (q1 > q0) ? (q1 - q0) : (q0 - q1);		//��ʼ�˶�����
						double  dec  = 1.0 * vel0 / estimate;					//���ٶ�
						int     dir  = (q1 > q0) ? 1 : -1;						//�˶�����
						double  vel  = vel0;
						int 	q    = q0;

						Item    item;
						item.index 			= slaveidx;
						item.cmdData.CMD 	= CSP;
						item.cmdData.Data1	= q0;

						cmdData[slaveidx] =  item.cmdData;

						Item	*toOverWrite = queue.head;
						Item	*newTail 	= queue.tail;
						int 	deltaCount 	= 0;
						int		newCount    = 0;
						while (vel > 0)
						{
							vel = vel - dec;
							q	= int(q + dir * vel);

							toOverWrite->cmdData.CMD	= CSP;
							toOverWrite->cmdData.Data1	= q;
							newTail	= toOverWrite;
							++newCount;

							toOverWrite = toOverWrite->next;
							if (toOverWrite == queue.head
								&& vel > 0)
							{
								Item * newItem = new Item;

								queue.tail->next	= newItem;
								newItem->prev		= queue.tail;
								newItem->next		= queue.head;
								queue.tail			= newItem;			//����tail
								queue.head->prev	= newItem;

								toOverWrite 		= newItem;
								++deltaCount;
							}
						}

						//�ͷ��ڴ�
						if (!deltaCount 
							&& toOverWrite != queue.head)
						{
							queue.tail->next	= NULL;
							m_garbageCollector.toss(toOverWrite);
						}

						queue.cur			= queue.head;		//��ͷ��ʼ����
						queue.tail			= newTail;
						queue.tail->next	= queue.head;
						queue.head->prev	= queue.tail;
						queue.count 	   	= newCount;
						queue.keeprun		= false;			//�л�Ϊ�������˶�

						tostop[slaveidx]->valid = true;
						tostop[slaveidx]->endpos = queue.tail->cmdData.Data1;
						tostop[slaveidx] = NULL;
					}
					else
					{
						//��δ�˶�
						if (lastSent[slaveidx].CMD == CSP)
						{
							tostop[slaveidx]->valid = true;
							tostop[slaveidx]->endpos = lastSent[slaveidx].Data1;
						}
						else{
							tostop[slaveidx]->valid = false;
						}
					}
					tostop[slaveidx] = NULL;
				}
				else
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
			lastSent[slaveidx] = cmdData[slaveidx];
		}

		coreMutex.unlock();//�ͷŶ��д���
	}
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
		
	//�޸�IO����
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

void RdWrManager::pushItems(Item **itemLists, size_t rows, size_t cols, bool keep)
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
			
			if (tosend[slaveidx].cur == NULL)					//�ɵ������Ѿ����͹�
				tosend[slaveidx].cur = toAppend;
		}

		//�ͷ��Ѿ����͹����ڴ�
		if(tosend[slaveidx].head != tosend[slaveidx].cur)
		{
			(tosend[slaveidx].cur)->prev->next = NULL;
			m_garbageCollector.toss(tosend[slaveidx].head);
		}
		
		tosend[slaveidx].head			= (tosend[slaveidx].cur);
		(tosend[slaveidx].cur)->prev 	= tosend[slaveidx].tail;
		(tosend[slaveidx].tail)->next 	= tosend[slaveidx].cur;

		tosend[slaveidx].keeprun= keep;
		if (keep)
		{
			adjusts[slaveidx].reset();
		}

		seqLock[slaveidx].unlock();
	}
}

//ͬ������������У������ڶ����˶��������ԭ�㣬ÿ�����������һ��Ecm_write������
void RdWrManager::pushItemsSync(Item **itemLists, size_t rows, size_t cols, bool keep)
{
	if (cols == 1)
	{//�����˶������С��
		pushItems(itemLists, rows, cols, keep);
	}
	else//����岹
	{
		//һ��Ϊһ�����ڣ�һ��Ϊһ����
		int c = 0;
		int slaveidx;

		//�ȴ���ǰ���ж���ȫ��������
		while(c < cols)
		{
			for(; c < cols; ++c)
			{				
				slaveidx = itemLists[c]->index;
				if (tosend.count(slaveidx)>0
					&& (tosend[slaveidx].cur))
					break; //��ǰ�������д���������
			}
		}

		//���ж��о���
		coreMutex.lock(); //��ô���
		for(c = 0; c < cols; ++c)
		{
			slaveidx = itemLists[c]->index;
			seqLock[slaveidx].lock();
		}
		coreMutex.unlock(); //��ô���
		//�Ѿ�������еĶ�����
		
	//////////////////////////////////////////////////////////////////////////////////////

	for(size_t	c = 0; c < cols; ++c)
	{
		Item * toAppend = itemLists[c];
		int	slaveidx 	= toAppend->index;
		
		if (tosend[slaveidx].tail)
		{
			(tosend[slaveidx].tail)->next = NULL;
			m_garbageCollector.toss(tosend[slaveidx].head);
		}
		
		//empty and memory deleted
		tosend[slaveidx].head 	= toAppend;
		tosend[slaveidx].tail 	= toAppend->prev;
		tosend[slaveidx].cur 	= toAppend;
		tosend[slaveidx].count	= rows;
		tosend[slaveidx].keeprun= keep;
		if (keep)
		{
			adjusts[slaveidx].reset();
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////
	
		//�ͷ����еĶ�����
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

//����ֹͣ
void RdWrManager::declStop(int slaveidx, DeclStopInfo *stopInfo)
{
	tostop[slaveidx] = stopInfo;
}

void RdWrManager::declStopSync(DeclStopInfo **stopInfos, size_t cols)
{
	if (cols == 1)
	{//�����˶������С��
		declStop(stopInfos[0]->slaveIdx, stopInfos[0]);
	}
	else//����岹
	{
		//һ��Ϊһ�����ڣ�һ��Ϊһ����
		int c = 0;
		int slaveidx;

		//�ȴ���ǰ���ж���ȫ��������
		while(c < cols)
		{
			for(; c < cols; ++c)
			{				
				slaveidx = stopInfos[c]->slaveIdx;
				if (tosend.count(slaveidx)>0
					&& (tosend[slaveidx].cur))
					break; //��ǰ�������д���������
			}
		}

		//���ж��о���
		coreMutex.lock(); //��ô���
		for(c = 0; c < cols; ++c)
		{
			slaveidx = stopInfos[c]->slaveIdx;
			seqLock[slaveidx].lock();
		}
		coreMutex.unlock(); //��ô���
		//�Ѿ�������еĶ�����
		
	//////////////////////////////////////////////////////////////////////////////////////

		for(size_t	c = 0; c < cols; ++c)
		{
			//todo�رյ��ڹ���
			slaveidx = stopInfos[c]->slaveIdx;
			tostop[slaveidx] 	= stopInfos[c];
		}

	///////////////////////////////////////////////////////////////////////////////////////
	
		//�ͷ����еĶ�����
		coreMutex.lock(); 
		for(c = 0; c < cols; ++c)
		{
			slaveidx = stopInfos[c]->slaveIdx;
			seqLock[slaveidx].unlock();
		}
		coreMutex.unlock();
	}
}

void RdWrManager::setAdjust(short axis, short deltav, size_t cycles)
{
	seqLock[axis].lock();

	adjusts[axis].dVel = deltav;
	adjusts[axis].remainCount = cycles;
	
	seqLock[axis].unlock();
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

void RdWrManager::setRdParas(unsigned int batchwrite, unsigned int fifolw)
{
	BATCH_WRITE = batchwrite;
	FIFO_LOWATER= fifolw;
}

void RdWrManager::run()
{
	transData	cmdData[DEF_MA_MAX];
	transData	respData[DEF_MA_MAX];
		
	//��ʼ��FIFO״̬
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
				LOGSINGLE_WARNING("FIFO full.readCount=%d, fifoRemain=%?d.", __FILE__, __LINE__, rdWrState.readCount,  FIFO_REMAIN(respData));
			}

			//��������ֵ
			for(std::map<int, IoSlaveState>::iterator iter = ioState.begin();
						iter != ioState.end();
						++iter)
			{
				if (IO_RD == respData[iter->first].CMD)
					(iter->second).setInput(respData[iter->first].Data1);
			}

			DmcManager::instance().setRespData(respData);

			//check wether fifo is empty
			switch(m_flag)
			{
			case 0:
				if (m_consecutive)
					m_flag = 1;
				break;
			case 1:
				if (FIFO_REMAIN(respData) < ECM_FIFO_SIZE)
					m_flag = 2;
				break;
			case 2:
				if (FIFO_REMAIN(respData) == ECM_FIFO_SIZE)
				{
					ioState[8].setOutput(1<<8);
					LOGSINGLE_FATAL("readCount=%d, fifoRemain=%?d, boostcount=%?d.", __FILE__, __LINE__, rdWrState.readCount,  FIFO_REMAIN(respData), m_boostcount);
				}
				break;
			default:
				;
			}

			if (m_consecutive)
			{
				if (FIFO_REMAIN(respData) > FIFO_LOWATER)
				{
					goto SEND;
				}
			}
			else
			{
				if (FIFO_REMAIN(respData)>=ECM_FIFO_SIZE)
				{//����FIFO������˴�������������˶�������
					m_boostcount = 0;
					m_flag		 = 0;
					goto SEND;
				}
			}
		}while(true);

SEND:
		if (m_consecutive)
			m_towrite = BATCH_WRITE;
		else
			m_towrite = 2;
		++m_boostcount;
	};

	LOGSINGLE_INFORMATION("RdWrManager Thread canceled.%s", __FILE__, __LINE__, std::string(""));
}

