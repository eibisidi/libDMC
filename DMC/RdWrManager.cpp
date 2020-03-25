#include "RdWrManager.h"
#include "CLogSingle.h"
#include "DmcManager.h"

#define FIFO_REMAIN(respData) 	((respData)[0].Data2 & 0xFFFF)	//FIFOʣ��ռ�
#define FIFO_FULL(respData)		((respData)[0].Data2 >> 16)		//FIFO���Ĵ���
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)

#define BATCH_WRITE		(10)
#define FIFO_LOWATER	(150)			
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
	m_thread.setPriority(Poco::Thread::PRIO_HIGHEST);
	m_thread.start(*this);
}

void RdWrManager::cancel()				//ֹͣ�߳�
{
	m_canceled = true;
	
	m_thread.join();	//�ȴ�run��������
	
	clear();
}

void RdWrManager::clear()
{
	m_canceled 		= false;
	m_consecutive	= false;
	m_towrite		= 2;

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
	ioState.clear();
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

	coreMutex.lock();//��ȡ���д���
	
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
			{//ֹͣ�ô�վ
				size_t unsents = (iter->second)->size();
				
				if (unsents > 0)
				{
					int estimate = (int)(tostop[slaveidx]->decltime * CYCLES_PER_SEC) + 1;					//Ԥ��ֹͣ��������
					int	q0, q1;
					if (unsents > 2 && estimate < unsents)
					{//���¼���һ����������
						q0 = (iter->second)->front().cmdData.Data1;
						(iter->second)->pop_front();
						q1 = (iter->second)->front().cmdData.Data1;

						(iter->second)->clear();		//�����ǰ���������д�����CSPĿ��λ��
	
						int     vel0 = (q1 > q0) ? (q1 - q0) : (q0 - q1);		//��ʼ�˶�����
						double  dec  = 1.0 * vel0 / estimate;					//���ٶ�
						int     dir  = (q1 > q0) ? 1 : -1;						//�˶�����
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
				{//��δ�˶�
					if (lastSent[slaveidx].CMD == CSP)
					{
						tostop[slaveidx]->valid = true;
						tostop[slaveidx]->endpos = lastSent[slaveidx].Data1;
					}
					else{
						tostop[slaveidx]->valid = false;
					}
				}

				//�Ƴ���ָֹͣ�� 
				tostop.erase(slaveidx);
			}
			////////////////////////////////////////////////////////////////////////////////////////////
			queueMutex[slaveidx].unlock();

			if (CSP == cmdData[slaveidx].CMD)
				con = true;
		}
	}

	coreMutex.unlock();//�ͷŶ��д���

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
			if(CSP != items[r*cols+c].cmdData.CMD)
				tosend[slaveidx]->clear();

			tosend[slaveidx]->push_back(items[r*cols + c]);
		}
		
		queueMutex[slaveidx].unlock();
	}
}


//ͬ������������У������ڶ����˶��������ԭ�㣬ÿ�����������һ��Ecm_write������
void RdWrManager::pushItemsSync(const Item *items, size_t rows, size_t cols)
{
	if (cols == 1)
	{//�����˶������С��
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
				slaveidx = items[c].index;
				if (tosend.count(slaveidx)>0
					&& (!tosend[slaveidx]->empty()))
					break; //��ǰ�������д���������
			}
			//Poco::Thread::sleep(1);
		}

		//���ж��о���
		c = 0;
		while(c < cols)
		{
			coreMutex.lock(); //��ô���
	
			for( ; c < cols; ++c)
			{
				slaveidx = items[c].index;
				if (false == queueMutex[slaveidx].tryLock())
					break; //��ǰ������æ
			}
			
			coreMutex.unlock();
			//Poco::Thread::sleep(1);
		}

		//�Ѿ�������еĶ�����
		
	//////////////////////////////////////////////////////////////////////////////////////
		for (size_t i = 0; i < rows * cols; ++i)
		{
			slaveidx = items[i].index;
			if (0 == tosend.count(slaveidx))
				tosend[slaveidx] = new ItemQueue;
			tosend[slaveidx]->push_back(items[i]);
		}
	///////////////////////////////////////////////////////////////////////////////////////
	
		//�ͷ����еĶ�����
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

	//queueMutex[slaveidx].lock();

	if (tosend.count(slaveidx))
		queuecount = tosend[slaveidx]->size();
	//queueMutex[slaveidx].unlock();

	return queuecount;
}

//����ֹͣ
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
				LOGSINGLE_FATAL("FIFO full.readCount=%d, fifoRemain=%?d.", __FILE__, __LINE__, rdWrState.readCount,  FIFO_REMAIN(respData));
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
			m_towrite = BATCH_WRITE;			//todo���Ƿ��޸�Ϊ����FIFO Remain���ӵ�������̬������һ��д�����������ǲ��ù̶�ֵ��
		else
			m_towrite = 2;
	};

	LOGSINGLE_INFORMATION("RdWrManager Thread canceled.%s", __FILE__, __LINE__, std::string(""));
}

