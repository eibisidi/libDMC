#include "RdWrManager.h"
#include "CLogSingle.h"
#include "DmcManager.h"

#define FIFO_REMAIN(respData) 	((respData)[0].Data2 & 0xFFFF)	//FIFOʣ��ռ�
#define FIFO_FULL(respData)		((respData)[0].Data2 >> 16)		//FIFO���Ĵ���
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)

#define BATCH_WRITE		(20)
#define FIFO_LOWATER	(32)			
#define ECM_FIFO_SIZE	(0xA0)				//ECM�ڲ�FIFO��Ŀ

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

void RdWrManager::cancel()				//ֹͣ�߳�
{
	m_canceled = true;

	setBusy();			//������У�����run�������޷��˳�
	
	m_thread.join();	//�ȴ�run��������
	clear();
}

void RdWrManager::clear()
{
	m_idle = true;
	m_canceled = false;
	
	for (int i = 0; i < DEF_MA_MAX; ++i)
	{
		queueFlags[i].store(false);	//�����г�ʼ״̬Ϊ����
	}

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
	LARGE_INTEGER frequency;									//��ʱ��Ƶ�� 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    	//��ʱ��Ƶ��   

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif

	int activeItems = 0;
	int slaveidx;
	bool qflag;
	
	memset(cmdData, 0, sizeof(transData)* cmdcount);
	
	m_mutex.lock();	
	
	for(std::map<int, ItemQueue *>::iterator iter = tosend.begin();
				iter!= tosend.end();
				++iter)
	{
		slaveidx = iter->first;
		qflag    = false;		//ϣ�����п���
		if (true == queueFlags[slaveidx].compare_exchange_weak(qflag, true))
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
			queueFlags[slaveidx].store(false);		//�ö��п���
		}
	}

	m_mutex.unlock();

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

//ÿ�д���һ�����ڣ�ÿ�д���һ����վ
void RdWrManager::pushItems(Item *items, int rows, size_t cols)
{
	//һ��Ϊһ�����ڣ�һ��Ϊһ����
	int c;
	int slaveidx;
	while(true)
	{
		m_mutex.lock();	

		for(c = 0; c < cols; ++c)
		{
			slaveidx = items[c].index;
			if (true == queueFlags[slaveidx].load())	//��ǰ������æ
			{
				break;
			}
		}

		if (c == cols)
		{//��ǰ���о�Ϊ����
			for(c = 0; c < cols; ++c)
			{
				slaveidx = items[c].index;
				queueFlags[slaveidx].store(true);		//��ǰ��������Ϊæ
			}
			m_mutex.unlock();
			break;
		}
		
		m_mutex.unlock();
		Poco::Thread::sleep(1);
	}
//////////////////////////////////////////////////////////////////////////////////////
	for (size_t i = 0; i < rows * cols; ++i)
	{
		slaveidx = items[i].index;
		if (0 == tosend.count(slaveidx))
			tosend[slaveidx] = new ItemQueue;
		tosend[slaveidx]->push_back(items[i]);
	}
///////////////////////////////////////////////////////////////////////////////////////
	//ÿ��������Ϊ����
	m_mutex.lock();	
	for(c = 0; c < cols; ++c)
	{
		slaveidx = items[c].index;
		queueFlags[slaveidx].store(false);		//��ǰ��������Ϊ����
	}
	m_mutex.unlock();
}

size_t RdWrManager::peekQueue(int slaveidx)
{
	size_t queuecount = 0;
	while(true)
	{
		m_mutex.lock(); 
		if (false == queueFlags[slaveidx].load())	//��ǰ���п���
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

//����ֹͣ
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

	std::map<int, ItemQueue *>::iterator iter = tosend.begin();
	for(; iter!= tosend.end(); ++iter)
	{
		if (iter->second 
			 && !(iter->second)->empty())
		{
			break;
		}
	}

	if (iter == tosend.end()) m_idle = true;				//����ǡ��Ϊ�գ��߳���Ϊ����
	m_mutex.unlock();
}

void RdWrManager::run()
{
	int 		towrite = BATCH_WRITE;
	transData	cmdData[DEF_MA_MAX];
	transData	respData[DEF_MA_MAX];
		
	//��ʼ��FIFO״̬
	if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
	{
		LOGSINGLE_FATAL("ECMUSBRead failed.%s", __FILE__, __LINE__, "");
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

			//������Ӧ����
			DmcManager::instance().setRespData(respData);

			switch(rdWrState.flag1)
			{
				case 0:
					if (FIFO_REMAIN(respData) < rdWrState.lastRemain)
					{
						rdWrState.flag1 = 1;
					}
					break;
				case 1://ʣ��FIFO����ʼ����
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
				case 2://ʣ��FIFO��������ֵ
					if (FIFO_REMAIN(respData) > FIFO_LOWATER)
					{
						rdWrState.flag1 = 3;
						rdWrState.lastRemain = FIFO_REMAIN(respData);
						goto SEND;
					}
					break;
				default:
					LOGSINGLE_FATAL("Unexpected case.%s", __FILE__, __LINE__, "");
			}
			
			rdWrState.lastRemain = FIFO_REMAIN(respData);

			if (rdWrState.flag1 && FIFO_REMAIN(respData) == ECM_FIFO_SIZE)
			{
				printf("FIFO EMPTY Error \n");
				LOGSINGLE_FATAL("FIFO empty.flag1=%d, readCount=%d, lastRemain=%?d, fifoRemain=%?d.", __FILE__, __LINE__,
								rdWrState.flag1, rdWrState.readCount, rdWrState.lastRemain, FIFO_REMAIN(respData));
				//keep running!!!
			}
		}while(rdWrState.readCount < 300);	//��ֹ������ѭ����������ȡ֮��������todo?

SEND:
		printf("last_remain=%d, fifo_remain=%d, readcount = %d, flag1=%d.\n", rdWrState.lastRemain, FIFO_REMAIN(respData), rdWrState.readCount, rdWrState.flag1);
		rdWrState.flag1 = 0;
		towrite = BATCH_WRITE;
	};

	LOGSINGLE_INFORMATION("RdWrManager Thread canceled.%s", __FILE__, __LINE__, "");
}

