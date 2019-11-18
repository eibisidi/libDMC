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
	m_thread.join();	//�ȴ�run��������
	clear();
}

void RdWrManager::clear()
{
	m_idle = true;
	m_canceled = false;
	memset(queueState, 0, sizeof(queueState));	

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
	
	memset(cmdData, 0, sizeof(transData)* cmdcount);
	
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
			{//ֹͣ�ô�վ
				int unsents = (iter->second)->size();
				
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
			queueState[slaveidx] = QUEUE_IDLE;
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
void RdWrManager::pushItems(Item *items, int rows, int cols)
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
			if (QUEUE_BUSY == queueState[slaveidx])
			{
				break;
			}
		}

		if (c == cols)
		{//��ǰ���о�Ϊ����
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

//����ֹͣ
void RdWrManager::declStop(int slaveidx, DeclStopInfo *stopInfo)
{
	m_mutex.lock(); 
	tostop[slaveidx] = stopInfo;
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
		
	//��ʼ��FIFO״̬
	if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
	{
		CLogSingle::logFatal("ECMUSBRead failed.", __FILE__, __LINE__);
		return;
	}

	unsigned int last_remain = ECM_FIFO_SIZE;
	unsigned int last_fifofull = FIFO_FULL(respData);
	
	int		flag1 = 0;		//1 �����������½�  2:��������������
	bool	bRet;
	int		readcount = 0;	//������ȡ����
	
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

		readcount = 0;
		do{
			do {
				bRet = ECMUSBRead((unsigned char*)respData, sizeof(respData));
				if (!bRet)
					printf("Read Error \n");
			}while(!bRet);

			++readcount;
			if (FIFO_FULL(respData) != last_fifofull)
			{
				printf("FIFO FULL Error \n");
				throw;
			}

			//������Ӧ����
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
						last_remain = FIFO_REMAIN(respData);
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
		}while(readcount < BATCH_WRITE);	//��ֹ������ѭ����������ȡ֮������

LABEL:
		//printf("to write = %d, last_remain=%d, fifo_remain=%d.\n", towrite, last_remain, FIFO_REMAIN(respData));
		flag1 = 0;
		towrite = BATCH_WRITE;
	};
}

