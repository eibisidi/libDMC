#include "RdManager.h"
#include "CLogSingle.h"
#include "DmcManager.h"


#define FIFO_REMAIN(respData) 	((respData)[0].Data2 & 0xFFFF)	//FIFOʣ��ռ�
#define FIFO_FULL(respData)		((respData)[0].Data2 >> 16)		//FIFO���Ĵ���
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)

#define ECM_FIFO_SIZE	(0xA0)				//ECM�ڲ�FIFO��Ŀ

RdManager::RdManager()
{
	clear();
}

RdManager::~RdManager()
{
}

void RdManager::addIoSlave(int slaveidx)
{
	ioState[slaveidx] = IoSlaveState();
}

void RdManager::start()
{
	m_thread.setPriority(Poco::Thread::PRIO_HIGHEST);
	m_thread.start(*this);
}

void RdManager::cancel()				//ֹͣ�߳�
{
	m_canceled = true;
	
	m_thread.join();	//�ȴ�run��������
	
	clear();
}


unsigned int RdManager::getIoInput(short slaveidx)
{
	return ioState[slaveidx].getInput();
}

void RdManager::clear()
{
}

RdManager & RdManager::instance()
{
	static RdManager _inst;
	return _inst;
}


void RdManager::run()
{
	transData	respData[DEF_MA_MAX];
		
	//��ʼ��FIFO״̬
	if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
	{
		LOGSINGLE_FATAL("ECMUSBRead failed.%s", __FILE__, __LINE__, std::string(""));
		return;
	}

	rdWrState.lastFifoFull	= FIFO_FULL(respData);
	rdWrState.fifoRemain	= FIFO_REMAIN(respData);
	
	bool	bRet;
	
	while(!m_canceled)
	{	
		do{
			bRet = ECMUSBRead((unsigned char*)respData, sizeof(respData));
			if (!bRet)
				LOGSINGLE_FATAL("Read Error.", __FILE__, __LINE__, std::string(""));
		}while(!bRet);

		rdWrState.fifoRemain = FIFO_REMAIN(respData);
		printf("Fifo remain=%d\n", rdWrState.fifoRemain);
		
		if (FIFO_FULL(respData) != rdWrState.lastFifoFull)
		{
			LOGSINGLE_FATAL("FIFO full.fifoRemain=%?d.", __FILE__, __LINE__, FIFO_REMAIN(respData));
		}

		if (rdWrState.consecutive && FIFO_REMAIN(respData) == ECM_FIFO_SIZE)
		{
			LOGSINGLE_FATAL("fifoRemain=%?d.", __FILE__, __LINE__, FIFO_REMAIN(respData));
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
	};

	LOGSINGLE_INFORMATION("RdWrManager Thread canceled.%s", __FILE__, __LINE__, std::string(""));
}

