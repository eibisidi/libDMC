#include "DmcManager.h"
#include "DMC.h"
#include "CLogSingle.h"

#include "Poco/DOM/DOMParser.h"
#include "Poco/DOM/Document.h"
#include "Poco/DOM/NodeIterator.h"
#include "Poco/DOM/NodeFilter.h"
#include "Poco/DOM/AutoPtr.h"
#include "Poco/SAX/InputSource.h"
#include "Poco/DOM/NamedNodeMap.h"
#include "Poco/DOM/Attr.h"
#include "Poco/NumberParser.h"
#include "Poco/DOM/NodeList.h"

#include <fstream>
#include <assert.h>

#define  MAXJ_RATIO (5)			//���Ӽ��ٶȱ���


void ClearCmdData(transData* data)
{
	for (int i = 0; i < DEF_MA_MAX; i++)
	{
		data[i].CMD = data[i].Parm = 0;
		data[i].Data1 = data[i].Data2 = 0;
	}
}

bool checkResponse(char slaves, char column, unsigned int expectValue, unsigned int retry)
{
	unsigned int realValue = 0xFEDCBA98; //inital data
	unsigned int retried;
	transData respData[DEF_MA_MAX];
	for (retried = 0; retried <= retry; retried++)
	{
		ECMUSBRead((unsigned char*)respData, sizeof(respData));

		if (column == 0)
		if (slaves == DEF_MA_MAX - 1)
			realValue = respData[slaves].CMD;
		else
			realValue = respData[slaves].CMD & 0xFFFF;
		else if (column == 1)
		{
			if (slaves == 0)
				realValue = (respData[slaves].Parm) & 0xFF;
			else
				realValue = respData[slaves].Parm;
		}
		else if (column == 2)
			realValue = respData[slaves].Data1;
		else
			realValue = respData[slaves].Data2;

		if (realValue == expectValue)
		{
			break;
		}
		else if (retry == retried)
		{
			;//printf("Fail,realValue= %d   expectValue= %d, retry = %d \n", realValue, expectValue, retried);
		}
		else
		{
			ClearCmdData(respData);
			ECMUSBWrite((unsigned char*)respData, sizeof(respData));
		}
		Sleep(1);
	}
	return (realValue == expectValue);
}

bool SdoWriteResponse(char slaveidx, unsigned int dict, unsigned int retry, unsigned int value)
{
	unsigned int retried;
	transData respData[DEF_MA_MAX];
	ClearCmdData(respData);
	for (retried = 0; retried < retry; retried++)
	{
		if (!ECMUSBRead((unsigned char*)respData, sizeof(respData)))
			return false;

		if (respData[DEF_MA_MAX-1].CMD == SDO_WR
			&& respData[DEF_MA_MAX-1].Parm == slaveidx
			&& respData[DEF_MA_MAX-1].Data1 == dict
			&& respData[DEF_MA_MAX-1].Data2 == value
			)
		{
			return true;
		}

		ClearCmdData(respData);
		if (!ECMUSBWrite((unsigned char*)respData, sizeof(respData)))
			return false;
	}
	
	return false;
}

bool SdoReadResponse(char slaveidx, unsigned int dict, unsigned int retry, unsigned int *value)
{
	unsigned int retried;
	transData respData[DEF_MA_MAX];
	for (retried = 0; retried < retry; retried++)
	{
		ECMUSBRead((unsigned char*)respData, sizeof(respData));

		if (respData[DEF_MA_MAX-1].CMD == SDO_RD
			&& respData[DEF_MA_MAX-1].Parm == slaveidx
			&& respData[DEF_MA_MAX-1].Data1 == dict
			)
		{
			*value = respData[DEF_MA_MAX-1].Data2;
			return true;
		}
			
		ClearCmdData(respData);
		ECMUSBWrite((unsigned char*)respData, sizeof(respData));
	}
	
	return false;
}

DmcManager & DmcManager::instance()
{
	static DmcManager _inst;
	return _inst;
}

DmcManager::DmcManager()
{
	clear();
}

DmcManager::~DmcManager()
{
	close();
}

void	DmcManager::clear()
{
	m_init = false;
	m_canceled = false;
	m_sdoOwner = NULL;
	newRespData = false;
	memset(m_cmdData, 0, sizeof(m_cmdData));
	memset(m_lastCmdData, 0, sizeof(m_lastCmdData));
	memset(m_respData, 0, sizeof(m_respData));
	memset(m_realRespData, 0, sizeof(m_respData));

	memset(m_slaveType, None, sizeof(m_slaveType));

	for(map<int, BaseRequest *>::const_iterator iter = m_requests.begin();
					iter != m_requests.end();
					++iter)
	{
		if (iter->second)
			delete iter->second;
	}
	m_requests.clear();

	m_masterConfig.clear();


	for(std::map<int, SlaveState *>::iterator iter = m_slaveStates.begin();
					iter != m_slaveStates.end();
					++iter)
	{
		delete (iter->second);
		iter->second = NULL;
	}
	m_slaveStates.clear();
}

unsigned long DmcManager::init()
{
	//��ʼ����־
	CLogSingle::initLogger();

	LOGSINGLE_INFORMATION("libDMC ver 1.0. buildtime : %s %s.", __FILE__, __LINE__, std::string(__DATE__), std::string(__TIME__));

	if (m_init)
	{	
		LOGSINGLE_ERROR("init already called.%s", __FILE__, __LINE__, std::string(""));
		return ERR_DUP_INIT;
	}

	//��ȡ������Ϣ
	if (!loadXmlConfig())
	{
		LOGSINGLE_ERROR("loadXmlConfig failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_LOAD_XML;
	}

	//������־��¼�ȼ�
	CLogSingle::setLogLevel(m_masterConfig.loglevel, !m_masterConfig.logpoint_axis.empty());

	if (!::OpenECMUSB())
	{
		LOGSINGLE_ERROR("OpenECMUSB failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_OPENUSB;
	}

	LOGSINGLE_INFORMATION("OpenECMUSB success.%s", __FILE__, __LINE__, std::string(""));

	Sleep(100);

	//set state to pre-OP
	::ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_PRE_OP; 
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		LOGSINGLE_ERROR("ECMUSBWrite failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_WRITE;
	}

	if (!::checkResponse(0, 1, STATE_PRE_OP, 1000))
	{
		LOGSINGLE_ERROR("checkResponse failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_PREOP;
	}

	//set axis
	ClearCmdData(m_cmdData);
	unsigned int topology;
	int j = 0;
	for (int i = 1; i <= 5; i++)
	{
		topology = 0;
		for (; j < i * 8; j++)
		{
			topology |= (m_slaveType[j] << (j % 8) * 4);
		}
		m_cmdData[0].CMD = SET_AXIS;
		m_cmdData[0].Parm = i - 1; //Group
		m_cmdData[0].Data1 = topology;
		m_cmdData[0].Data2 = 0;
		if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
		{
			LOGSINGLE_ERROR("ECMUSBWrite failed.%s", __FILE__, __LINE__, std::string(""));
			return ERR_ECM_WRITE;
		}
		Sleep(2);
	}

	//set dc
	m_cmdData[0].CMD = SET_DC;
	m_cmdData[0].Parm = 0;
	m_cmdData[0].Data1 = DC_CYCLE_us; 	// set cycle time to 2000 us
	m_cmdData[0].Data2 = 0xFFFF; 		// Auto offet adjustment

	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		LOGSINGLE_ERROR("ECMUSBWrite failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_WRITE;
	}
	Sleep(2);
	if (!::ECMUSBRead((unsigned char*)m_respData, sizeof(m_respData)))
	{
		LOGSINGLE_ERROR("ECMUSBRead failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_READ;
	}

	//Set DRIVE_MODE
	ClearCmdData(m_cmdData);
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (m_slaveType[i - 1] == DRIVE
			|| STEP == m_slaveType[i - 1])
		{
			m_cmdData[i].CMD = DRIVE_MODE;
			m_cmdData[i].Data1 = CSP_MODE; // set to CSP mode
			m_cmdData[i].Data2 = DCSYNC; // set using DC sync
		}
	}
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		LOGSINGLE_ERROR("ECMUSBWrite failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_WRITE;
	}

	Sleep(2);

	// set state to Safe-OP
	ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_SAFE_OP;
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		LOGSINGLE_ERROR("ECMUSBWrite failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_WRITE;
	}
	if(!::checkResponse(0, 1, STATE_SAFE_OP, 1000))  // wait for enter to safe op mode
	{
		LOGSINGLE_ERROR("checkResponse failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_SAFEOP;
	}
	
	// set state to OP
	ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_OPERATIONAL;
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		LOGSINGLE_ERROR("ECMUSBWrite failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_WRITE;
	}

	if (!::checkResponse(0, 1, STATE_OPERATIONAL, 1000))// wait for enter to op mode
	{
		LOGSINGLE_ERROR("checkResponse failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_ECM_OP;
	}

	initSlaveState();

	//�����߳�
	m_rdWrManager.start();
	
	m_thread.setPriority(Poco::Thread::PRIO_NORMAL);
	m_thread.start(*this);

	//��������������澯
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (m_slaveType[i - 1] == DRIVE || m_slaveType[i - 1] == STEP)	
		{
			if (ERR_NOERR != clr_alarm(i))
			{
				LOGSINGLE_FATAL("clr_alarm for axis %d failed.", __FILE__, __LINE__, i);
				return ERR_CLR_ALARM;
			}
		}
	}

	//������������ʼ�������ò���
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (DRIVE == m_slaveType[i - 1]
			|| STEP == m_slaveType[i - 1])	
		{
			if (ERR_NOERR != init_driver(i))
			{
				LOGSINGLE_FATAL("init_driver for axis %d failed.", __FILE__, __LINE__, i);
				return ERR_INIT_AXIS;
			}
		}
	}

	//��������������
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (m_slaveType[i - 1] == DRIVE || m_slaveType[i - 1] == STEP)  
		{
			if (ERR_NOERR != servo_on(i))
			{
				LOGSINGLE_FATAL("servo_on for axis %d failed.", __FILE__, __LINE__, i);
				return ERR_SERVO_ON;
			}
		}
	}
	
	m_init = true;

	return ERR_NOERR;
}

void DmcManager::close()
{
	if (!m_init)
		return;
	
	m_canceled = true;

	m_thread.join();

	m_rdWrManager.cancel();		//ֹͣ�߳�

	::ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_INIT; 
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		LOGSINGLE_ERROR("ECMUSBWrite failed.%s", __FILE__, __LINE__, std::string(""));
	}
	::CloseECMUSB();

	
	CLogSingle::closeLogger();

	clear();
}

void DmcManager::logCspPoints(const Item *pItems, int rows, size_t cols) const
{
	if (m_masterConfig.logpoint_axis.empty())
		return;

	for (int r = 0; r < rows; ++r)
	{
		//�������˳������滮λ��
		for (std::set<int>::const_iterator iter = m_masterConfig.logpoint_axis.begin();
											iter != m_masterConfig.logpoint_axis.end();
											++iter)
		{
			for(int c = 0; c < cols; ++c)
			{
				if(pItems[r*cols + c].index == *iter)
				{
					CLogSingle::logPoint((int)(pItems[r*cols+c].cmdData.Data1));
					break;
				}
			}
		}
		CLogSingle::logPoint("\n");
	}
}


void DmcManager::beforeWriteCmd()
{
#ifdef TIMING
	LARGE_INTEGER frequency;								//��ʱ��Ƶ�� 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    //��ʱ��Ƶ��   

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif

	memset(m_cmdData, 0, sizeof(m_cmdData));
	for (map<int,BaseRequest *>::const_iterator iter = m_requests.begin();
			iter != m_requests.end();
			)
	{
		int slave_idx = iter->first;
		
		BaseRequest *req 	= iter->second;
		FsmRetType retval 	= req->exec();
		
		if (REQUEST_STATE_BUSY != req->reqState)
		{
			freeSdoCmdResp(req);
			delete req;
			iter = m_requests.erase(iter);
		}
		else
			iter++;

		m_slaveStates[slave_idx]->setSlaveState(retval);	//����״̬������ֵ��������ǰ״̬

		if (m_cmdData[slave_idx].CMD != GET_STATUS)
		{
			m_lastCmdData[slave_idx] = m_cmdData[slave_idx];
		}

	}

#ifdef TIMING
	QueryPerformanceCounter(&timeEnd); 
	elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 
	printf("time elapsed = %f\n", elapsed);
#endif
}

void  DmcManager::initSlaveState()
{
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (m_slaveType[i - 1] == DRIVE || m_slaveType[i - 1] == STEP)  
			m_slaveStates[i] = new DriverSlaveState;
		else if (m_slaveType[i - 1] == IO)
			m_slaveStates[i] = new IoSlaveState;
	}
}

bool DmcManager::loadXmlConfig()
{
	std::ifstream in("Master.xml");
	Poco::XML::InputSource src(in);
	try
	{
		Poco::XML::DOMParser parser;
		Poco::AutoPtr<Poco::XML::Document> pDoc = parser.parse(&src);
		
		Poco::XML::NodeIterator it(pDoc, Poco::XML::NodeFilter::SHOW_ELEMENT | Poco::XML::NodeFilter::SHOW_ATTRIBUTE);
		Poco::XML::Node* pNode = it.nextNode();
		while (pNode)
		{
			if ("Slave" == pNode->nodeName())
			{//��վ����
				Poco::AutoPtr<Poco::XML::NamedNodeMap> pAttrs = pNode->attributes();
				Poco::XML::Attr* pAttr = static_cast<Poco::XML::Attr*>(pAttrs->getNamedItem("index"));

				int slave_index = Poco::NumberParser::parse(pAttr->nodeValue());
				unsigned char slave_type  = None;
				pAttr = static_cast<Poco::XML::Attr*>(pAttrs->getNamedItem("type"));
				if ("Drive" == pAttr->nodeValue())
					slave_type = DRIVE;
				else if ("Step" == pAttr->nodeValue())
					slave_type = STEP;
				else if ("IO" == pAttr->nodeValue())
					slave_type = IO;
				else
				{
					LOGSINGLE_ERROR("XML Error, Slave %d, Unknown type = %s.", __FILE__, __LINE__, slave_index, pAttr->nodeValue());
					return false;
				}


				SlaveConfig sc(slave_index, slave_type);
				//���������ӽڵ�
				Poco::AutoPtr<Poco::XML::NodeList > pChilds = pNode->childNodes();

				for (int i = 0; i < pChilds->length(); ++i)
				{
					Poco::XML::Node *pChild =  pChilds->item(i);
					if (Poco::XML::Node::ELEMENT_NODE != pChild->nodeType())
						continue;

					Poco::AutoPtr<Poco::XML::NamedNodeMap> pChildAttrs = pChild->attributes();
					Poco::XML::Attr* pIndexAttr = static_cast<Poco::XML::Attr*>(pChildAttrs->getNamedItem("index"));
					Poco::XML::Attr* pSubIndexAttr = static_cast<Poco::XML::Attr*>(pChildAttrs->getNamedItem("subindex"));
					Poco::XML::Attr* pSizeAttr = static_cast<Poco::XML::Attr*>(pChildAttrs->getNamedItem("size"));
					Poco::XML::Attr* pValueAttr = static_cast<Poco::XML::Attr*>(pChildAttrs->getNamedItem("value"));
					
					SDO sdo;
					sdo.sdo_index = Poco::NumberParser::parseHex(pIndexAttr->nodeValue());
					sdo.sdo_subindex = Poco::NumberParser::parseHex(pSubIndexAttr->nodeValue());
					sdo.sdo_size = Poco::NumberParser::parse(pSizeAttr->nodeValue());

					if (!Poco::NumberParser::tryParse(pValueAttr->nodeValue(), sdo.sdo_value))
					{//ֵʮ���ƽ���ʧ�ܺ�ʹ��16����������
						sdo.sdo_value = Poco::NumberParser::parseHex(pValueAttr->nodeValue());
					}

					sc.slave_sdos.push_back(sdo);
				}

				if (0 != m_masterConfig.slave_indexes.count(sc.slave_index))
				{
					LOGSINGLE_ERROR("XML Error, duplicate slave index %d.", __FILE__, __LINE__, sc.slave_index);
					return false;
				}

				m_masterConfig.slave_indexes.insert(sc.slave_index);
				m_masterConfig.slave_configs.push_back(sc);
			}
			else if ("LogLevel" == pNode->nodeName())
			{//��־��¼�ȼ�
				Poco::AutoPtr<Poco::XML::NodeList > pChilds = pNode->childNodes();				
				for (int i = 0; i < pChilds->length(); ++i)
				{
					Poco::XML::Node *pChild =  pChilds->item(i);
					if (Poco::XML::Node::TEXT_NODE != pChild->nodeType())
						continue;
					m_masterConfig.loglevel = Poco::NumberParser::parse(pChild->nodeValue());
				}
			}
			else if ("LogPoint" == pNode->nodeName())
			{//��¼�滮��
				Poco::AutoPtr<Poco::XML::NamedNodeMap> pAttrs = pNode->attributes();
				Poco::XML::Attr* pAttr = static_cast<Poco::XML::Attr*>(pAttrs->getNamedItem("enable"));

				if (pAttr && "On" == pAttr->nodeValue())
				{
					//��������<Axis>�ӽڵ�
					Poco::AutoPtr<Poco::XML::NodeList > pChilds = pNode->childNodes();
	
					for (int i = 0; i < pChilds->length(); ++i)
					{
						Poco::XML::Node *pChild =  pChilds->item(i);
						if (Poco::XML::Node::ELEMENT_NODE != pChild->nodeType())
							continue;
						if ("Axis" != pChild->nodeName())
							continue;
						
						Poco::AutoPtr<Poco::XML::NamedNodeMap> pChildAttrs = pChild->attributes();
						Poco::XML::Attr* pIndexAttr = static_cast<Poco::XML::Attr*>(pChildAttrs->getNamedItem("index"));

						if (pIndexAttr)
						{
							int axis = Poco::NumberParser::parse(pIndexAttr->nodeValue());
							m_masterConfig.logpoint_axis.insert(axis);
						}
					}
				}
			}
			else if ("HomeMethod" == pNode->nodeName())
			{//���㷽ʽ
				Poco::AutoPtr<Poco::XML::NodeList > pChilds = pNode->childNodes();				
				for (int i = 0; i < pChilds->length(); ++i)
				{
					Poco::XML::Node *pChild =  pChilds->item(i);
					if (Poco::XML::Node::TEXT_NODE != pChild->nodeType())
						continue;
					m_masterConfig.home_method = Poco::NumberParser::parse(pChild->nodeValue());
				}
			}
			else if ("HomeTimeout" == pNode->nodeName())
			{//���㳬ʱʱ��
				Poco::AutoPtr<Poco::XML::NodeList > pChilds = pNode->childNodes();				
				for (int i = 0; i < pChilds->length(); ++i)
				{
					Poco::XML::Node *pChild =  pChilds->item(i);
					if (Poco::XML::Node::TEXT_NODE != pChild->nodeType())
						continue;
					m_masterConfig.home_timeout = Poco::NumberParser::parse(pChild->nodeValue());
				}
			}
			else if ("ServoPosBias" == pNode->nodeName())
			{//�ŷ�λ�ôﵽ���������Χ
				Poco::AutoPtr<Poco::XML::NodeList > pChilds = pNode->childNodes();				
				for (int i = 0; i < pChilds->length(); ++i)
				{
					Poco::XML::Node *pChild =  pChilds->item(i);
					if (Poco::XML::Node::TEXT_NODE != pChild->nodeType())
						continue;
					m_masterConfig.servo_pos_bias = Poco::NumberParser::parse(pChild->nodeValue());
				}
			}
			pNode = it.nextNode();
		}
	}
	catch (Poco::Exception& exc)
	{
		LOGSINGLE_ERROR("XML Exception, %s.", __FILE__, __LINE__, exc.displayText());
		return false;
	}

	//���ô�վ������
	for(int i = 0; i < m_masterConfig.slave_configs.size(); ++i)
	{
		m_slaveType[m_masterConfig.slave_configs[i].slave_index - 1] = m_masterConfig.slave_configs[i].slave_type; 
	}
	
	return true;
}

long DmcManager::getDriverCmdPos(short slaveidx)
{
	long cmdpos = 0;

	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
		cmdpos = dss->getCmdPos();
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);

	return cmdpos;
}

void DmcManager::setDriverCmdPos(short slaveidx, long val)
{
	long cmdpos = 0;

	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
		dss->setCmdPos(val);
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);
}

int DmcManager::getServoPosBias() const
{
	return m_masterConfig.servo_pos_bias;
}

void DmcManager::updateState()
{
	//����������״̬
	unsigned CMD;
	for(std::map<int, SlaveState *>::iterator iter = m_slaveStates.begin();
				iter != m_slaveStates.end();
				++iter)
	{
		int		slaveidx = iter->first;
		
		CMD =((m_respData[slaveidx].CMD & 0xFF));
		
		if (GET_STATUS == CMD)
			CMD = m_lastCmdData[slaveidx].CMD;

		if (isDriverSlave(slaveidx))
		{//Driver
			DriverSlaveState *dss = dynamic_cast<DriverSlaveState*>(iter->second);
			switch(CMD)
			{
				case ALM_CLR:
					dss->setStatus(m_respData[slaveidx].Parm);
					break;
				case CSP:
				case GO_HOME:
				case ABORT_HOME:
					dss->setStatus(m_respData[slaveidx].Parm);
					dss->setCurPos(m_respData[slaveidx].Data1);
					dss->setAlarmCode(m_respData[slaveidx].Data2 >> 16);	//����Ϊ�澯��
					if (dss->getAlarmCode())
						dss->setSlaveState(MOVESTATE_ERR);
					break;
				case SV_ON:
					dss->setStatus(m_respData[slaveidx].Parm);
					dss->setCurPos(m_respData[slaveidx].Data1);
					break;
				case SV_OFF:
					dss->setStatus(m_respData[slaveidx].Parm);
					break;
				case DRIVE_MODE:
					break;
				default:
					;
			}
		}
		else if(isIoSlave(slaveidx))
		{
			IoSlaveState *iss = dynamic_cast<IoSlaveState*>(iter->second);
			switch(CMD)
			{
				case IO_RD:
					iss->setInput(m_respData[slaveidx].Data1);
					break;
				default:
					;
			}
		}
	}

}

bool DmcManager::isDriverSlave(short slaveidx) const
{
	if (slaveidx > 0 && slaveidx < DEF_MA_MAX - 1)
	{
		return (DRIVE == m_slaveType[slaveidx - 1] || STEP == m_slaveType[slaveidx - 1]);
	}
	return false;
}

bool DmcManager::isIoSlave(short slaveidx) const
{
	if (slaveidx > 0 && slaveidx < DEF_MA_MAX - 1)
	{
		return (IO == m_slaveType[slaveidx - 1]);
	}
	return false;
}

void DmcManager::setSlaveState(short slaveidx, unsigned int  ss)
{
	assert(m_slaveStates.count(slaveidx) > 0);
	m_slaveStates[slaveidx]->setSlaveState(ss);
}

void DmcManager::addRequest(short slaveidx, BaseRequest *req)
{
	assert (NULL != req);
	if (m_requests.count(slaveidx) > 0)
		delete m_requests[slaveidx];
	m_requests[slaveidx] = req;
	m_condition.signal();
}

long DmcManager::getCurpos(short slaveidx)
{
	long curpos = 0;

	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
		curpos = dss->getCurPos();
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);

	return curpos;
}

transData * DmcManager::getCmdData(short slaveidx)
{
	assert (slaveidx >= 0 && slaveidx < DEF_MA_MAX - 1);
	return &(m_cmdData[slaveidx]);
}

transData * DmcManager::getRespData(short slaveidx)
{
	assert (slaveidx >= 0 && slaveidx < DEF_MA_MAX - 1);
	return &(m_respData[slaveidx]);
}

bool	DmcManager::getSdoCmdResp(BaseRequest *req, transData **ppCmd, transData **ppResp)
{
	assert (NULL != ppCmd);
	assert (NULL != ppResp);
	if (NULL == m_sdoOwner 
		|| m_sdoOwner == req)
	{
		m_sdoOwner = req;
		*ppCmd = &m_cmdData[DEF_MA_MAX - 1];
		*ppResp = &m_respData[DEF_MA_MAX - 1];
		return true;
	}
	else
		return false;
}

void DmcManager::freeSdoCmdResp(BaseRequest *req)
{
	if (m_sdoOwner == req)
		m_sdoOwner = NULL;
}

void DmcManager::restoreLastCmd(transData *cmdData)
{
	int slaveidx;
	slaveidx = (int)(cmdData - m_cmdData);
	*cmdData = m_lastCmdData[slaveidx];
}

void DmcManager::setRespData(transData *respData)
{
#ifdef TIMING
	static double longest = 0;
	static double shortest = 1000000;
	static double total = 0;
	static int	  count = 0;
	LARGE_INTEGER frequency;									//��ʱ��Ƶ�� 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000; 	//��ʱ��Ƶ��	

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif

	m_mutexRespData.lock();

	memcpy(m_realRespData, respData, sizeof(m_realRespData));
	newRespData = true;
	m_conditionRespData.signal();
	
	m_mutexRespData.unlock();

#ifdef TIMING
	QueryPerformanceCounter(&timeEnd); 
	elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 
	
	total += elapsed;
	if(elapsed > longest)
		longest = elapsed;
	if (elapsed < shortest)
		shortest = elapsed;
	++count;
	printf("setRespData elapsed = %f, longest=%f, shortest=%f, average=%f.\n", elapsed, longest, shortest, total/count);
#endif
}

void DmcManager::copyRespData()
{
#if TIMING
	LARGE_INTEGER frequency;								//��ʱ��Ƶ�� 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    //��ʱ��Ƶ��   

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif
	m_mutexRespData.lock();
	
	while(!newRespData)
		m_conditionRespData.wait(m_mutexRespData);

	memcpy(m_respData, m_realRespData, sizeof(m_realRespData));
	newRespData = false;

	m_mutexRespData.unlock();

	updateState();

#if TIMING
	QueryPerformanceCounter(&timeEnd); 
	elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 
	printf("time elapsed = %f\n", elapsed);
#endif
}

bool DmcManager::isDriverOpCsp(short slaveidx)
{
	OpMode op = OPMODE_NONE;

	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
		op = dss->getOpMode();
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);

	return (OPMODE_CSP == op);
}

bool DmcManager::isDriverOn(short slaveidx)
{
	unsigned short status = 0;
	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
		status = dss->getStatus();
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);

	return (0x1637 == status
		|| 0x9637 == status);		//ԭ�����ҵ�
}

bool DmcManager::isDriverOff(short slaveidx)
{
	unsigned short status = 0;
	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
		status = dss->getStatus();
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);
	//��ͱ�����1����������׼���á�
	return (0 == (0x01 & status ));
}

unsigned short DmcManager::getDriverStatus(short slaveidx)
{
	unsigned short status = 0;
	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
		status = dss->getStatus();
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);
	//��ͱ�����1����������׼���á�
	return (status);
}

bool DmcManager::isDriverHomed(short slaveidx)
{
	unsigned short status = 0;
	long 			curpos = 0;
	DriverSlaveState * dss = dynamic_cast<DriverSlaveState *>(m_slaveStates[slaveidx]);
	if (dss)
	{
		status = dss->getStatus();
		curpos = dss->getCurPos();
	}
	else
		LOGSINGLE_FATAL("slave %?d is not a driver.", __FILE__, __LINE__, slaveidx);

	assert(dss != NULL);
	//��ͱ�����1����������׼���á�
	return ( (0x9637 == status || 0x1637 == status)
		&& 0 == curpos);
}

bool DmcManager::isServo(short slaveidx)
{
	assert (slaveidx > 0 && slaveidx < DEF_MA_MAX - 1);
	return DRIVE == m_slaveType[slaveidx - 1];				//�ŷ����
}

void DmcManager::setIoOutput(short slaveidx, unsigned int output)
{
	IoSlaveState * iss = dynamic_cast<IoSlaveState *>(m_slaveStates[slaveidx]);
	if (iss)
		iss->setOutput(output);
	else
		LOGSINGLE_FATAL("slave %?d is not a io.", __FILE__, __LINE__, slaveidx);
}

unsigned int DmcManager::getIoOutput(short slaveidx)
{
	unsigned int output = 0;

	IoSlaveState * iss = dynamic_cast<IoSlaveState *>(m_slaveStates[slaveidx]);
	if (iss)
		output = iss->getOutput();
	else
		LOGSINGLE_FATAL("slave %?d is not a io.", __FILE__, __LINE__, slaveidx);

	return output;
}

unsigned int DmcManager::getIoInput(short slaveidx)
{
	unsigned int input = 0;

	IoSlaveState * iss = dynamic_cast<IoSlaveState *>(m_slaveStates[slaveidx]);
	if (iss)
		input = iss->getInput();
	else
		LOGSINGLE_FATAL("slave %?d is not a io.", __FILE__, __LINE__, slaveidx);

	return input;
}

void  DmcManager::flipread(short slaveidx)
{
	bool shouldread = false;
	IoSlaveState * iss = dynamic_cast<IoSlaveState *>(m_slaveStates[slaveidx]);
	if (iss)
		shouldread = iss->flip();
	else
	{
		LOGSINGLE_FATAL("slave %?d is not a io.", __FILE__, __LINE__, slaveidx);
		return;
	}

	if (shouldread)
	{
		m_items[m_cols].index = slaveidx;
		m_items[m_cols].cmdData.CMD	= IO_RD;
		m_items[m_cols].cmdData.Data1 = 0;
		m_items[m_cols].cmdData.Data2 = 0;
	}
	else
	{
		m_items[m_cols].index = slaveidx;
		m_items[m_cols].cmdData.CMD	= IO_WR;
		m_items[m_cols].cmdData.Data1 = getIoOutput(slaveidx);
		m_items[m_cols].cmdData.Data2 = 0;
	}

	m_lastCmdData[slaveidx] = m_items[m_cols].cmdData;
	++m_cols;
} 

void DmcManager::run()
{	
	int i;

	while(!m_canceled)
	{		
		m_mutex.lock();
		if(!m_requests.empty())
		{
			beforeWriteCmd();
			m_mutex.unlock();

			for(i = 0, m_cols = 0; i < DEF_MA_MAX; ++i)
			{
				if (isIoSlave(i))
				{//pigtails io_rd 
					flipread(i);
				}
				else if (m_cmdData[i].CMD != GET_STATUS)
				{
					if (m_cmdData[i].CMD == CSP && 0 == m_cmdData[i].Data2)
						continue;
					if (m_cmdData[i].CMD == CSP)
						m_cmdData[i].Data2 = 0;			//CSP���λ���ط�
					m_items[m_cols].index = i;
					m_items[m_cols].cmdData = m_cmdData[i];
					++m_cols;
				}		
			}

			if (m_cols > 0)
				m_rdWrManager.pushItems(m_items, 1, m_cols);	//���뷢�Ͷ���

			m_rdWrManager.setBusy();
			copyRespData();//���ն��д���ˢ��
		}
		else
		{
			m_rdWrManager.setIdle();
			bool wake = m_condition.tryWait(m_mutex, 10);
			m_mutex.unlock();
			if (false == wake)	//����10ms���ȴ��û�����
			{//polls current inputs if idle

				for(i = 0, m_cols = 0; i < DEF_MA_MAX; ++i)
				{
					if(isIoSlave(i))
					{
						flipread(i);
					}
				}
				if (m_cols > 0)
					m_rdWrManager.pushItems(m_items, 1, m_cols);	//���뷢�Ͷ���

				m_rdWrManager.setBusy();
				copyRespData();//���ն��д���ˢ��
			}
		}

	}

	LOGSINGLE_INFORMATION("DmcManager Thread canceled.%s", __FILE__, __LINE__, std::string(""));
}

unsigned long DmcManager::clr_alarm(short axis)
{
	unsigned long retValue = ERR_NOERR;
	ClrAlarmRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if (false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (m_requests.count(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}

		newReq = new ClrAlarmRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		newReq->slave_idx = axis;

		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);		
	}while(0);

	m_mutex.unlock();

	unsigned long ms;
	if (ERR_NOERR == retValue)
	{
		while(true)
		{
			ms = d1000_check_done(axis);				
			if (MOVESTATE_BUSY != ms)
				break;
		}
	}

	if (MOVESTATE_STOP == ms)
	{
		retValue = ERR_NOERR;
		LOGSINGLE_INFORMATION("clr_alarm axis(%?d) succeed, status = 0x%?x.", __FILE__, __LINE__, axis, getDriverStatus(axis));
	}
	else
	{
		retValue = ERR_CLR_ALARM;
		LOGSINGLE_ERROR("clr_alarm axis(%?d) failed.",	__FILE__, __LINE__, axis);
	}

	return retValue;
		
}

unsigned long DmcManager::init_driver(short axis)
{
	unsigned long retValue = ERR_NOERR;
	InitSlaveRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (m_requests.count(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}

		newReq = new InitSlaveRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		newReq->slave_idx = axis;

		for(int i = 0; i < m_masterConfig.slave_configs.size(); ++i)
		{
			if (axis == m_masterConfig.slave_configs[i].slave_index)
			{
				newReq->iter	  = m_masterConfig.slave_configs[i].slave_sdos.begin();
				newReq->iterEnd   = m_masterConfig.slave_configs[i].slave_sdos.end();
				break;
			}
		}
		
		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);		
	}while(0);

	m_mutex.unlock();

	unsigned long ms;
	if (ERR_NOERR == retValue)
	{
		while(true)
		{
			ms = d1000_check_done(axis);				
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (MOVESTATE_STOP == ms)
		{
			retValue = ERR_NOERR;
			LOGSINGLE_INFORMATION("init_driver axis(%?d) succeed.", __FILE__, __LINE__, axis);
		}
		else
		{
			retValue = ERR_CLR_ALARM;
			LOGSINGLE_ERROR("init_driver axis(%?d) failed.",	__FILE__, __LINE__, axis);
		}
	}
	return retValue;		
}

unsigned long DmcManager::servo_on(short axis)
{
	unsigned long retValue = ERR_NOERR;
	ServoOnRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if (false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (m_requests.count(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}

		newReq = new ServoOnRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		newReq->slave_idx = axis;

		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);		
	}while(0);

	m_mutex.unlock();

	unsigned long ms;
	if (ERR_NOERR == retValue)
	{
		while(true)
		{
			ms = d1000_check_done(axis); 				
			if (MOVESTATE_BUSY != ms)
				break;
		}
	}

	if (MOVESTATE_STOP == ms)
	{
		retValue = ERR_NOERR;
		LOGSINGLE_INFORMATION("servo_on axis(%?d) succeed, cmdpos = %?d.", __FILE__, __LINE__, axis, getDriverCmdPos(axis));
	}
	else
	{
		retValue = ERR_SERVO_ON;
		LOGSINGLE_ERROR("servo_on axis(%?d) failed.",  __FILE__, __LINE__, axis);
	}

	return retValue;
		
}

unsigned long DmcManager::start_move(short axis,long Dist,double MaxVel,double Tacc, bool abs, MoveType movetype)
{
	if (Tacc < 1E-6
		|| MaxVel < 1E-6)
		return ERR_INVALID_ARG;

	unsigned long retValue = ERR_NOERR;
	MoveRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if (false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (m_requests.count(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}

		//�ж�Ŀ��λ���Ƿ��Ѿ�����
		if (!abs && Dist == 0)
		{
			setSlaveState(axis, MOVESTATE_STOP);
			break;						//���ģʽ������Ϊ0
		}

		if (abs && Dist == getDriverCmdPos(axis))
		{
			setSlaveState(axis, MOVESTATE_STOP);
			break;						//����ģʽ��λ���ѵ���
		}
	
		newReq = new MoveRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		setSlaveState(axis, MOVESTATE_BUSY);
		
		newReq->slave_idx = axis;
		newReq->abs 	  = abs;				//��� ����
		newReq->dist	  = Dist;
		newReq->movetype  = movetype;			//S / T
		newReq->maxvel	  = MaxVel;				//����ٶ�
		newReq->maxa	  = MaxVel / Tacc;		//�����ٶ�
		newReq->maxj	  = MAXJ_RATIO *(newReq->maxa);
		addRequest(axis, newReq);		
	}while(0);

	m_mutex.unlock();

	return retValue;
}

unsigned long DmcManager::home_move(short axis,long highVel,long lowVel,long acc)
{
	if (acc <= 0
		|| highVel <= 0
		|| lowVel <= 0)
		return ERR_INVALID_ARG;

	unsigned long retValue = ERR_NOERR;
	HomeMoveRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (m_requests.count(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}

		newReq = new HomeMoveRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		setSlaveState(axis, MOVESTATE_BUSY);
		
		newReq->slave_idx = axis;
		newReq->home_method = m_masterConfig.home_method;
		newReq->high_speed  = highVel;
		newReq->low_speed	= lowVel;
		newReq->acc			= acc;	
		newReq->home_timeout= m_masterConfig.home_timeout;
		addRequest(axis, newReq);
	}while(0);

	m_mutex.unlock();

	return retValue;
}

unsigned long DmcManager::multi_home_move(short totalAxis, short * axisArray)
{
	if (totalAxis <= 0
		|| NULL ==  axisArray)
		return ERR_INVALID_ARG;

	unsigned long					retValue = ERR_NOERR;
	MultiHomeRequest 				*newReq = NULL;
	MultiHomeRequest::MultiHomeRef	*newRef = NULL;
	
	m_mutex.lock();
	
	//�Ƚ����ݴ���
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		if (m_requests.count(axis))
		{
			LOGSINGLE_ERROR("Axis %?d is busy.", __FILE__, __LINE__, axis);
			retValue = ERR_AXIS_BUSY;
			goto DONE;
		}
	}

	newRef = new MultiHomeRequest::MultiHomeRef;
	if(NULL == newRef)
	{
		retValue = ERR_MEM;
		goto DONE;
	}

//�½�����
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		newReq = new MultiHomeRequest(axis, newRef, m_masterConfig.home_timeout);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}
DONE:
	m_condition.signal();

	m_mutex.unlock();

	return retValue;
}

unsigned long DmcManager::start_line(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs, MoveType movetype)
{
	if (totalAxis <= 0
		|| Tacc < 1E-6
		|| maxvel < 1E-6)
		return ERR_INVALID_ARG;

	unsigned long 	retValue = ERR_NOERR;
	MultiAxisRequest *newReq = NULL;
	LinearRef		 *newLinearRef = NULL;
	
	m_mutex.lock();
	
	//�Ƚ����ݴ���
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		if (m_requests.count(axis))
		{
			LOGSINGLE_ERROR("Axis %?d is busy.", __FILE__, __LINE__, axis);
			retValue = ERR_AXIS_BUSY;
			goto DONE;
		}
	}

	newLinearRef = new LinearRef;
	if(NULL == newLinearRef)
	{
		retValue = ERR_MEM;
		goto DONE;
	}

	newLinearRef->maxvel = maxvel;
	newLinearRef->maxa	  = maxvel / Tacc;
	newLinearRef->maxj	  = MAXJ_RATIO * newLinearRef->maxa;
	newLinearRef->movetype  = movetype;

//�½�����
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		newReq = new MultiAxisRequest(axis, newLinearRef, distArray[i], abs);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}
DONE:
	m_condition.signal();

	m_mutex.unlock();

	return retValue;
		
}

unsigned long DmcManager::start_archl(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs,  long hh, long hu, long hd)
{
	if (totalAxis <= 0
			|| Tacc < 1E-6
			|| maxvel < 1E-6
			|| hu < 0
			|| hd < 0)
			return ERR_INVALID_ARG;
	
	unsigned long	retValue = ERR_NOERR;
	MultiAxisRequest *newReq = NULL;
	ArchlRef		 *newArchlRef = NULL;
	
	m_mutex.lock();
	
	//�Ƚ����ݴ���
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		if (m_requests.count(axis))
		{
			retValue = ERR_AXIS_BUSY;
			goto DONE;
		}
	}

	newArchlRef = new ArchlRef;
	if(NULL == newArchlRef)
	{
		retValue = ERR_MEM;
		goto DONE;
	}

	newArchlRef->maxvel = maxvel;
	newArchlRef->maxa	= maxvel / Tacc;
	newArchlRef->hu		= hu;
	newArchlRef->hh		= hh;
	newArchlRef->hd		= hd;
	
	//�½�����
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		newReq = new MultiAxisRequest(axis, newArchlRef, distArray[i], abs, (i==0)/*�Ƿ�ΪZ��*/);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}
DONE:
	m_condition.signal();

	m_mutex.unlock();

	return retValue;
}

unsigned long DmcManager::check_done(short axis)
{
	unsigned int ms = MOVESTATE_NONE;

	assert(m_slaveStates.count(axis) > 0);
	ms = m_slaveStates[axis]->getSlaveState();

	//Poco::Thread::sleep(1);

	return ms;
}

long DmcManager::get_command_pos(short axis)
{
	long retpos = 0;
	if (isDriverSlave(axis))
	{
		retpos = getDriverCmdPos(axis);
	}

	return retpos;
}

unsigned long DmcManager::decel_stop(short axis, double tDec, bool bServOff)
{
	assert(tDec > 1E-6);
	unsigned long retValue = ERR_NOERR;
	DStopRequest *newReq = NULL;

	m_mutex.lock();
	do{
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (0 == m_requests.count(axis))
		{//��ǰδ���˶�
			retValue = ERR_AXIS_NOT_MOVING;
			break;
		}

		MoveRequest *moveReq = dynamic_cast<MoveRequest * >(m_requests[axis]);
		if (NULL != moveReq)
		{//�����˶�
			newReq = new DStopRequest(axis, tDec, bServOff);
			setSlaveState(axis, MOVESTATE_BUSY);
			addRequest(axis, newReq);
		}
		else
		{
			MultiAxisRequest *maReq = dynamic_cast<MultiAxisRequest * >(m_requests[axis]);
			if (NULL != maReq)
			{//�����˶�
				const std::map<int, BaseMultiAxisPara *> paras = maReq->axispara->ref->paras;	//makes a copy
				for (std::map<int, BaseMultiAxisPara *>::const_iterator iter = paras.begin();
							iter != paras.end();
							++iter)
				{
					BaseMultiAxisPara *para = iter->second;

					newReq = new DStopRequest(para->req->slave_idx, tDec, bServOff);
					
					setSlaveState(para->req->slave_idx, MOVESTATE_BUSY);
					addRequest(para->req->slave_idx, newReq);
					maReq = NULL;				//no longer valid
				}
				
			}
			else
			{//��ǰδ���˶�
				retValue = ERR_AXIS_NOT_MOVING;
				break;
			}
		}
	}while(0);
	m_mutex.unlock();
	return retValue;
}

unsigned long DmcManager::immediate_stop(short axis)
{
	return decel_stop(axis, 0.1, true);
}

unsigned long DmcManager::out_bit(short slave_idx, short bitNo, short bitData)
{
	unsigned long retValue = ERR_NOERR;
	
	do{
		if ( false == isIoSlave(slave_idx))
		{
			retValue = ERR_IO_NO_SLAVE;
			break;
		}

		unsigned int cur_output = getIoOutput(slave_idx);

		if (bitData)	//Turn on
			cur_output = cur_output | (1 << bitNo);
		else			//Turn off
			cur_output = cur_output &(~(1 << bitNo));

		setIoOutput(slave_idx, cur_output);
	}while(0);
	return retValue;
}

unsigned long DmcManager::in_bit(short slave_idx, unsigned int *bitData)
{
	unsigned long retValue = ERR_NOERR;
	
	do{
		if ( false == isIoSlave(slave_idx))
		{
			retValue = ERR_IO_NO_SLAVE;
			break;
		}

		*bitData = getIoInput(slave_idx);
	}while(0);

	return retValue;
}

