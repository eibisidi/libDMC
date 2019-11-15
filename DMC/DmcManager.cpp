#include "DmcManager.h"
#include "DMC.h"
#include <assert.h>
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


#define  MAXJ_RATIO (5)			//最大加加速度倍率


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
			// printf("PASS\n");
			break;
		}
		else if (retry == retried)
		{
			printf("Fail,realValue= %d   expectValue= %d, retry = %d \n", realValue, expectValue, retried);
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
	m_masterState.clear();
	m_driverState.clear();
	m_ioState.clear();
}

unsigned long DmcManager::init()
{
	//初始化日志
	CLogSingle::initLogger();

	CLogSingle::logInformation("libDMC ver 1.0. buildtime : %s %s.", __FILE__, __LINE__, std::string(__DATE__), std::string(__TIME__));

	if (m_init)
	{	
		CLogSingle::logError("init already called.", __FILE__, __LINE__);
		return ERR_DUP_INIT;
	}

	//读取配置信息
	if (!loadXmlConfig())
	{
		CLogSingle::logError("loadXmlConfig failed.", __FILE__, __LINE__);
		return ERR_LOAD_XML;
	}

	//设置日志记录等级
	CLogSingle::setLogLevel(m_masterConfig.loglevel, !m_masterConfig.logpoint_axis.empty());

	if (!::OpenECMUSB())
	{
		CLogSingle::logError("OpenECMUSB failed.", __FILE__, __LINE__);
		return ERR_OPENUSB;
	}

	CLogSingle::logInformation("OpenECMUSB success.", __FILE__, __LINE__);

	Sleep(100);

	//set state to pre-OP
	::ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_PRE_OP; 
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		CLogSingle::logError("ECMUSBWrite failed.", __FILE__, __LINE__);
		return ERR_ECM_WRITE;
	}

	if (!::checkResponse(0, 1, STATE_PRE_OP, 1000))
	{
		CLogSingle::logError("checkResponse failed.", __FILE__, __LINE__);
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
			CLogSingle::logError("ECMUSBWrite failed.", __FILE__, __LINE__);
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
		CLogSingle::logError("ECMUSBWrite failed.", __FILE__, __LINE__);
		return ERR_ECM_WRITE;
	}
	Sleep(2);
	if (!::ECMUSBRead((unsigned char*)m_respData, sizeof(m_respData)))
	{
		CLogSingle::logError("ECMUSBRead failed.", __FILE__, __LINE__);
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
		CLogSingle::logError("ECMUSBWrite failed.", __FILE__, __LINE__);
		return ERR_ECM_WRITE;
	}

	Sleep(2);

	// set state to Safe-OP
	ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_SAFE_OP;
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		CLogSingle::logError("ECMUSBWrite failed.", __FILE__, __LINE__);
		return ERR_ECM_WRITE;
	}
	if(!::checkResponse(0, 1, STATE_SAFE_OP, 1000))  // wait for enter to safe op mode
	{
		CLogSingle::logError("checkResponse failed.", __FILE__, __LINE__);
		return ERR_ECM_SAFEOP;
	}
	
	// set state to OP
	ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_OPERATIONAL;
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		CLogSingle::logError("ECMUSBWrite failed.", __FILE__, __LINE__);
		return ERR_ECM_WRITE;
	}

	if (!::checkResponse(0, 1, STATE_OPERATIONAL, 1000))// wait for enter to op mode
	{
		CLogSingle::logError("checkResponse failed.", __FILE__, __LINE__);
		return ERR_ECM_OP;
	}

	initSlaveState();

	//启动线程
	RdWrManager::instance().start();
	
	m_thread.setPriority(Poco::Thread::PRIO_NORMAL);
	m_thread.start(*this);

	//所有驱动器清除告警
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (m_slaveType[i - 1] == DRIVE || m_slaveType[i - 1] == STEP)	
		{
			if (ERR_NOERR != clr_alarm(i))
			{
				CLogSingle::logFatal("clr_alarm for axis %d failed.", __FILE__, __LINE__, i);
				return ERR_CLR_ALARM;
			}
		}
	}

	//所有驱动器开始设置配置参数
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (DRIVE == m_slaveType[i - 1]
			|| STEP == m_slaveType[i - 1])	
		{
			if (ERR_NOERR != init_driver(i))
			{
				CLogSingle::logFatal("init_driver for axis %d failed.", __FILE__, __LINE__, i);
				return ERR_INIT_AXIS;
			}
		}
	}

	//所有驱动器励磁
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (m_slaveType[i - 1] == DRIVE || m_slaveType[i - 1] == STEP)  
		{
			if (ERR_NOERR != servo_on(i))
			{
				CLogSingle::logFatal("servo_on for axis %d failed.", __FILE__, __LINE__, i);
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

	::ClearCmdData(m_cmdData);
	m_cmdData[0].CMD = SET_STATE;
	m_cmdData[0].Data1 = STATE_INIT; 
	if (!::ECMUSBWrite((unsigned char*)m_cmdData, sizeof(m_cmdData)))
	{
		CLogSingle::logError("ECMUSBWrite failed.", __FILE__, __LINE__);
	}
	::CloseECMUSB();

	
	CLogSingle::closeLogger();

	clear();
}

void DmcManager::beforeWriteCmd()
{
#ifdef TIMING
	LARGE_INTEGER frequency;								//计时器频率 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000;    //计时器频率   

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
		
		BaseRequest *req = iter->second;
		req->exec();

		if (REQUEST_STATE_BUSY != req->reqState)
		{
			freeSdoCmdResp(req);
			delete req;
			iter = m_requests.erase(iter);
		}
		else
			iter++;

		if (m_cmdData[slave_idx].CMD != GET_STATUS)
		{
			m_lastCmdData[slave_idx] = m_cmdData[slave_idx];

			if (!m_masterConfig.logpoint_axis.empty())
			{//开启了规划记录
				std::set<int>::reverse_iterator riter = m_masterConfig.logpoint_axis.rbegin();
				if (slave_idx == *riter 					//序号最大的轴，开始一行
					&& m_cmdData[slave_idx].CMD == CSP)
				{
					std::string line;
					for (std::set<int>::iterator iter = m_masterConfig.logpoint_axis.begin();
									iter != m_masterConfig.logpoint_axis.end();
									++iter)
					{
						//line += Poco::format("%d    ",  (int)m_cmdData[*iter].Data1);
						CLogSingle::logPoint((int)m_cmdData[*iter].Data1);
						//!!!!!!!!!!!!!!!!
						//m_cmdData[*iter].CMD = GET_STATUS;
						//ofs << (int)m_cmdData[*iter].Data1 << "    " ;
					}
					CLogSingle::logPoint("\n");
				}
			}

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
	m_driverState.clear();
	for (int i = 1; i < DEF_MA_MAX - 1; i++)
	{
		if (m_slaveType[i - 1] == DRIVE || m_slaveType[i - 1] == STEP)  
			m_driverState[i] = DriverState();
		else if (m_slaveType[i - 1] == IO)
			m_ioState[i] = IoState();
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
			{//从站配置
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
					CLogSingle::logError("XML Error, Slave %d, Unknown type = %s.", __FILE__, __LINE__, slave_index, pAttr->nodeValue());
					return false;
				}


				SlaveConfig sc(slave_index, slave_type);
				//处理所有子节点
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
					sdo.sdo_value = Poco::NumberParser::parse(pValueAttr->nodeValue());
					sc.slave_sdos.push_back(sdo);
				}

				if (0 != m_masterConfig.slave_indexes.count(sc.slave_index))
				{
					CLogSingle::logError("XML Error, duplicate slave index %d.", __FILE__, __LINE__, sc.slave_index);
					return false;
				}

				m_masterConfig.slave_indexes.insert(sc.slave_index);
				m_masterConfig.slave_configs.push_back(sc);
			}
			else if ("LogLevel" == pNode->nodeName())
			{//日志记录等级
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
			{//记录规划点
				Poco::AutoPtr<Poco::XML::NamedNodeMap> pAttrs = pNode->attributes();
				Poco::XML::Attr* pAttr = static_cast<Poco::XML::Attr*>(pAttrs->getNamedItem("enable"));

				if (pAttr && "On" == pAttr->nodeValue())
				{
					//处理所有<Axis>子节点
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
			{//回零方式
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
			{//回零超时时间
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
			{//伺服位置达到检测允许误差范围
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
	catch (Exception& exc)
	{
		CLogSingle::logError("XML Exception, %s.", __FILE__, __LINE__, exc.displayText());
		return false;

	}

	//设置从站的类型
	for(int i = 0; i < m_masterConfig.slave_configs.size(); ++i)
	{
		m_slaveType[m_masterConfig.slave_configs[i].slave_index - 1] = m_masterConfig.slave_configs[i].slave_type; 
	}
	
	return true;
}

long DmcManager::getDriverCmdPos(short slaveidx)
{
	assert(m_driverState.count(slaveidx) > 0);

	return m_driverState[slaveidx].cmdpos;
}

void DmcManager::setDriverCmdPos(short slaveidx, long val)
{
	assert(m_driverState.count(slaveidx) > 0);
	m_driverState[slaveidx].cmdpos = val;
}

int DmcManager::getServoPosBias() const
{
	return m_masterConfig.servo_pos_bias;
}

void DmcManager::updateState()
{
	#if 0
	//检查FIFO是否被覆盖
	if(m_masterState.fifoFull != FIFO_FULL(m_respData))
	{
		CLogSingle::logFatal("FIFO overwritten. lastfifofull = %?d, fifofull=%?d.", __FILE__, __LINE__, m_masterState.fifoFull, FIFO_FULL(m_respData));
		//throw;
		//assert(0);
	}
	
	if (FIFO_REMAIN(m_respData) == ECM_FIFO_SIZE)
	{
		printf("FIFO is empty."); 
		m_masterState.fifoEmptyCount++;
	}
	else
		m_masterState.fifoEmptyCount = 0;

	if (FIFO_REMAIN(m_respData) > m_masterState.fifoRemain)
		m_masterState.fifoIncre = true;		//FIFO剩余空间开始增加，开始净消耗
	else
		m_masterState.fifoIncre = false;


	
	//printf("fiforemain=%d, incre = %d, empty=%d.\n", FIFO_REMAIN(m_respData), (int)m_masterState.fifoIncre, m_masterState.fifoEmptyCount);
	
	m_masterState.state		 = m_respData[0].Parm & 0xFF;	//更新主站状态
	m_masterState.errorcode	 = m_respData[0].Parm >> 8;
	m_masterState.fifoFull = FIFO_FULL(m_respData);
	m_masterState.fifoRemain = FIFO_REMAIN(m_respData);

#endif
	//更新驱动器状态
	unsigned CMD;
	for (map<int, DriverState>::iterator iter = m_driverState.begin();
					iter != m_driverState.end();
					++iter)
	{
		int		slaveidx = iter->first;
		CMD =((m_respData[slaveidx].CMD & 0xFF));

		if (GET_STATUS == CMD)
			CMD = m_lastCmdData[slaveidx].CMD;

		switch(CMD)
		{
			case ALM_CLR:
				(iter->second).status = m_respData[slaveidx].Parm;
				break;
			case CSP:
			case SV_ON:
			case GO_HOME:
			case ABORT_HOME:
				//(iter->second).opmode = (OpMode)(m_respData[slaveidx].CMD >> 8);
				(iter->second).status = m_respData[slaveidx].Parm;
				(iter->second).curpos = m_respData[slaveidx].Data1;
				break;
			case SV_OFF:
				//(iter->second).opmode = (OpMode)(m_respData[slaveidx].CMD >> 8);
				(iter->second).status = m_respData[slaveidx].Parm;
				break;
			case DRIVE_MODE:
				//(iter->second).opmode = (OpMode)(m_respData[slaveidx].CMD >> 8);
				break;
			default:
				;
		}
	}

	//更新IO状态
	for (map<int, IoState>::iterator iter = m_ioState.begin();
					iter != m_ioState.end();
					++iter)
	{
		int		slaveidx = iter->first;
		CMD =((m_respData[slaveidx].CMD & 0xFF));

		if (GET_STATUS == CMD)
			CMD = m_lastCmdData[slaveidx].CMD;
		switch(CMD)
		{
			case IO_RD:
				(iter->second).input = m_respData[slaveidx].Data1;
				break;
			default:
				;
		}
	}
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
	assert(m_driverState.count(slaveidx) > 0);
	return m_driverState[slaveidx].curpos;
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
	slaveidx = cmdData - m_cmdData;
	*cmdData = m_lastCmdData[slaveidx];
}

void DmcManager::setRespData(transData *respData)
{
#if 0
		LARGE_INTEGER frequency;								//计时器频率 
		QueryPerformanceFrequency(&frequency);	 
		double quadpart = (double)frequency.QuadPart / 1000000;    //计时器频率   
	
		LARGE_INTEGER timeStart, timeEnd;
		double elapsed;
		QueryPerformanceCounter(&timeStart); 
#endif

	m_mutexRespData.lock();
	//printf("setRespData signaled\n");

	memcpy(m_realRespData, respData, sizeof(m_realRespData));
	newRespData = true;
	m_conditionRespData.signal();
	m_mutexRespData.unlock();

#if 0
		QueryPerformanceCounter(&timeEnd); 
		elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 
		printf("time elapsed = %f\n", elapsed);
#endif

}

void DmcManager::copyRespData()
{

#if 0
		LARGE_INTEGER frequency;								//计时器频率 
		QueryPerformanceFrequency(&frequency);	 
		double quadpart = (double)frequency.QuadPart / 1000000;    //计时器频率   
	
		LARGE_INTEGER timeStart, timeEnd;
		double elapsed;
		QueryPerformanceCounter(&timeStart); 
#endif


	bool wakedup = false;
	m_mutexRespData.lock();
	//printf("copyRespData\n");
	
	while(!newRespData)
		//wakedup = m_conditionRespData.tryWait(m_mutexRespData, 10);
		m_conditionRespData.wait(m_mutexRespData);

	//if(wakedup)
	{
		memcpy(m_respData, m_realRespData, sizeof(m_realRespData));
		newRespData = false;
	}
	m_mutexRespData.unlock();

	//if (wakedup)
		updateState();

#if 0
			QueryPerformanceCounter(&timeEnd); 
			elapsed = (timeEnd.QuadPart - timeStart.QuadPart) / quadpart; 
			printf("time elapsed = %f\n", elapsed);
#endif

}



bool DmcManager::isDriverOpCsp(short slaveidx)
{
	assert (m_driverState.count(slaveidx) > 0);
	return (OPMODE_CSP == m_driverState[slaveidx].opmode);
}

bool DmcManager::isDriverOn(short slaveidx)
{
	assert (m_driverState.count(slaveidx) > 0);
	
	return (0x1637 == m_driverState[slaveidx].status
		|| 0x9637 == m_driverState[slaveidx].status);		//原点已找到
}

bool DmcManager::isDriverOff(short slaveidx)
{
	assert (m_driverState.count(slaveidx) > 0);
#if 0

	if (DRIVE == m_slaveType[slaveidx - 1])
		return (0x0250 == m_driverState[slaveidx].status || 0x8250 == m_driverState[slaveidx].status); //伺服sv_off状态字为 0x0250 , 步进状态字为 0x670
	else
		return (0x0250 == m_driverState[slaveidx].status || 0x670 == m_driverState[slaveidx].status);
#endif
	//最低比特置1，代表电机已准备好。
	return (0 == (0x01 & m_driverState[slaveidx].status ));
}

unsigned short DmcManager::getDriverStatus(short slaveidx)
{
	assert (m_driverState.count(slaveidx) > 0);
	return m_driverState[slaveidx].status;
}

bool DmcManager::isDriverHomed(short slaveidx)
{
	assert (m_driverState.count(slaveidx) > 0);
	return ( (0x9637 == m_driverState[slaveidx].status || 0x1637 == m_driverState[slaveidx].status)
		&& 0 == m_driverState[slaveidx].curpos);
}

bool DmcManager::isServo(short slaveidx)
{
	assert (slaveidx > 0 && slaveidx < DEF_MA_MAX - 1);
	return DRIVE == m_slaveType[slaveidx - 1];				//伺服电机
}

void DmcManager::setMoveState(short slaveidx, MoveState ms)
{
	assert(m_driverState.count(slaveidx) > 0);
	m_driverState[slaveidx].movestate = ms;
}

void DmcManager::setIoRS(short slavidx, IoRequestState iors)
{
	assert(m_ioState.count(slavidx) > 0);
	m_ioState[slavidx].iors = iors;
}

void DmcManager::run()
{	
	int i, cols;
	while(!m_canceled)
	{		
		while(!m_requests.empty())
		{
			m_mutex.lock();
			beforeWriteCmd();

			for(i = 0, cols = 0; i < DEF_MA_MAX; ++i)
			{
				if (m_cmdData[i].CMD == GET_STATUS
					|| (m_cmdData[i].CMD == CSP && m_cmdData[i].Data2 == 0))
					continue;

				if (m_cmdData[i].CMD == CSP)
					m_cmdData[i].Data2 = 0;			//CSP最后位置重发
			
				m_items[cols].index = i;
				m_items[cols].cmdData = m_cmdData[i];
				++cols;
			}

			if (cols > 0)
				RdWrManager::instance().pushItems(m_items, 1, cols);

			copyRespData();

			m_mutex.unlock();

			Poco::Thread::sleep(10);
		}

		RdWrManager::instance().setIdle();

		m_mutex.lock();
		m_condition.tryWait(m_mutex, 10);	//休眠10ms
		m_mutex.unlock();
		//Poco::Thread::yield();	//让出CPU避免连续获得锁//m_thread.wakeUp();	
	}

	CLogSingle::logInformation("Thread canceled.", __FILE__, __LINE__);
}

unsigned long DmcManager::clr_alarm(short axis)
{
	unsigned long retValue = ERR_NOERR;
	ClrAlarmRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if ( 0 == m_driverState.count(axis))
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

		setMoveState(axis, MOVESTATE_BUSY);
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
		CLogSingle::logInformation("clr_alarm axis(%?d) succeed, status = 0x%?x.", __FILE__, __LINE__, axis, getDriverStatus(axis));
	}
	else
	{
		retValue = ERR_CLR_ALARM;
		CLogSingle::logError("clr_alarm axis(%?d) failed.",	__FILE__, __LINE__, axis);
	}

	return retValue;
		
}

unsigned long DmcManager::init_driver(short axis)
{
	unsigned long retValue = ERR_NOERR;
	InitSlaveRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if ( 0 == m_driverState.count(axis))
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
		
		setMoveState(axis, MOVESTATE_BUSY);
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
			CLogSingle::logInformation("init_driver axis(%?d) succeed.", __FILE__, __LINE__, axis);
		}
		else
		{
			retValue = ERR_CLR_ALARM;
			CLogSingle::logError("init_driver axis(%?d) failed.",	__FILE__, __LINE__, axis);
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
		if ( 0 == m_driverState.count(axis))
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

		setMoveState(axis, MOVESTATE_BUSY);
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
		CLogSingle::logInformation("servo_on axis(%?d) succeed, cmdpos = %?d.", __FILE__, __LINE__, axis, getDriverCmdPos(axis));
	}
	else
	{
		retValue = ERR_SERVO_ON;
		CLogSingle::logError("servo_on axis(%?d) failed.",  __FILE__, __LINE__, axis);
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
	//printf("[%d] abspos = %d, curpos = %d.\n", axis, getDriverCmdPos(axis), getCurpos(axis));

	do{
		if ( 0 == m_driverState.count(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (m_requests.count(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}

		//判断目标位置是否已经到达
		if (!abs && Dist == 0)
		{
			setMoveState(axis, MOVESTATE_STOP);
			break;						//相对模式，距离为0
		}

		if (abs && Dist == getDriverCmdPos(axis))
		{
			setMoveState(axis, MOVESTATE_STOP);
			break;						//绝对模式，位置已到达
		}
	
		newReq = new MoveRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		setMoveState(axis, MOVESTATE_BUSY);
		
		newReq->slave_idx = axis;
		newReq->abs 	  = abs;				//相对 绝对
		newReq->dist	  = Dist;
		newReq->movetype  = movetype;			//S / T
		newReq->maxvel	  = MaxVel;				//最大速度
		newReq->maxa	  = MaxVel / Tacc;		//最大加速度
		newReq->maxj	  = MAXJ_RATIO *(newReq->maxa);
		addRequest(axis, newReq);		
	}while(0);

	m_mutex.unlock();

	return retValue;
		
}

unsigned long DmcManager::home_move(short axis,long highVel,long lowVel,double Tacc)
{
	if (Tacc < 1E-6
		|| highVel <= 0
		|| lowVel <= 0)
		return ERR_INVALID_ARG;

	unsigned long retValue = ERR_NOERR;
	HomeMoveRequest *newReq = NULL;
	
	m_mutex.lock();

	do{
		if ( 0 == m_driverState.count(axis))
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

		setMoveState(axis, MOVESTATE_BUSY);
		
		newReq->slave_idx = axis;
		newReq->home_method = m_masterConfig.home_method;
		newReq->high_speed  = highVel;
		newReq->low_speed	= lowVel;
		newReq->acc			= (unsigned int )(highVel / Tacc);	
		newReq->home_timeout= m_masterConfig.home_timeout;
		addRequest(axis, newReq);
	}while(0);

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
	
	//先进行容错检查
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		if ( 0 == m_driverState.count(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		if (m_requests.count(axis))
		{
			CLogSingle::logError("Axis %?d is busy.", __FILE__, __LINE__, axis);
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

//新建请求
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		newReq = new MultiAxisRequest(axis, newLinearRef, distArray[i], abs);
		setMoveState(axis, MOVESTATE_BUSY);
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
			|| maxvel < 1E-6)
			return ERR_INVALID_ARG;
	
	unsigned long	retValue = ERR_NOERR;
	MultiAxisRequest *newReq = NULL;
	ArchlRef		 *newArchlRef = NULL;
	
	m_mutex.lock();
	
	//先进行容错检查
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		if ( 0 == m_driverState.count(axis))
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
	
	//新建请求
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		newReq = new MultiAxisRequest(axis, newArchlRef, distArray[i], abs, (i==0)/*是否为Z轴*/);
		setMoveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}
DONE:
	m_condition.signal();

	m_mutex.unlock();

	return retValue;
}

unsigned long DmcManager::check_done(short axis)
{
	MoveState ms = MOVESTATE_NONE;
	m_mutex.lock();
	if (m_driverState.count(axis))
		ms = m_driverState[axis].movestate;
	m_mutex.unlock();
	Poco::Thread::sleep(10);

	return ms;
}

long DmcManager::get_command_pos(short axis)
{
	long retpos = 0;
	m_mutex.lock();

	if (m_driverState.count(axis))
		retpos = m_driverState[axis].cmdpos;

	m_mutex.unlock();
	return retpos;
}

unsigned long DmcManager::decel_stop(short axis, double tDec, bool bServOff)
{
	assert(tDec > 1E-6);
	unsigned long retValue = ERR_NOERR;
	DStopRequest *newReq = NULL;
	double curspeed;
	int    curpos ;
	bool	moving = false;

	m_mutex.lock();
	do{
		if ( 0 == m_driverState.count(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (0 == m_requests.count(axis))
		{//当前未在运动
			retValue = ERR_AXIS_NOT_MOVING;
			break;
		}

		MoveRequest *moveReq = dynamic_cast<MoveRequest * >(m_requests[axis]);
		if (NULL != moveReq)
		{//单轴运动
			newReq = new DStopRequest(axis, tDec, bServOff);
			setMoveState(axis, MOVESTATE_BUSY);
			addRequest(axis, newReq);
		}
		else
		{
			MultiAxisRequest *lineReq = dynamic_cast<MultiAxisRequest * >(m_requests[axis]);
			if (NULL != lineReq)
			{//多轴运动
				const std::set<BaseMultiAxisPara *> paras = lineReq->axispara->ref->paras;	//makes a copy
				for (std::set<BaseMultiAxisPara *>::const_iterator iter = paras.begin();
							iter != paras.end();
							++iter)
				{
					BaseMultiAxisPara *para = *iter;

					newReq = new DStopRequest(para->req->slave_idx, tDec, bServOff);
					
					setMoveState(para->req->slave_idx, MOVESTATE_BUSY);
					addRequest(para->req->slave_idx, newReq);
					lineReq = NULL;				//no longer valid
				}
				
			}
			else
			{//当前未在运动
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

unsigned long DmcManager::out_bit(short slave_idx, unsigned int bitData)
{
	unsigned long retValue = ERR_NOERR;
	WriteIoRequest *newReq = NULL;
	m_mutex.lock();
	
	do{
		if ( 0 == m_ioState.count(slave_idx))
		{
			retValue = ERR_IO_NO_SLAVE;
			break;
		}

		if (m_requests.count(slave_idx) > 0)
		{
			retValue = ERR_IO_BUSY;
			break;
		}
		
		newReq = new WriteIoRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		setIoRS(slave_idx, IORS_BUSY);
		newReq->slave_idx = slave_idx;
		newReq->output	  = bitData;
		addRequest(slave_idx, newReq);
	}while(0);

	m_mutex.unlock();

	IoRequestState iors;
	if (ERR_NOERR == retValue)
	{
		//wait until done
		while(true)
		{
			m_mutex.lock();
			iors = m_ioState[slave_idx].iors;
			if (IORS_SUCCESS == iors )
			{
				m_mutex.unlock();
				break;
			}
			else if (IORS_TIMEOUT  == iors)
			{
				retValue = ERR_IO_WRITE_TIMEOUT;
				m_mutex.unlock();
				break;
			}
			
			m_mutex.unlock();
			
			Poco::Thread::sleep(10);
		}
	}
	return retValue;
}

unsigned long DmcManager::in_bit(short slave_idx, unsigned int *bitData)
{
	unsigned long retValue = ERR_NOERR;
	ReadIoRequest *newReq = NULL;
	m_mutex.lock();
	
	do{
		if ( 0 == m_ioState.count(slave_idx))
		{
			retValue = ERR_IO_NO_SLAVE;
			break;
		}

		if (m_requests.count(slave_idx) > 0)
		{
			retValue = ERR_IO_BUSY;
			break;
		}
		
		newReq = new ReadIoRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}

		setIoRS(slave_idx, IORS_BUSY);
		newReq->slave_idx = slave_idx;
		addRequest(slave_idx, newReq);

	}while(0);

	m_mutex.unlock();

	IoRequestState iors;
	if (ERR_NOERR == retValue)
	{
		//wait until done
		while(true)
		{
			m_mutex.lock();
			iors = m_ioState[slave_idx].iors;
			if (IORS_SUCCESS == iors )
			{
				*bitData = m_ioState[slave_idx].input;
				m_mutex.unlock();
				break;
			}
			else if (IORS_TIMEOUT  == iors)
			{
				retValue = ERR_IO_READ_TIMEOUT;
				m_mutex.unlock();
				break;
			}
			
			m_mutex.unlock();
			
			Poco::Thread::sleep(10);
		}
	}
	return retValue;
}

