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
	quadpart	= 1.0;
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
	m_slaveStates.clear();
}

unsigned long DmcManager::init()
{
#ifdef DEBUG_MEMLEAK
	_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );	//To Dectect Memory Leaks.
#endif

	LARGE_INTEGER frequency;									//计时器频率 
	QueryPerformanceFrequency(&frequency);	 
	quadpart = (double)frequency.QuadPart; 						//计时器频率	

	//初始化日志
	CLogSingle::initLogger();

	LOGSINGLE_INFORMATION("libDMC ver 1.0. buildtime : %s %s.", __FILE__, __LINE__, std::string(__DATE__), std::string(__TIME__));

	if (m_init)
	{	
		LOGSINGLE_ERROR("init already called.%s", __FILE__, __LINE__, std::string(""));
		return ERR_DUP_INIT;
	}

	//读取配置信息
	if (!loadXmlConfig())
	{
		LOGSINGLE_ERROR("loadXmlConfig failed.%s", __FILE__, __LINE__, std::string(""));
		return ERR_LOAD_XML;
	}

	//设置日志记录等级
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

	//启动线程
	m_rdWrManager.start();
	//m_thread.setPriority(Poco::Thread::PRIO_NORMAL);
	m_thread.setOSPriority(THREAD_PRIORITY_TIME_CRITICAL);
	m_thread.start(*this);

	//所有驱动器清除告警
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

	//所有驱动器开始设置配置参数
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

	//所有驱动器励磁
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

	m_rdWrManager.cancel();		//停止线程

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

#ifdef DEBUG_MEMLEAK
	_CrtDumpMemoryLeaks();
#endif

}

void DmcManager::logCspPoints(Item **itemLists, int rows, size_t cols) const
{
	if (m_masterConfig.logpoint_axis.empty())
		return;

	std::vector<const Item*> vecPrintCur;
	for(int c = 0; c < cols; ++c)
	{
		const Item * itemList = itemLists[c];
		vecPrintCur.push_back(itemList);
	}
	
	for (int r = 0; r < rows; ++r)
	{
		//按照轴号顺序输出规划位置
		for (std::set<int>::const_iterator iter = m_masterConfig.logpoint_axis.begin();
											iter != m_masterConfig.logpoint_axis.end();
											++iter)
		{
			for(int c = 0; c < cols; ++c)
			{
				const Item * itemList = itemLists[c];
				if (itemList->index == *iter)
				{
					CLogSingle::logPoint((int)(vecPrintCur[c]->cmdData.Data1));
					vecPrintCur[c] = vecPrintCur[c]->next;
				}
			}
		}
		CLogSingle::logPoint("\n");
	}
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

		m_slaveStates[slave_idx].setSlaveState(retval);	//按照状态机返回值设置请求当前状态

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
			m_slaveStates[i] = DriverSlaveState();
		else if (m_slaveType[i - 1] == IO)
			m_rdWrManager.addIoSlave(i);
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
					LOGSINGLE_ERROR("XML Error, Slave %d, Unknown type = %s.", __FILE__, __LINE__, slave_index, pAttr->nodeValue());
					return false;
				}

				//驱动器偏差
				int bias = 0;
				pAttr = static_cast<Poco::XML::Attr*>(pAttrs->getNamedItem("bias"));
				if (pAttr)
				{
					bias =  Poco::NumberParser::parse(pAttr->nodeValue());
				}
				
				SlaveConfig sc(slave_index, slave_type, bias);
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

					if (!Poco::NumberParser::tryParse(pValueAttr->nodeValue(), sdo.sdo_value))
					{//值十进制解析失败后，使用16进制数解析
						sdo.sdo_value = Poco::NumberParser::parseHex(pValueAttr->nodeValue());
					}

					sc.slave_sdos.push_back(sdo);
				}

				if (0 != m_masterConfig.slave_configs.count(sc.slave_index))
				{
					LOGSINGLE_ERROR("XML Error, duplicate slave index %d.", __FILE__, __LINE__, sc.slave_index);
					return false;
				}

				m_masterConfig.slave_configs[sc.slave_index] = sc;
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
			pNode = it.nextNode();
		}
	}
	catch (Poco::Exception& exc)
	{
		LOGSINGLE_ERROR("XML Exception, %s.", __FILE__, __LINE__, exc.displayText());
		return false;
	}

	//设置从站的类型
	for(std::map<int, SlaveConfig>::const_iterator iter = m_masterConfig.slave_configs.begin();
				iter != m_masterConfig.slave_configs.end();
				++iter)
	{
		m_slaveType[iter->first - 1] = (iter->second).slave_type;
	}
	
	return true;
}

long DmcManager::getDriverCmdPos(short slaveidx)
{
	long cmdpos = 0;

	if (m_slaveStates.count(slaveidx))
		cmdpos = m_slaveStates[slaveidx].getCmdPos();

	return cmdpos;
}

void DmcManager::setDriverCmdPos(short slaveidx, long val)
{
	if (m_slaveStates.count(slaveidx))
		 m_slaveStates[slaveidx].setCmdPos(val);
}

int DmcManager::getServoPosBias(int slaveidx)
{
	return m_masterConfig.slave_configs[slaveidx].axis_bias;
}

unsigned long DmcManager::getSlaveState(short axis)
{
	unsigned int ms = MOVESTATE_NONE;

	if(m_slaveStates.count(axis) )
		ms = m_slaveStates[axis].getSlaveState();

	return ms;
}

void DmcManager::updateState()
{
	//更新驱动器状态
	unsigned CMD;
	for(std::map<int, DriverSlaveState>::iterator iter = m_slaveStates.begin();
				iter != m_slaveStates.end();
				++iter)
	{
		int		slaveidx = iter->first;
		
		CMD =((m_respData[slaveidx].CMD & 0xFF));
		
		switch(CMD)
		{
			case GET_STATUS:
			case ALM_CLR:
			case CSP:
			case GO_HOME:
			case ABORT_HOME:
			case SV_ON:
			case SV_OFF:
				(iter->second).setStatus(m_respData[slaveidx].Parm);
				(iter->second).setCurPos(m_respData[slaveidx].Data1);
				(iter->second).setAlarmCode(m_respData[slaveidx].Data2 >> 16);	//高字为告警码
				if ((iter->second).getAlarmCode())
					(iter->second).setSlaveState(MOVESTATE_ERR);
				break;
			case DRIVE_MODE:
				break;
			default:
				;
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
	m_slaveStates[slaveidx].setSlaveState(ss);
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

	if (m_slaveStates.count(slaveidx))
		curpos = m_slaveStates[slaveidx].getCurPos();

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

void DmcManager::pushItems(Item **itemLists, size_t rows, size_t cols, bool sync, bool keep)
{
	if(NULL == itemLists
		|| rows <= 0
		|| cols <= 0)
		return;

	if(sync)
		m_rdWrManager.pushItemsSync(itemLists, rows, cols, keep);
	else
		m_rdWrManager.pushItems(itemLists, rows, cols, keep);

	//将最后一行的命令更新到m_lastCmdData中
	for(size_t c = 0; c < cols; ++c)
	{
		Item * tailItem = itemLists[c]->prev;
		int slave_idx 	= tailItem->index;
		m_lastCmdData[slave_idx] = tailItem->cmdData;
	}
}

void DmcManager::setRespData(transData *respData)
{
#ifdef TIMING
	static double longest = 0;
	static double shortest = 1000000;
	static double total = 0;
	static int	  count = 0;
	LARGE_INTEGER frequency;									//计时器频率 
	QueryPerformanceFrequency(&frequency);	 
	double quadpart = (double)frequency.QuadPart / 1000000; 	//计时器频率	

	LARGE_INTEGER timeStart, timeEnd;
	double elapsed;
	QueryPerformanceCounter(&timeStart); 
#endif

	m_seqlock.lock();
	memcpy(m_realRespData, respData, sizeof(m_realRespData));
	m_conditionRespData.signal();
	m_seqlock.unlock();

#if 0
	m_mutexRespData.lock();

	memcpy(m_realRespData, respData, sizeof(m_realRespData));
	newRespData = true;
	m_conditionRespData.signal();
	
	m_mutexRespData.unlock();
#endif
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
	unsigned int begin_seq,end_seq;
	do {
		m_mutexRespData.lock();
		m_conditionRespData.wait(m_mutexRespData);
		m_mutexRespData.unlock();
		begin_seq 	= m_seqlock.seq;
		if (!(begin_seq & 1))
			memcpy(m_respData, m_realRespData, sizeof(m_realRespData));
		end_seq		= m_seqlock.seq;
	}while ((begin_seq & 1) ||( end_seq != begin_seq));
	
	updateState();
}

bool DmcManager::isDriverOpCsp(short slaveidx)
{
	OpMode op = OPMODE_NONE;

	if (m_slaveStates.count(slaveidx))
		op = m_slaveStates[slaveidx].getOpMode();

	return (OPMODE_CSP == op);
}

bool DmcManager::isDriverOn(short slaveidx)
{
	unsigned short status = 0;

	if (m_slaveStates.count(slaveidx))
		status = m_slaveStates[slaveidx].getStatus();

	return ((status & 0xFF) == 0x37);	//某些情况电机状态字为0x2337，为防止进行SV_ON操作，仅判别最低字节

	//return (0x1637 == status
	//	|| 0x9637 == status);		//原点已找到
}

bool DmcManager::isDriverOff(short slaveidx)
{
	unsigned short status = 0;

	if (m_slaveStates.count(slaveidx))
		status = m_slaveStates[slaveidx].getStatus();

	//最低比特置1，代表电机已准备好。
	return (0 == (0x01 & status ));
}

unsigned short DmcManager::getDriverStatus(short slaveidx)
{
	unsigned short status = 0;
	if (m_slaveStates.count(slaveidx))
		status = m_slaveStates[slaveidx].getStatus();

	return (status);
}

bool DmcManager::isDriverHomed(short slaveidx)
{
	unsigned short status = 0;
	long 			curpos = 0;

	if (m_slaveStates.count(slaveidx))
	{
		status = m_slaveStates[slaveidx].getStatus();
		curpos = m_slaveStates[slaveidx].getCurPos();
	}

	return ( (0x9637 == status || 0x1637 == status)
		&& 0 == curpos);
}

bool DmcManager::isServo(short slaveidx)
{
	assert (slaveidx > 0 && slaveidx < DEF_MA_MAX - 1);
	return DRIVE == m_slaveType[slaveidx - 1];				//伺服电机
}

void DmcManager::setIoOutput(short slaveidx, unsigned int output)
{
	m_rdWrManager.setIoOutput( slaveidx,  output);
}

unsigned int DmcManager::getIoOutput(short slaveidx)
{
	return m_rdWrManager.getIoOutput(slaveidx);
}

unsigned int DmcManager::getIoInput(short slaveidx)
{
	return m_rdWrManager.getIoInput(slaveidx);
}

void DmcManager::run()
{	
	while(!m_canceled)
	{		
		m_mutex.lock();
		if(!m_requests.empty())
		{
			beforeWriteCmd();
			m_mutex.unlock();
			int i = 0;
			for(m_cols = 0; i < DEF_MA_MAX; ++i)
			{
				if (m_cmdData[i].CMD != GET_STATUS)
				{
					m_itemLists[m_cols] = new Item;
					m_itemLists[m_cols]->index = i;
					m_itemLists[m_cols]->cmdData = m_cmdData[i];
					++m_cols;
				}		
			}

			if (m_cols > 0)
				pushItems(m_itemLists, 1, m_cols, false);	//加入发送队列

			copyRespData();//接收队列处理，刷新
		}
		else
		{
			if (!m_condition.tryWait(m_mutex, 2))
			{
				copyRespData();
			}
			m_mutex.unlock();			
		}
	}

	LOGSINGLE_INFORMATION("DmcManager Thread canceled.%s", __FILE__, __LINE__, std::string(""));
}

unsigned long DmcManager::clr_alarm(short axis)
{
	unsigned long retValue = ERR_NOERR;
	ClrAlarmRequest *newReq = NULL;
	
	do{
		if (false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}
		
		if (MOVESTATE_BUSY == getSlaveState(axis))
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

		m_mutex.lock();
		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);
		m_mutex.unlock();
	}while(0);

	unsigned long ms = MOVESTATE_NONE;
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

	do{
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (MOVESTATE_BUSY == getSlaveState(axis))
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

		m_mutex.lock();
		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);		
		m_mutex.unlock();
	}while(0);

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
	
	do{
		if (false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (MOVESTATE_BUSY == getSlaveState(axis))
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
		
		m_mutex.lock();
		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);	
		m_mutex.unlock();	
	}while(0);

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

	do{
		if (false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (MOVESTATE_BUSY == getSlaveState(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}

		//判断目标位置是否已经到达
		if (!abs && Dist == 0)
		{
			setSlaveState(axis, MOVESTATE_STOP);
			break;						//相对模式，距离为0
		}

		if (abs && Dist == getDriverCmdPos(axis))
		{
			setSlaveState(axis, MOVESTATE_STOP);
			break;						//绝对模式，位置已到达
		}
	
		newReq = new MoveRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}
		newReq->slave_idx = axis;
		newReq->abs 	  = abs;				//相对 绝对
		newReq->dist	  = Dist;
		newReq->movetype  = movetype;			//S / T
		newReq->maxvel	  = MaxVel;				//最大速度
		newReq->maxa	  = MaxVel / Tacc;		//最大加速度
		newReq->maxj	  = MAXJ_RATIO *(newReq->maxa);
		
		m_mutex.lock();
		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);
		m_mutex.unlock();	
	}while(0);

	return retValue;
}

/*
	已废弃不用
	highVel: 高速搜索减速点，索引0x6099 子索引0x0001 实际意义见电机驱动器参考手册
	lowVel：低速搜索原点，索引0x6099 子索引0x0002 实际意义见电机驱动器参考手册
	acc： 减速度 ，索引0x609A 子索引0x0000 实际意义见电机驱动器参考手册
*/
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
	
	//先进行容错检查
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		if (MOVESTATE_BUSY == getSlaveState(axis))
		{
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
	
	m_mutex.lock();
	//新建请求
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		newReq = new MultiHomeRequest(axis, newRef, m_masterConfig.home_timeout);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}	
	m_condition.signal();	
	m_mutex.unlock();
DONE:
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
	
	//先进行容错检查
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		if (MOVESTATE_BUSY == getSlaveState(axis))
		{
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
	
	m_mutex.lock();
	//新建请求
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		newReq = new MultiAxisRequest(axis, newLinearRef, distArray[i], abs);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}
	m_condition.signal();	
	m_mutex.unlock();
DONE:
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
	
	//先进行容错检查
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		if (MOVESTATE_BUSY == getSlaveState(axis))
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
	
	m_mutex.lock();
	//新建请求
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		long dist = distArray[i];
		newReq = new MultiAxisRequest(axis, newArchlRef, distArray[i], abs, (i==0)/*是否为Z轴*/);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}	
	m_condition.signal();
	m_mutex.unlock();
DONE:
	return retValue;
}

unsigned long DmcManager::check_done(short axis)
{
	unsigned int ms = MOVESTATE_NONE;

	if(m_slaveStates.count(axis) )
		ms = m_slaveStates[axis].getSlaveState();

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

unsigned long DmcManager::start_running(short totalAxis,short *axisArray,long *VelArray, double Tacc)
{
	if (totalAxis <= 0
			|| axisArray == NULL
			|| VelArray  == NULL
			|| Tacc < 1E-6)
		return ERR_INVALID_ARG;
	
	unsigned long	retValue = ERR_NOERR;
	MultiAxisRequest *newReq = NULL;
	AccRef		 	*newAccRef = NULL; //匀加速
	
	//先进行容错检查
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		unsigned long ms = getSlaveState(axis);
		
		if (MOVESTATE_BUSY == ms
			|| MOVESTATE_RUNNING == ms)
		{
			retValue = ERR_AXIS_BUSY;
			goto DONE;
		}
	}

	newAccRef = new AccRef;
	if(NULL == newAccRef)
	{
		retValue = ERR_MEM;
		goto DONE;
	}

	newAccRef->maxv = VelArray[0];
	newAccRef->tAcc = Tacc;
	
	m_mutex.lock();
	//新建请求
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short 	axis = axisArray[i];
		long 	vel = VelArray[i];
		newReq = new MultiAxisRequest(axis, newAccRef, vel);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}	
	m_condition.signal();
	m_mutex.unlock();
DONE:
	return retValue;
}

unsigned long DmcManager::adjust(short axis, short deltav, size_t cycles)
{
	if ( false == isDriverSlave(axis))
	{
		return ERR_NO_AXIS;
	}

	m_rdWrManager.setAdjust(axis, deltav, cycles);
	return ERR_NOERR;
}

unsigned long DmcManager::end_running(short totalAxis,short *axisArray,double tDec)
{
	if (totalAxis <= 0
			|| axisArray == NULL
			|| tDec < 1E-6)
		return ERR_INVALID_ARG;
	
	unsigned long	retValue 					= ERR_NOERR;
	MultiDeclRequest *newReq 					= NULL;
	MultiDeclRequest::MultiDeclRef	*newDeclRef = NULL; //匀加速
	
	//先进行容错检查
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short axis = axisArray[i];
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			goto DONE;
		}

		unsigned long ms = getSlaveState(axis);
		
		if (MOVESTATE_BUSY == ms)
		{
			retValue = ERR_AXIS_BUSY;
			goto DONE;
		}
		
		if (MOVESTATE_RUNNING != ms)
		{
			retValue = ERR_AXIS_NOT_MOVING;
			goto DONE;
		}
	}

	newDeclRef = new MultiDeclRequest::MultiDeclRef;
	if(NULL == newDeclRef)
	{
		retValue = ERR_MEM;
		goto DONE;
	}

	m_mutex.lock();
	//新建请求
	for(int i = 0 ; i < totalAxis; ++i)
	{
		short	axis = axisArray[i];
		newReq = new MultiDeclRequest(axis, newDeclRef, tDec);
		setSlaveState(axis, MOVESTATE_BUSY);
		m_requests[axis] = newReq;
	}	
	m_condition.signal();
	m_mutex.unlock();
DONE:
	return retValue;
}

unsigned long DmcManager::decel_stop(short axis, double tDec, bool bServOff)
{
	assert(tDec > 1E-6);
	unsigned long retValue = ERR_NOERR;
	DStopRequest *newReq = NULL;

	do{
		if ( false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (!bServOff
			&& MOVESTATE_BUSY != getSlaveState(axis))
		{//当前未在运动
			retValue = ERR_AXIS_NOT_MOVING;
			break;
		}
		
		m_mutex.lock();

		if (m_requests.count(axis) > 0)
		{

			MoveRequest *moveReq = dynamic_cast<MoveRequest * >(m_requests[axis]);
			if (NULL != moveReq)
			{//单轴运动
				newReq = new DStopRequest(axis, tDec, bServOff);
				setSlaveState(axis, MOVESTATE_BUSY);
				addRequest(axis, newReq);
			}
			else
			{
				MultiAxisRequest *maReq = dynamic_cast<MultiAxisRequest * >(m_requests[axis]);
				if (NULL != maReq)
				{//多轴运动
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
				{
					MultiHomeRequest *mhReq = dynamic_cast<MultiHomeRequest * >(m_requests[axis]);
					if (NULL != mhReq
						&& mhReq->ref->getStarted())		//已经启动GO_HOME
					{//多轴回原点
						std::set<int> toAbort = mhReq->ref->getAxises();	//makes a copy
						for (std::set<int>::iterator iter = toAbort.begin();
									iter != toAbort.end();
							)
						{
							if(MOVESTATE_BUSY != check_done(*iter)) 
								toAbort.erase(iter++);		//该轴回零命令已经处理完毕
							else
								iter++;
						}

						if (!toAbort.empty())
						{//仍然有轴回零未成功
							MultiAbortHomeRequest::MultiAbortHomeRef *newRef = NULL;
							MultiAbortHomeRequest *newAhReq = NULL;
							newRef = new MultiAbortHomeRequest::MultiAbortHomeRef;
							for (std::set<int>::iterator iter = toAbort.begin();
									iter != toAbort.end();
									++iter)
							{
								newAhReq = new MultiAbortHomeRequest(*iter, newRef);
								setSlaveState(*iter, MOVESTATE_BUSY);
								addRequest(*iter, newAhReq);
								mhReq = NULL;
							}
						
						}
					}
					else
					{//当前未在运动
						DStopRequest *dsReq = dynamic_cast<DStopRequest*>(m_requests[axis]);
						if (NULL != dsReq)
						{
							retValue = ERR_AXIS_NOT_MOVING;
							break;
						}
					}
				}
			}	
		}
		else
		{
			if (bServOff)
			{
				newReq = new DStopRequest(axis, tDec, bServOff);
				setSlaveState(axis, MOVESTATE_BUSY);
				addRequest(axis, newReq);
			}
			else
			{
				retValue = ERR_AXIS_NOT_MOVING;
				break;
			}
		}
		m_mutex.unlock();
	}while(0);

	return retValue;
}

unsigned long DmcManager::immediate_stop(short axis)
{
	return decel_stop(axis, 0.1, true);
}

unsigned long DmcManager::make_overflow(short axis)
{
	unsigned long retValue = ERR_NOERR;
	MakeOverFlowRequest *newReq = NULL;

	do{
		if (false == isDriverSlave(axis))
		{
			retValue = ERR_NO_AXIS;
			break;
		}

		if (MOVESTATE_BUSY == getSlaveState(axis))
		{
			retValue = ERR_AXIS_BUSY;
			break;
		}
	
		newReq = new MakeOverFlowRequest;
		if (NULL == newReq)
		{
			retValue = ERR_MEM;
			break;
		}
		newReq->slave_idx = axis;
		
		m_mutex.lock();
		setSlaveState(axis, MOVESTATE_BUSY);
		addRequest(axis, newReq);
		m_mutex.unlock();	
	}while(0);

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
		LOGSINGLE_INFORMATION("make_overflow axis(%?d) succeed, cmdpos = %?d.", __FILE__, __LINE__, axis, getDriverCmdPos(axis));
	}
	else
	{
		retValue = ERR_INIT_AXIS;
		LOGSINGLE_ERROR("make_overflow axis(%?d) failed.",  __FILE__, __LINE__, axis);
	}

	return retValue;
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

