#ifndef DMC_MANAGER
#define DMC_MANAGER
#include "DMC.h"
#include "RdWrManager.h"
#include "Request.h"
#include "SlaveState.h"

#include "NEXTWUSBLib_12B.h"
#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include <map>
//#define TIMING 1  

using std::map;

class DmcManager : public Poco::Runnable
{
public:
	static DmcManager & instance();
	
	virtual void run();
	
	unsigned long init();
	void close();
	//记录CSP规划点位置到日志文件中
	void logCspPoints(Item **itemLists, int rows, size_t cols) const;

	//运动
	unsigned long clr_alarm(short axis);
	unsigned long init_driver(short axis);
	unsigned long servo_on(short axis);
	unsigned long start_move(short axis,long Dist,double MaxVel,double Tacc, bool abs, MoveType movetype);
	unsigned long home_move(short axis,long highVel,long lowVel,long acc);
	unsigned long multi_home_move(short totalAxis, short * axisArray);
	unsigned long start_line(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs,  MoveType movetype);
	unsigned long start_archl(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs,  long hh, long hu, long hd);
	unsigned long check_done(short axis);
	long get_command_pos(short axis);

	//同步匀加速持续运动
	unsigned long start_running(short TotalAxis,short *AxisArray,long *VelArray, double Tacc);
	unsigned long adjust(short axis, short deltav, size_t cycles);
	unsigned long end_running(short TotalAxis,short *AxisArray,double tDec);

	//停止
	unsigned long decel_stop(short axis, double tDec, bool bServOff = false);
	unsigned long immediate_stop(short axis);

	unsigned long make_overflow(short axis);

	//电机状态相关
	long getCurpos(short slaveidx);			//获得电机当前位置,驱动器实时更新
	bool isDriverOpCsp(short slaveidx);	//判断电机当前是否处于CSP操作模式
	bool isDriverOn(short slaveidx);		 //判断电机是否励磁
	bool isDriverOff(short slaveidx);
	unsigned short getDriverStatus(short slaveidx);
	bool isDriverHomed(short slaveidx); //判断电机是否已经回原点
	bool isServo(short slaveidx);			 //判断电机是否属于伺服

	long getDriverCmdPos(short slaveidx);									//获得起始位置
	void setDriverCmdPos(short slaveidx, long val); 						//更新起始位置
	int	getServoPosBias(int slaveidx);
	unsigned long getSlaveState(short axis);

	//IO
	unsigned long out_bit(short slave_idx, short bitNo, short bitData);
	unsigned long in_bit(short ioslave_idx, unsigned int *bitData);

	void setIoOutput(short slaveidx, unsigned int output);
	unsigned int getIoOutput(short slaveidx);
	unsigned int getIoInput(short slaveidx);

	//内部TransData命令
	transData *getCmdData(short slaveidx);
	transData *getRespData(short slaveidx);
	bool	getSdoCmdResp(BaseRequest *req, transData **ppCmd, transData **ppResp);		//获取SDO命令，成功返回true
	void 	freeSdoCmdResp(BaseRequest *req);											  //释放SDO命令
	void restoreLastCmd(transData *cmdData);
	void pushItems(Item **itemLists, size_t rows, size_t cols, bool sync, bool keep = false);

	void setRespData(transData *respData);		// 发送接收线程返回数据，以及发送接收线程当前状态
	void copyRespData();						// 处理返回的数据
	
	virtual ~DmcManager();

	RdWrManager 		m_rdWrManager;			//发送接收管理线程
	double				quadpart;
private:
	DmcManager();
	void 	clear();

	void beforeWriteCmd();				//填充下一次需要发送的命令

	bool loadXmlConfig();
	void initSlaveState();
	void updateState();					//每次调用ECMUSBRead后，更新所有从站状态

	bool isDriverSlave(short slaveidx) const;
	bool isIoSlave(short slaveidx) const;
	void setSlaveState(short slavidx, unsigned int ss);
	void addRequest(short slaveidx, BaseRequest *req);

	Poco::Thread		m_thread;
	Poco::Mutex  		m_mutex;						//互斥量，保护m_requests
	Poco::Condition		m_condition;					//条件变量

	bool				m_init;
	bool				m_canceled;						//线程停止

	BaseRequest			*m_sdoOwner;					//SDO当前所属请求
	
	transData			m_cmdData[DEF_MA_MAX];
	transData			m_lastCmdData[DEF_MA_MAX];
	transData			m_respData[DEF_MA_MAX];
	
	unsigned char		m_slaveType[DEF_MA_MAX - 2]; 	//从站类型, DRIVER/IO

	Poco::Mutex  		m_mutexRespData;				//响应数据互斥量
	Poco::Condition		m_conditionRespData;			//响应数据条件变量
	bool				newRespData;					//响应数据是否刷新
	transData			m_realRespData[DEF_MA_MAX];		//实时响应数据

	MasterConfig		m_masterConfig;					//主站配置信息

	std::map<int, BaseRequest *> 	m_requests;			
	std::map<int, DriverSlaveState> m_slaveStates;

	Item *				m_itemLists[DEF_MA_MAX];
	int					m_cols;
};

#endif
