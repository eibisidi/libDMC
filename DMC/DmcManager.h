#ifndef DMC_MANAGER
#define DMC_MANAGER
#include "DMC.h"
#include <map>
#include "RdWrManager.h"

#include "NEXTWUSBLib_12B.h"
#include "Poco/Mutex.h"
#include "Poco/Condition.h"
#include "Poco/Runnable.h"
#include "Poco/Thread.h"
#include "Request.h"

//#define TIMING 1

using std::map;

#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)


enum OpMode						//控制模式
{
	OPMODE_NONE	  = -1,
	OPMODE_HOMING = HOMING_MODE,
	OPMODE_CSP	  = CSP_MODE,
	OPMODE_CSV	  = CSV_MODE,
	OPMODE_CST	  = CST_MODE,
};

//电机状态
struct DriverState
{
	long 			curpos;		//当前位置, 驱动器返回实时更新
	MoveState		movestate;
	unsigned short	status;		//状态字
	OpMode			opmode;		//操作模式
	long			cmdpos;		//绝对位置,第一次电机使能后赋初值,回零后更新
	
	DriverState()
	{
		curpos = 0;
		movestate  = MOVESTATE_NONE;
		status = 0;
		opmode = OPMODE_CSP;//OPMODE_NONE;
		cmdpos = 0;
	}
};

enum IoRequestState
{
	IORS_NONE 	= -1,
	IORS_BUSY 	= 0,
	IORS_SUCCESS,
	IORS_TIMEOUT,
};

struct IoState
{
	unsigned int    input;			//开入值
	IoRequestState	iors;
	IoState()
	{
		input 	= 0;
		iors	= IORS_NONE;
	}
};

class DmcManager : public Poco::Runnable
{

public:
	static DmcManager & instance();
	
	virtual void run();
	
	unsigned long init();
	void close();
	//记录CSP规划点位置到日志文件中
	void logCspPoints(const Item *pItems, int rows, int cols) const;

	//运动
	unsigned long clr_alarm(short axis);
	unsigned long init_driver(short axis);
	unsigned long servo_on(short axis);
	unsigned long start_move(short axis,long Dist,double MaxVel,double Tacc, bool abs, MoveType movetype);
	unsigned long home_move(short axis,long highVel,long lowVel,long acc);
	unsigned long start_line(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs,  MoveType movetype);
	unsigned long start_archl(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs,  long hh, long hu, long hd);
	unsigned long check_done(short axis);
	long get_command_pos(short axis);

	//停止
	unsigned long decel_stop(short axis, double tDec, bool bServOff = false);
	unsigned long immediate_stop(short axis);

	//电机状态相关
	long getCurpos(short slaveidx);			//获得电机当前位置,驱动器实时更新
	bool isDriverOpCsp(short slaveidx);	//判断电机当前是否处于CSP操作模式
	bool isDriverOn(short slaveidx);		 //判断电机是否励磁
	bool isDriverOff(short slaveidx);
	unsigned short getDriverStatus(short slaveidx);
	bool isDriverHomed(short slaveidx); //判断电机是否已经回原点
	bool isServo(short slaveidx);			 //判断电机是否属于伺服
	void setMoveState(short slaveidx, MoveState ms);//设置电机当前状态
	long getDriverCmdPos(short slaveidx);									//获得起始位置
	void setDriverCmdPos(short slaveidx, long val); 						//更新起始位置
	int	getServoPosBias() const;

	//IO
	unsigned long out_bit(short ioslave_idx, unsigned int bitData);
	unsigned long in_bit(short ioslave_idx, unsigned int *bitData);
	void setIoRS(short slavidx, IoRequestState iors);	//设置当前IO状态

	//内部TransData命令
	transData *getCmdData(short slaveidx);
	transData *getRespData(short slaveidx);
	bool	getSdoCmdResp(BaseRequest *req, transData **ppCmd, transData **ppResp);		//获取SDO命令，成功返回true
	void 	freeSdoCmdResp(BaseRequest *req);											  //释放SDO命令
	void restoreLastCmd(transData *cmdData);

	void setRespData(transData *respData);		// 发送接收线程返回数据，以及发送接收线程当前状态
	void copyRespData();						// 处理返回的数据
	
	virtual ~DmcManager();

	RdWrManager m_rdWrManager;			//发送接收管理线程
private:
	DmcManager();
	void 	clear();

	void beforeWriteCmd();				//填充下一次需要发送的命令

	bool loadXmlConfig();
	void initSlaveState();
	void updateState();					//每次调用ECMUSBRead后，更新所有从站状态

	void addRequest(short slaveidx, BaseRequest *req);

	Poco::Thread		m_thread;
	Poco::Mutex  		m_mutex;				//互斥量
	Poco::Condition		m_condition;			//条件变量

	bool				m_init;
	bool				m_canceled;				//线程停止

	BaseRequest			*m_sdoOwner;			//SDO当前所属请求
	
	transData			m_cmdData[DEF_MA_MAX];
	transData			m_lastCmdData[DEF_MA_MAX];
	transData			m_respData[DEF_MA_MAX];
	
	unsigned char		m_slaveType[DEF_MA_MAX - 2]; //从站类型, DRIVER/IO


	Poco::Mutex  		m_mutexRespData;				//响应数据互斥量
	Poco::Condition		m_conditionRespData;			//响应数据条件变量
	bool				newRespData;					//响应数据是否刷新
	transData			m_realRespData[DEF_MA_MAX];		//实时响应数据

	MasterConfig		m_masterConfig;			//主站配置信息

	map<int, BaseRequest *> m_requests;			

	map<int, DriverState>m_driverState;			//电机状态
	map<int, IoState>	 m_ioState;				//IO状态

	Item				m_items[DEF_MA_MAX];
};

#endif
