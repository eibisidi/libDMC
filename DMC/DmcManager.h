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


enum OpMode						//����ģʽ
{
	OPMODE_NONE	  = -1,
	OPMODE_HOMING = HOMING_MODE,
	OPMODE_CSP	  = CSP_MODE,
	OPMODE_CSV	  = CSV_MODE,
	OPMODE_CST	  = CST_MODE,
};

//���״̬
struct DriverState
{
	long 			curpos;		//��ǰλ��, ����������ʵʱ����
	MoveState		movestate;
	unsigned short	status;		//״̬��
	OpMode			opmode;		//����ģʽ
	long			cmdpos;		//����λ��,��һ�ε��ʹ�ܺ󸳳�ֵ,��������
	
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
	unsigned int    input;			//����ֵ
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
	//��¼CSP�滮��λ�õ���־�ļ���
	void logCspPoints(const Item *pItems, int rows, int cols) const;

	//�˶�
	unsigned long clr_alarm(short axis);
	unsigned long init_driver(short axis);
	unsigned long servo_on(short axis);
	unsigned long start_move(short axis,long Dist,double MaxVel,double Tacc, bool abs, MoveType movetype);
	unsigned long home_move(short axis,long highVel,long lowVel,long acc);
	unsigned long start_line(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs,  MoveType movetype);
	unsigned long start_archl(short totalAxis, short *axisArray,long *distArray, double maxvel, double Tacc, bool abs,  long hh, long hu, long hd);
	unsigned long check_done(short axis);
	long get_command_pos(short axis);

	//ֹͣ
	unsigned long decel_stop(short axis, double tDec, bool bServOff = false);
	unsigned long immediate_stop(short axis);

	//���״̬���
	long getCurpos(short slaveidx);			//��õ����ǰλ��,������ʵʱ����
	bool isDriverOpCsp(short slaveidx);	//�жϵ����ǰ�Ƿ���CSP����ģʽ
	bool isDriverOn(short slaveidx);		 //�жϵ���Ƿ�����
	bool isDriverOff(short slaveidx);
	unsigned short getDriverStatus(short slaveidx);
	bool isDriverHomed(short slaveidx); //�жϵ���Ƿ��Ѿ���ԭ��
	bool isServo(short slaveidx);			 //�жϵ���Ƿ������ŷ�
	void setMoveState(short slaveidx, MoveState ms);//���õ����ǰ״̬
	long getDriverCmdPos(short slaveidx);									//�����ʼλ��
	void setDriverCmdPos(short slaveidx, long val); 						//������ʼλ��
	int	getServoPosBias() const;

	//IO
	unsigned long out_bit(short ioslave_idx, unsigned int bitData);
	unsigned long in_bit(short ioslave_idx, unsigned int *bitData);
	void setIoRS(short slavidx, IoRequestState iors);	//���õ�ǰIO״̬

	//�ڲ�TransData����
	transData *getCmdData(short slaveidx);
	transData *getRespData(short slaveidx);
	bool	getSdoCmdResp(BaseRequest *req, transData **ppCmd, transData **ppResp);		//��ȡSDO����ɹ�����true
	void 	freeSdoCmdResp(BaseRequest *req);											  //�ͷ�SDO����
	void restoreLastCmd(transData *cmdData);

	void setRespData(transData *respData);		// ���ͽ����̷߳������ݣ��Լ����ͽ����̵߳�ǰ״̬
	void copyRespData();						// �����ص�����
	
	virtual ~DmcManager();

	RdWrManager m_rdWrManager;			//���ͽ��չ����߳�
private:
	DmcManager();
	void 	clear();

	void beforeWriteCmd();				//�����һ����Ҫ���͵�����

	bool loadXmlConfig();
	void initSlaveState();
	void updateState();					//ÿ�ε���ECMUSBRead�󣬸������д�վ״̬

	void addRequest(short slaveidx, BaseRequest *req);

	Poco::Thread		m_thread;
	Poco::Mutex  		m_mutex;				//������
	Poco::Condition		m_condition;			//��������

	bool				m_init;
	bool				m_canceled;				//�߳�ֹͣ

	BaseRequest			*m_sdoOwner;			//SDO��ǰ��������
	
	transData			m_cmdData[DEF_MA_MAX];
	transData			m_lastCmdData[DEF_MA_MAX];
	transData			m_respData[DEF_MA_MAX];
	
	unsigned char		m_slaveType[DEF_MA_MAX - 2]; //��վ����, DRIVER/IO


	Poco::Mutex  		m_mutexRespData;				//��Ӧ���ݻ�����
	Poco::Condition		m_conditionRespData;			//��Ӧ������������
	bool				newRespData;					//��Ӧ�����Ƿ�ˢ��
	transData			m_realRespData[DEF_MA_MAX];		//ʵʱ��Ӧ����

	MasterConfig		m_masterConfig;			//��վ������Ϣ

	map<int, BaseRequest *> m_requests;			

	map<int, DriverState>m_driverState;			//���״̬
	map<int, IoState>	 m_ioState;				//IO״̬

	Item				m_items[DEF_MA_MAX];
};

#endif
