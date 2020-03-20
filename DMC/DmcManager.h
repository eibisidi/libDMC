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



struct IoState
{
	unsigned int    input;			//����ֵ
	unsigned int    output;			//���ֵ
	IoRequestState	iors;
	IoState()
	{
		input 	= 0;
		output  = 0;
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
	void logCspPoints(const Item *pItems, int rows, size_t cols) const;

	//�˶�
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

	long getDriverCmdPos(short slaveidx);									//�����ʼλ��
	void setDriverCmdPos(short slaveidx, long val); 						//������ʼλ��
	int	getServoPosBias() const;

	//IO
	unsigned long out_bit(short slave_idx, short bitNo, short bitData);
	unsigned long in_bit(short ioslave_idx, unsigned int *bitData);

	void setIoOutput(short slaveidx, unsigned int output);
	unsigned int getIoOutput(short slaveidx);
	unsigned int getIoInput(short slaveidx);
	void flipread(short slaveidx);

	//�ڲ�TransData����
	transData *getCmdData(short slaveidx);
	transData *getRespData(short slaveidx);
	bool	getSdoCmdResp(BaseRequest *req, transData **ppCmd, transData **ppResp);		//��ȡSDO����ɹ�����true
	void 	freeSdoCmdResp(BaseRequest *req);											  //�ͷ�SDO����
	void restoreLastCmd(transData *cmdData);
	void pushItems(const Item *items, size_t rows, size_t cols, bool sync);

	void setRespData(transData *respData);		// ���ͽ����̷߳������ݣ��Լ����ͽ����̵߳�ǰ״̬
	void copyRespData();						// �����ص�����
	
	virtual ~DmcManager();

	RdWrManager 		m_rdWrManager;			//���ͽ��չ����߳�
private:
	DmcManager();
	void 	clear();

	void beforeWriteCmd();				//�����һ����Ҫ���͵�����

	bool loadXmlConfig();
	void initSlaveState();
	void updateState();					//ÿ�ε���ECMUSBRead�󣬸������д�վ״̬

	bool isDriverSlave(short slaveidx) const;
	bool isIoSlave(short slaveidx) const;
	void setSlaveState(short slavidx, unsigned int ss);
	void addRequest(short slaveidx, BaseRequest *req);


	Poco::Thread		m_thread;
	Poco::Mutex  		m_mutex;				//������������m_requests
	Poco::Condition		m_condition;			//��������

	bool				m_init;
	bool				m_canceled;				//�߳�ֹͣ

	BaseRequest			*m_sdoOwner;			//SDO��ǰ��������
	
	transData			m_cmdData[DEF_MA_MAX];
	transData			m_lastCmdData[DEF_MA_MAX];//todo��ʵͬ������ñ����Ƿ����
	transData			m_respData[DEF_MA_MAX];
	
	unsigned char		m_slaveType[DEF_MA_MAX - 2]; //��վ����, DRIVER/IO


	Poco::Mutex  		m_mutexRespData;				//��Ӧ���ݻ�����
	Poco::Condition		m_conditionRespData;			//��Ӧ������������
	bool				newRespData;					//��Ӧ�����Ƿ�ˢ��
	transData			m_realRespData[DEF_MA_MAX];		//ʵʱ��Ӧ����

	MasterConfig		m_masterConfig;			//��վ������Ϣ

	std::map<int, BaseRequest *> m_requests;			
	std::map<int, SlaveState *> m_slaveStates;

	Item				m_items[DEF_MA_MAX];
	int					m_cols;

};

#endif
