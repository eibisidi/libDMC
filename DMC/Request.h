#ifndef REQUEST_H
#define REQUEST_H

#include "AxisPara.h"
#include "RdWrManager.h"

#include "Poco/Timestamp.h"

#include <vector>
#include <set>

#define DEF_HOME_METHOD     (19)			//ȱʡ��ԭ�㷽ʽ
#define DEF_HOME_TIMEOUT 	(30)			//ȱʡ��ԭ�㳬ʱʱ��30s
#define DEF_SERVO_POS_BIAS  (10)			//ȱʡ�ŷ�λ�ôﵽ���������Χ

#define CSP_DATA2_DUMMY (0xFF)


class DmcManager;

typedef unsigned int FsmRetType;

//������Ϣ
class SDO
{
public:
	unsigned int 	sdo_index;
	unsigned int 	sdo_subindex;
	unsigned char 	sdo_size;
	int 			sdo_value;
};

class SlaveConfig
{
public:
	int 				slave_index;
	unsigned char		slave_type;
	std::vector<SDO>	slave_sdos;
	SlaveConfig(int index, unsigned char type)
			:slave_index(index), slave_type(type)
	{
	}
};
	
class MasterConfig
{
public:
	int						 loglevel;			//��־��¼�ȼ�
	int						 home_method;		//��ԭ�㷽ʽ
	int						 home_timeout;		//��ԭ�㳬ʱʱ��
	int						 servo_pos_bias;	//�ŷ�λ�ôﵽ���������Χ
	std::set<int>			 slave_indexes;
	std::vector<SlaveConfig> slave_configs;
	std::set<int>			 logpoint_axis;		//��¼�滮�����

	MasterConfig()					//Ĭ�ϵȼ�4��¼�澯
	{
		clear();
	}

	void clear()
	{
		loglevel 		= 4;
		home_method 	= DEF_HOME_METHOD;
		home_timeout	= DEF_HOME_TIMEOUT;
		servo_pos_bias	= DEF_SERVO_POS_BIAS;
		slave_indexes.clear();
		slave_configs.clear();
		logpoint_axis.clear();
	}
};


enum RequestState
{
	REQUEST_STATE_NONE = 0,
	REQUEST_STATE_BUSY,
	REQUEST_STATE_SUCCESS,
	REQUEST_STATE_FAIL,
};

class BaseRequest
{
public:
	int 			rechecks;
	int				attempts;		//�ط�����
	RequestState	reqState;
	DmcManager		*dmc;
	int 			slave_idx;
	transData		*cmdData;
	transData		*respData;
	virtual ~BaseRequest(){}
	virtual FsmRetType exec() = 0;
	BaseRequest();
};

class DStopRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(DStopRequest *req);
	static FsmRetType fsm_state_svoff(DStopRequest *req);
	static FsmRetType fsm_state_csp(DStopRequest *req);
	static FsmRetType fsm_state_start(DStopRequest *req);

	bool 	positionReached(int q , int bias = 0) const;
public:
	FsmRetType (* fsmstate)(DStopRequest *);	
	
	virtual ~DStopRequest() {}
	virtual FsmRetType exec();


	bool		serveOff;				//ֹͣ���Ƿ�ر�ʹ��
	DeclStopInfo stopInfo;
	
	DStopRequest(int axis, double Tdec, bool svoff)
	{
		slave_idx = axis;
		stopInfo.decltime = Tdec;
		serveOff = false;
		fsmstate = fsm_state_start;
	}
};

class MoveRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(MoveRequest *req);
	static FsmRetType fsm_state_csp(MoveRequest *req);
	static FsmRetType fsm_state_svon(MoveRequest *req);
	static FsmRetType fsm_state_sdowr_cspmode(MoveRequest *req);
	static FsmRetType fsm_state_wait_sdowr_cspmode(MoveRequest *req);
	static FsmRetType fsm_state_start(MoveRequest *req);
	static void pushCspPoints(MoveRequest *req);

	bool startPlan();
	bool 	positionReached(int q , int bias = 0) const;


public:

	bool			abs;				//��� ����
	long			dist;				//���=λ�ƣ� ����=Ŀ��λ��
	MoveType		movetype;			//�滮����S/T
	MParam			*moveparam;			//�滮����
	
	double 			maxvel;				//����ٶȣ���λ������/s)
	double 			maxa;				//�����ٶȣ���λ(����/s^2)
	double 			maxj;				//���Ӽ��ٶȣ���λ(����/s^3),����S��������Ч

	int 			startpos;
	int 			dstpos;
	
	FsmRetType (* fsmstate)(MoveRequest *);	
	
	virtual ~MoveRequest() 
	{
		if (moveparam)
		{
			delete moveparam;
		}
	}
	
	virtual FsmRetType exec();
	MoveRequest()
	{
		abs = false;
		fsmstate = fsm_state_start;
		movetype = MOVETYPE_T;
		moveparam = NULL;

		maxvel 	 = 0;
		maxa	 = 0;
		maxj	 = 0;
		startpos = 0;
		dstpos	 = 0;
	}
};

class ClrAlarmRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(ClrAlarmRequest *req);
	static FsmRetType fsm_state_sdord_errcode(ClrAlarmRequest *req);
	static FsmRetType fsm_state_wait_sdord_errcode(ClrAlarmRequest *req);
	static FsmRetType fsm_state_alm_clr(ClrAlarmRequest *req);
	static FsmRetType fsm_state_start(ClrAlarmRequest *req);

public:	
	FsmRetType (* fsmstate)(ClrAlarmRequest *);	
	
	virtual ~ClrAlarmRequest() 
	{
	}
	
	virtual FsmRetType exec();
	ClrAlarmRequest()
	{
		fsmstate = fsm_state_start;
	}
};

class InitSlaveRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(InitSlaveRequest *req);
	static FsmRetType fsm_state_sdowr(InitSlaveRequest *req);
	static FsmRetType fsm_state_wait_sdowr(InitSlaveRequest *req);
	static FsmRetType fsm_state_start(InitSlaveRequest *req);

public:
	std::vector<SDO>::const_iterator iter;
	std::vector<SDO>::const_iterator iterEnd;

	FsmRetType (* fsmstate)(InitSlaveRequest *);	
	
	virtual ~InitSlaveRequest() 
	{
	}
	
	virtual FsmRetType exec();
	InitSlaveRequest()
	{
		fsmstate = fsm_state_start;
	}
};


class ServoOnRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(ServoOnRequest *req);
	static FsmRetType fsm_state_svon(ServoOnRequest *req);
	static FsmRetType fsm_state_svoff(ServoOnRequest *req);
	static FsmRetType fsm_state_sdowr_cspmode(ServoOnRequest *req);
	static FsmRetType fsm_state_wait_sdowr_cspmode(ServoOnRequest *req);
	static FsmRetType fsm_state_start(ServoOnRequest *req);

public:	
	FsmRetType (* fsmstate)(ServoOnRequest *);	
	
	virtual ~ServoOnRequest() 
	{
	}
	
	virtual FsmRetType exec();
	ServoOnRequest()
	{
		fsmstate = fsm_state_start;
	}
};

class MultiAxisRequest : public BaseRequest
{
private:
	static FsmRetType fsm_state_done(MultiAxisRequest *req);
	static FsmRetType  fsm_state_wait_all_pos_reached(MultiAxisRequest *req);
	static FsmRetType  fsm_state_csp(MultiAxisRequest *req);
	static FsmRetType  fsm_state_svon(MultiAxisRequest *req);
	static FsmRetType  fsm_state_wait_all_svon(MultiAxisRequest *req);
	static FsmRetType  fsm_state_sdowr_cspmode(MultiAxisRequest *req);
	static FsmRetType  fsm_state_wait_sdowr_cspmode(MultiAxisRequest *req);
	static FsmRetType fsm_state_start(MultiAxisRequest *req);
	static void pushCspPoints(MultiAxisRequest *req);

	bool	startPlan();
	bool	positionReached(int curpos, int bias = 0) const;

public:
	BaseMultiAxisPara 	*axispara;

	FsmRetType (* fsmstate)(MultiAxisRequest *);	
	
	virtual ~MultiAxisRequest() 
	{
		if (axispara)
			delete  axispara;
	}
	
	virtual FsmRetType exec();

	MultiAxisRequest(int axis, LinearRef *newLinearRef, int dist, bool abs);				//ֱ�߲岹
	MultiAxisRequest(int axis, ArchlRef *newArchlRef, int dist, bool abs, bool z);			//Z�Ṱ�Ų岹
};

class HomeMoveRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(HomeMoveRequest *req);
	static FsmRetType fsm_state_aborthome(HomeMoveRequest *req);
	static FsmRetType fsm_state_gohome(HomeMoveRequest *req);
	static FsmRetType fsm_state_sdowr_acc(HomeMoveRequest *req);
	static FsmRetType fsm_state_sdowr_lowspeed(HomeMoveRequest *req);
	static FsmRetType fsm_state_sdowr_highspeed(HomeMoveRequest *req);
	static FsmRetType fsm_state_sdowr_homemethod(HomeMoveRequest *req);
	static FsmRetType fsm_state_sdowr_homeoffset(HomeMoveRequest *req);
	static FsmRetType fsm_state_sdowr_homemode(HomeMoveRequest *req);
	static FsmRetType fsm_state_wait_sdowr_homemode(HomeMoveRequest *req);
	static FsmRetType fsm_state_svon(HomeMoveRequest *req);
	static FsmRetType fsm_state_start(HomeMoveRequest *req);

	bool homeTimeout() const;
public:
	char			home_method;		//��ԭ�㷽ʽ
	unsigned int    high_speed;			//����
	unsigned int 	low_speed;			//����
	unsigned int    acc;				//�Ӽ��ٶ�
	Poco::Timestamp	starttime;			//���ڻ�ԭ�㳬ʱ�ж�
	int				home_timeout;		//��ԭ�㳬ʱʱ��,��λs
	
	FsmRetType (* fsmstate)(HomeMoveRequest *);	 
	
	virtual ~HomeMoveRequest() {}
	virtual FsmRetType exec();
	HomeMoveRequest()
	{
		home_method = 0;
		high_speed  = 0;
		low_speed	= 0;
		acc			= 0;
		home_timeout= 0;
		fsmstate = fsm_state_start;
	}
};

class ReadIoRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(ReadIoRequest *req);
	static FsmRetType fsm_state_io_rd(ReadIoRequest *req);
	static FsmRetType fsm_state_start(ReadIoRequest *req);

public:
	FsmRetType (* fsmstate)(ReadIoRequest *);	
	
	virtual ~ReadIoRequest() {}
	virtual FsmRetType exec();
	ReadIoRequest()
	{
		fsmstate = fsm_state_start;
	}

};

class WriteIoRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(WriteIoRequest *req);
	static FsmRetType fsm_state_io_wr(WriteIoRequest *req);
	static FsmRetType fsm_state_start(WriteIoRequest *req);

public:

	unsigned int	output;
	FsmRetType (* fsmstate)(WriteIoRequest *);	
	
	virtual ~WriteIoRequest() {}
	virtual FsmRetType exec();
	WriteIoRequest()
	{
		output	  = 0;
		fsmstate = fsm_state_start;
	}
};

#endif
