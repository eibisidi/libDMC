#ifndef REQUEST_H
#define REQUEST_H

#include "AxisPara.h"
#include "RdWrManager.h"
#include "CLogSingle.h"

#include "Poco/Timestamp.h"

#include <vector>
#include <set>

#define DEF_HOME_METHOD     (21)			//ȱʡ��ԭ�㷽ʽ
#define DEF_HOME_TIMEOUT 	(60)			//ȱʡ��ԭ�㳬ʱʱ��30s
#define DEF_SERVO_POS_BIAS  (100)			//ȱʡ�ŷ�λ�ôﵽ���������Χ


#define REQUEST_TIMING 1

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
#ifdef REQUEST_TIMING
	Poco::Timestamp ctime;			//����ʱ��
#endif
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
		
#ifdef REQUEST_TIMING
			double elapsed =  ctime.elapsed() / 1000000.0;
			double cost = elapsed - moveparam->T;
			LOGSINGLE_INFORMATION("~MoveRequest axis=%?d, plantime=%f, lifetime=%f, cost=%f.", __FILE__, __LINE__, slave_idx,moveparam->T, elapsed, cost);
#endif
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

class MultiHomeRequest : public BaseRequest
{
private:
	static FsmRetType fsm_state_done(MultiHomeRequest *req);
	static FsmRetType fsm_state_gohome(MultiHomeRequest *req);
	static FsmRetType fsm_state_wait_all_sync(MultiHomeRequest *req);
	static FsmRetType fsm_state_wait_1s(MultiHomeRequest *req);
	static FsmRetType fsm_state_sdowr_homemode(MultiHomeRequest *req);
	static FsmRetType fsm_state_wait_sdowr_homemode(MultiHomeRequest *req);
	static FsmRetType fsm_state_svon(MultiHomeRequest *req);
	static FsmRetType fsm_state_start(MultiHomeRequest *req);
	static void pushMultiHome(MultiHomeRequest *req);

	bool homeTimeout() const;

public:
	class MultiHomeRef
	{
	private:
		int rc;
		int sync_count;
		int last_slaveidx;
		int error;
		std::set<int> axises;
		bool started;
	public:
		MultiHomeRef():rc(0), sync_count(0),last_slaveidx(0),error(0),started(false){}
		virtual ~MultiHomeRef() {}
		void duplicate()
		{
			++rc;
		}
		void release() 
		{
			if (--rc == 0)
			{
				delete this;
			}
		}

		void reg_sync(int slaveidx)
		{
			++sync_count;
			if (slaveidx > this->last_slaveidx)
				this->last_slaveidx = slaveidx;
			axises.insert(slaveidx);
		}

		bool all_sync() const
		{
			return sync_count == rc;
		}

		int getError() const
		{
			return error;
		}

		void setError()
		{
			error  = 1;
		}

		bool isLast(int i) const
		{
			return i == last_slaveidx;
		}

		const std::set<int>& getAxises() const
		{
			return axises;
		}

		void setStarted() {started = true;}
		bool getStarted()const {return started;}
	};

	MultiHomeRef * 	ref;
	Poco::Timestamp	starttime;			//���ڻ�ԭ�㳬ʱ�ж�
	int				home_timeout;		//��ԭ�㳬ʱʱ��,��λs

	FsmRetType (* fsmstate)(MultiHomeRequest *);	
	
	virtual ~MultiHomeRequest() 
	{
		if (ref)
			ref->release();
	}
	
	virtual FsmRetType exec();

	MultiHomeRequest(int axis, MultiHomeRef *newRef, int to);
};

class MultiAbortHomeRequest : public BaseRequest
{
private:
	static FsmRetType fsm_state_done(MultiAbortHomeRequest *req);
	static FsmRetType fsm_state_svoff(MultiAbortHomeRequest *req);
	static FsmRetType fsm_state_aborthome(MultiAbortHomeRequest *req);	
	static FsmRetType fsm_state_start(MultiAbortHomeRequest *req);
	static void pushMultiAbortHome(MultiAbortHomeRequest *req);

public:
	class MultiAbortHomeRef
	{
	private:
		int rc;
		int sync_count;
		int first_slave;
		int error;
		std::set<int> axises;
	public:
		MultiAbortHomeRef():rc(0), sync_count(0),first_slave(0),error(0) {}
		virtual ~MultiAbortHomeRef() {}
		void duplicate(int axis)
		{
			++rc;
			axises.insert(axis);
		}
		void release() 
		{
			if (--rc == 0)
			{
				delete this;
			}
		}

		void reg_sync(int slaveidx)
		{
			++sync_count;
		}

		bool all_sync() const
		{
			return sync_count == rc;
		}

		int getError() const
		{
			return error;
		}

		void setError()
		{
			error  = 1;
		}

		bool isFirst(int i) const
		{
			if (!axises.empty())
				return (i == *axises.begin());
			return false;
		}

		const std::set<int>& getAxises() const
		{
			return axises;
		}
	};

	MultiAbortHomeRef * 	ref;
	Poco::Timestamp	starttime;			//���ڻ�ԭ�㳬ʱ�ж�
	int				home_timeout;		//��ԭ�㳬ʱʱ��,��λs

	FsmRetType (* fsmstate)(MultiAbortHomeRequest *);	
	
	virtual ~MultiAbortHomeRequest() 
	{
		if (ref)
			ref->release();
	}
	
	virtual FsmRetType exec();

	MultiAbortHomeRequest(int axis, MultiAbortHomeRef *newRef );
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
