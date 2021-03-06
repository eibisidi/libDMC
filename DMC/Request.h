#ifndef REQUEST_H
#define REQUEST_H

#include "AxisPara.h"
#include "RdWrManager.h"
#include "CLogSingle.h"

#include "Poco/Timestamp.h"

#include <vector>
#include <set>

#define DEF_HOME_METHOD     (21)			//缺省回原点方式
#define DEF_HOME_TIMEOUT 	(60)			//缺省回原点超时时间30s
#define DEF_SERVO_POS_BIAS  (100)			//缺省伺服位置达到检测允许误差范围


#define REQUEST_TIMING 1

class DmcManager;

typedef unsigned int FsmRetType;

//配置信息
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
	unsigned int		axis_bias;				//驱动器允许偏差
	SlaveConfig(int index, unsigned char type, unsigned int bias)
			:slave_index(index), slave_type(type), axis_bias(bias)
	{
	}

	SlaveConfig()
	{
		slave_index = 0;
		slave_type	= None;
		axis_bias	= 0;
	}
};
	
class MasterConfig
{
public:
	int						 loglevel;			//日志记录等级
	int						 home_method;		//回原点方式
	int						 home_timeout;		//回原点超时时间
	std::map<int, SlaveConfig>slave_configs;
	std::set<int>			 logpoint_axis;		//记录规划点的轴

	MasterConfig()					//默认等级4记录告警
	{
		clear();
	}

	void clear()
	{
		loglevel 		= 4;
		home_method 	= DEF_HOME_METHOD;
		home_timeout	= DEF_HOME_TIMEOUT;
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
	int				attempts;		//重发次数
	RequestState	reqState;
	DmcManager		*dmc;
	int 			slave_idx;
	transData		*cmdData;
	transData		*respData;
#ifdef REQUEST_TIMING		
	LARGE_INTEGER 	ctime;			//请求创建时间
	LARGE_INTEGER 	stime;			//进入fsm_start的时间
	LARGE_INTEGER   dtime;			//请求析构时间
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


	bool		serveOff;				//停止后是否关闭使能
	DeclStopInfo stopInfo;
	
	DStopRequest(int axis, double Tdec, bool svoff)
	{
		slave_idx 			= axis;
		stopInfo.decltime 	= Tdec;
		serveOff 			= svoff;
		fsmstate 			= fsm_state_start;
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

#ifdef REQUEST_TIMING		
	LARGE_INTEGER	start_push_time;			//将CSP命令写入链表开始时刻
	LARGE_INTEGER	end_push_time;				//将CSP命令写入链表结束时刻
	LARGE_INTEGER	queue_empty_time;			//链表消耗空时刻	
	LARGE_INTEGER	in_pos_time;				//位置到达时刻
#endif

public:

	bool			abs;				//相对 绝对
	long			dist;				//相对=位移， 绝对=目标位置
	MoveType		movetype;			//规划类型S/T
	MParam			*moveparam;			//规划参数
	
	double 			maxvel;				//最大速度，单位（脉冲/s)
	double 			maxa;				//最大加速度，单位(脉冲/s^2)
	double 			maxj;				//最大加加速度，单位(脉冲/s^3),仅对S型曲线有效

	int 			startpos;
	int 			dstpos;
	
	FsmRetType (* fsmstate)(MoveRequest *);	
	
	virtual ~MoveRequest();
	
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
	static FsmRetType fsm_state_wait_all_sent(MultiAxisRequest *req);
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

	bool 	keep;/*加速后持续匀速运动*/

#ifdef REQUEST_TIMING		
	LARGE_INTEGER	start_push_time;			//将CSP命令写入链表开始时刻
	LARGE_INTEGER	end_push_time;				//将CSP命令写入链表结束时刻
	LARGE_INTEGER	queue_empty_time;			//链表消耗空时刻 
	LARGE_INTEGER	in_pos_time;				//位置到达时刻
#endif

public:
	BaseMultiAxisPara 	*axispara;

	FsmRetType (* fsmstate)(MultiAxisRequest *);	
	
	virtual ~MultiAxisRequest();
	
	virtual FsmRetType exec();

	MultiAxisRequest(int axis, LinearRef *newLinearRef, int dist, bool abs);				//直线插补
	MultiAxisRequest(int axis, ArchlRef *newArchlRef, int dist, bool abs, bool z);			//Z轴拱门插补
	MultiAxisRequest(int axis, AccRef *newAccRef, long maxvel);								//匀加速后持续运动
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
	char			home_method;		//回原点方式
	unsigned int    high_speed;			//高速
	unsigned int 	low_speed;			//低速
	unsigned int    acc;				//加减速度
	Poco::Timestamp	starttime;			//用于回原点超时判断
	int				home_timeout;		//回原点超时时间,单位s
	
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
	Poco::Timestamp	starttime;			//用于回原点超时判断
	int				home_timeout;		//回原点超时时间,单位s

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
	Poco::Timestamp	starttime;			//用于回原点超时判断
	int				home_timeout;		//回原点超时时间,单位s

	FsmRetType (* fsmstate)(MultiAbortHomeRequest *);	
	
	virtual ~MultiAbortHomeRequest() 
	{
		if (ref)
			ref->release();
	}
	
	virtual FsmRetType exec();

	MultiAbortHomeRequest(int axis, MultiAbortHomeRef *newRef );
};

class MultiDeclRequest : public BaseRequest
{
private:
	static FsmRetType fsm_state_done(MultiDeclRequest *req);
	static FsmRetType fsm_state_svoff(MultiDeclRequest *req);
	static FsmRetType fsm_state_csp(MultiDeclRequest *req);	
	static FsmRetType fsm_state_start(MultiDeclRequest *req);
	static void pushMultiDecl(MultiDeclRequest *req);
	bool  	positionReached(int q , int bias) const;

public:
	class MultiDeclRef
	{
	private:
		int rc;
		int sync_count;
		int first_slave;
		int error;
		std::map<int, DeclStopInfo *> declInfos;
	public:
		MultiDeclRef():rc(0), sync_count(0),first_slave(0),error(0) {}
		virtual ~MultiDeclRef() {}
		void duplicate(int axis, DeclStopInfo * stopInfo)
		{
			++rc;
			declInfos[axis] = stopInfo;
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
			if (!declInfos.empty())
				return (i == (declInfos.begin()->first));
			return false;
		}

		const std::map<int, DeclStopInfo *>& getStopInfos() const
		{
			return declInfos;
		}
	};

	MultiDeclRef * 	ref;
	bool			serveOff;				//停止后是否关闭使能
	DeclStopInfo 	stopInfo;

	FsmRetType (* fsmstate)(MultiDeclRequest *);	
	
	virtual ~MultiDeclRequest() 
	{
		if (ref)
			ref->release();
	}
	
	virtual FsmRetType exec();

	MultiDeclRequest(int axis, MultiDeclRef *newRef, double tdec);
};

/*已废弃*/
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

/*已废弃*/
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

/*测试使用,正向运动到即将正溢位置
使用前修改驱动器一圈分辨率，避免电机过速*/
class MakeOverFlowRequest: public BaseRequest
{
private:
	static FsmRetType fsm_state_done(MakeOverFlowRequest *req);
	static FsmRetType fsm_wait_pos_reached(MakeOverFlowRequest *req);
	static FsmRetType fsm_state_start(MakeOverFlowRequest *req);
	static const int STEP_INC = 0x4000;
	static const int DST_POS  = (0x7FFFFFFF - 10);

	bool  positionReached(int q , int bias) const;
public:

	FsmRetType (* fsmstate)(MakeOverFlowRequest *);	
	
	virtual ~MakeOverFlowRequest() {}
	virtual FsmRetType exec();
	MakeOverFlowRequest()
	{
		fsmstate = fsm_state_start;
	}
};

#endif
