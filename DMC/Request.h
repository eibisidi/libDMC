#ifndef REQUEST_H
#define REQUEST_H

#include "stddef.h"
#include "NEXTWUSBLib_12B.h"
#include "plan.h"
#include <vector>
#include <fstream>
#include <set>
#include "Poco/Timestamp.h"

#define DEF_HOME_METHOD     (19)			//缺省回原点方式
#define DEF_HOME_TIMEOUT 	(30)			//缺省回原点超时时间30s
#define DEF_SERVO_POS_BIAS  (10)			//缺省伺服位置达到检测允许误差范围

class DmcManager;

//配置信息
class SDO
{
public:
	unsigned int sdo_index;
	unsigned int sdo_subindex;
	unsigned char sdo_size;
	unsigned int sdo_value;
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
	int						 loglevel;			//日志记录等级
	int						 home_method;		//回原点方式
	int						 home_timeout;		//回原点超时时间
	int						 servo_pos_bias;	//伺服位置达到检测允许误差范围
	std::set<int>			 slave_indexes;
	std::vector<SlaveConfig> slave_configs;

	MasterConfig()					//默认等级4记录告警
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
	}
};


enum RequestState
{
	REQUEST_STATE_NONE = 0,
	REQUEST_STATE_BUSY,
	REQUEST_STATE_SUCCESS,
	REQUEST_STATE_FAIL,
};

enum MoveType{
	MOVETYPE_T = 0,
	MOVETYPE_S,
};


class BaseRequest
{
public:
	int 			retries;
	RequestState	reqState;
	DmcManager		*dmc;
	int 			slave_idx;
	transData		*cmdData;
	transData		*respData;
	virtual ~BaseRequest(){}
	virtual void exec() = 0;
	BaseRequest();
};

class DStopRequest: public BaseRequest
{
private:
	static void fsm_state_done(DStopRequest *req);
	static void fsm_state_svoff(DStopRequest *req);
	static void fsm_state_csp(DStopRequest *req);
	static void fsm_state_start(DStopRequest *req);
	bool 	startPlan();
	bool 	positionReached(int q , int bias = 0) const;
public:
	void (* fsmstate)(DStopRequest *);	
	
	virtual ~DStopRequest() {}
	virtual void exec();
	double 		startSpeed;				//当前速度
	double 		maxa;					//最大减速度
	DParam		dParam;					//减速规划参数
	bool		serveOff;				//停止后是否关闭使能

	int 		startpos;				//起始位置
	int			dstpos;					//终止位置
	DStopRequest()
	{
		startpos = 0;
		dstpos   = 0;
		startSpeed = 0;
		maxa 	 = 0;
		serveOff = false;
		fsmstate = fsm_state_start;
	}
};

class IStopRequest: public BaseRequest
{
private:
	static void fsm_state_start(IStopRequest *req);
	static void fsm_state_svoff(IStopRequest *req);
	static void fsm_state_done(IStopRequest *req);
public:
	void (* fsmstate)(IStopRequest *);	
	
	virtual ~IStopRequest() {}
	virtual void exec();
	IStopRequest()
	{
		fsmstate = fsm_state_start;
	}
};

class MoveRequest: public BaseRequest
{
private:
	static void fsm_state_done(MoveRequest *req);
	static void fsm_state_csp(MoveRequest *req);
	static void fsm_state_svon(MoveRequest *req);
	static void fsm_state_sdowr_cspmode(MoveRequest *req);
	static void fsm_state_wait_sdowr_cspmode(MoveRequest *req);
	static void fsm_state_start(MoveRequest *req);

	bool startPlan();
	bool 	positionReached(int q , int bias = 0) const;

public:

	bool			abs;				//相对 绝对
	long			dist;				//相对=位移， 绝对=目标位置
	MoveType		movetype;			//规划类型S/T
	MParam			*moveparam;			//规划参数
	int				curpos;				//上一次规划的位置,用于减速停止,此位置已经发送到FIFO中
	double 			maxvel;				//最大速度，单位（脉冲/s)
	double 			maxa;				//最大加速度，单位(脉冲/s^2)
	double 			maxj;				//最大加加速度，单位(脉冲/s^3),仅对S型曲线有效

	int 			startpos;
	int 			dstpos;
	
	double getCurSpeed() const;			//当前规划的速度
	int    getCurPos()const;			//当前规划的位置


	
	void (* fsmstate)(MoveRequest *);	
	
	virtual ~MoveRequest() 
	{
		if (moveparam)
		{
			delete moveparam;
		}
	}
	
	virtual void exec();
	MoveRequest()
	{
		abs = false;
		fsmstate = fsm_state_start;
		movetype = MOVETYPE_T;
		moveparam = NULL;
		curpos	 = 0;
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
	static void fsm_state_done(ClrAlarmRequest *req);
	static void fsm_state_sdord_errcode(ClrAlarmRequest *req);
	static void fsm_state_wait_sdord_errcode(ClrAlarmRequest *req);
	static void fsm_state_start(ClrAlarmRequest *req);

public:	
	void (* fsmstate)(ClrAlarmRequest *);	
	
	virtual ~ClrAlarmRequest() 
	{
	}
	
	virtual void exec();
	ClrAlarmRequest()
	{
		fsmstate = fsm_state_start;
	}
};

class InitSlaveRequest: public BaseRequest
{
private:
	static void fsm_state_done(InitSlaveRequest *req);
	static void fsm_state_sdowr(InitSlaveRequest *req);
	static void fsm_state_wait_sdowr(InitSlaveRequest *req);
	static void fsm_state_start(InitSlaveRequest *req);

public:
	std::vector<SDO>::const_iterator iter;
	std::vector<SDO>::const_iterator iterEnd;

	void (* fsmstate)(InitSlaveRequest *);	
	
	virtual ~InitSlaveRequest() 
	{
	}
	
	virtual void exec();
	InitSlaveRequest()
	{
		fsmstate = fsm_state_start;
	}
};


class ServoOnRequest: public BaseRequest
{
private:
	static void fsm_state_done(ServoOnRequest *req);
	static void fsm_state_svon(ServoOnRequest *req);
	static void fsm_state_svoff(ServoOnRequest *req);
	static void fsm_state_sdowr_cspmode(ServoOnRequest *req);
	static void fsm_state_wait_sdowr_cspmode(ServoOnRequest *req);
	static void fsm_state_start(ServoOnRequest *req);

public:	
	void (* fsmstate)(ServoOnRequest *);	
	
	virtual ~ServoOnRequest() 
	{
	}
	
	virtual void exec();
	ServoOnRequest()
	{
		fsmstate = fsm_state_start;
	}
};

class LineRef
{
public:
	int 			_rc;
	int 			_svon_count;				//已经励磁的电机数目
	int				_pos_reached_count;			//已经到达位置的电机数目
	MParam		   *_moveparam;					//规划结果
	int 			_last_slaveidx;				//相关轴索引最大的从站，用来计算位置
	double	 		_max_dist;					//相关轴最大运动距离
	double			_cur_ratio;					//当前运动比率, 范围[0.0,1.0]
	int				_error;						//错误标记, -1代表某个相关轴运动出现错误
	int				_planned;					//0尚未规划  -1规划失败 1规划成功
	LineRef():_rc(0), _svon_count(0),  _last_slaveidx(0), _max_dist(0), _cur_ratio(0.0), _error(0), _planned(0)
	{
		_pos_reached_count = 0;
		_moveparam 		   = NULL;
	}

	virtual ~LineRef()
	{
		if (_moveparam)
		{
			delete _moveparam;
			_moveparam  = NULL;
		}
	}

	int startPlan(double maxvel, double maxa, double maxj, MoveType movetype = MOVETYPE_T);

	bool isLastCycle() const
	{
		return _moveparam->elapsed == _moveparam->cycles - 1;
	}

	bool moreCycles() const
	{
		return (_moveparam->cycles > _moveparam->elapsed);
	}

	int getError() const
	{
		return _error;
	}

	void setError()
	{
		_error = -1;
	}

	void reg_sv_on(int slaveidx, unsigned long dist)
	{
		if (slaveidx > _last_slaveidx)
			_last_slaveidx = slaveidx;
		if (dist > _max_dist)
			_max_dist = dist;

		++_svon_count;
	}

	bool sv_allon() const
	{
		return _svon_count == _rc;
	}

	void reg_pos_reached()
	{
		++_pos_reached_count;
	}

	bool pos_allreached()
	{
		return _pos_reached_count == _svon_count;		//_rc当释放后会减一
	}

	double getDistanceRatio(int slave_index);
	double getCurrentVel() const;
	double getRefDistance() const;
	
	void duplicate()
	{
		++_rc;
	}
	
	void release()
	{
		if (--_rc == 0)
		{
			delete this;
		}
	}
};

class LineRequest: public BaseRequest
{
private:
	static void fsm_state_done(LineRequest *req);
	static void  fsm_state_wait_all_pos_reached(LineRequest *req);
	static void  fsm_state_csp(LineRequest *req);
	static void  fsm_state_svon(LineRequest *req);
	static void  fsm_state_wait_all_svon(LineRequest *req);
	static void  fsm_state_sdowr_cspmode(LineRequest *req);
	static void  fsm_state_wait_sdowr_cspmode(LineRequest *req);
	static void fsm_state_start(LineRequest *req);

	bool	startPlan();
	unsigned int getDistance();		//目标运动距离，正数
	int getPosition();
	bool	positionReached(int curpos, int bias = 0);


public:
	bool			abs;				//相对 绝对
	long			dist;				//相对=位移， 绝对=目标位置
	double			maxvel;				//最大速度，标量
	double 			maxa;				//最大加速度，标量
	double 			maxj;				//最大加加速度，标量，仅S型有效
	MoveType		movetype;			//规划类型S/T		
	LineRef			*ref;				//参考对象

	int				dir;				//运动方向，1正向运动，-1反向运动
	double  		distRatio;			//与参考对象的距离比值
	int				curpos;				//上一次规划的位置

	int				startpos;			//起始位置
	int				dstpos;				//终止位置


	double getCurSpeed() const;			//当前规划的速度
	int    getCurPos()const;			//当前规划的位置

	void (* fsmstate)(LineRequest *);	
	
	virtual ~LineRequest() 
	{
		if (ref)
			ref->release();
	}
	
	virtual void exec();
	LineRequest( LineRef *lineref) : ref(lineref)
	{
		abs = false;
		dist = 0;
		movetype = MOVETYPE_T;
		maxvel = 0;
		maxa   = 0;
		maxj   = 0;
		fsmstate = fsm_state_start;
		startpos = 0;
		ref->duplicate();
		dir	= 0;
		distRatio = 0;
		curpos = 0;
		dstpos = 0;
		
	}
};

class HomeMoveRequest: public BaseRequest
{
private:
	static void fsm_state_done(HomeMoveRequest *req);
	static void fsm_state_aborthome(HomeMoveRequest *req);
	static void fsm_state_gohome(HomeMoveRequest *req);
	static void fsm_state_sdowr_acc(HomeMoveRequest *req);
	static void fsm_state_sdowr_lowspeed(HomeMoveRequest *req);
	static void fsm_state_sdowr_highspeed(HomeMoveRequest *req);
	static void fsm_state_sdowr_homemethod(HomeMoveRequest *req);
	static void fsm_state_sdowr_homemode(HomeMoveRequest *req);
	static void fsm_state_wait_sdowr_homemode(HomeMoveRequest *req);
	static void fsm_state_svon(HomeMoveRequest *req);
	static void fsm_state_start(HomeMoveRequest *req);

	bool homeTimeout() const;
public:
	char			home_method;		//回原点方式
	unsigned int    high_speed;			//高速
	unsigned int 	low_speed;			//低速
	unsigned int    acc;				//加减速度
	Poco::Timestamp	starttime;			//用于回原点超时判断
	int				home_timeout;		//回原点超时时间,单位s
	
	void (* fsmstate)(HomeMoveRequest *);	 
	
	virtual ~HomeMoveRequest() {}
	virtual void exec();
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
	static void fsm_state_done(ReadIoRequest *req);
	static void fsm_state_io_rd(ReadIoRequest *req);
	static void fsm_state_start(ReadIoRequest *req);

public:
	void (* fsmstate)(ReadIoRequest *);	
	
	virtual ~ReadIoRequest() {}
	virtual void exec();
	ReadIoRequest()
	{
		fsmstate = fsm_state_start;
	}

};

class WriteIoRequest: public BaseRequest
{
private:
	static void fsm_state_done(WriteIoRequest *req);
	static void fsm_state_io_wr(WriteIoRequest *req);
	static void fsm_state_start(WriteIoRequest *req);

public:

	unsigned int	output;
	void (* fsmstate)(WriteIoRequest *);	
	
	virtual ~WriteIoRequest() {}
	virtual void exec();
	WriteIoRequest()
	{
		output	  = 0;
		fsmstate = fsm_state_start;
	}
};


#endif
