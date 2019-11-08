#ifndef PLAN_H
#define PLAN_H

#define DC_CYCLE_us 	(2000)							//2000us
#define CYCLES_PER_SEC 	(1000000 / DC_CYCLE_us)			//500 cycle/sec

enum MoveType{
	MOVETYPE_T = 0,
	MOVETYPE_S,
};


//运动计划
class MParam
{
public:
	double q0;
	double q1;

	int	   sign;					// (q1-q0)符号
	int    cycles;					//T对应的周期数
	double T;						//总时间
	int	   elapsed;					//[0~cycles)

	virtual ~MParam() {}
	virtual int position() = 0;						//获得下一个规划位置
	virtual int position(int ts)  const= 0;			//获得指定时间规划位置
	virtual double speed() const = 0;				//获得当前速度
	virtual double speed(int ts) const = 0;			//获得指定时间速度

	MParam()
	{
		q0 = q1 = T = 0;
		elapsed = sign = cycles = 0;
	}
};

class SParam:public MParam
{
public:
	double vmax;
	double amax;
	double jmax;

	//轨迹参数
	double vlim;
	double alima;

	double Tj1;				//变加速时间
	double Ta;				//加速时间
	double Tv;				//匀速时间
	virtual int position();
	virtual int position(int ts)  const;
	virtual double speed() const;
	virtual double speed(int ts) const;
	virtual ~SParam() {}
	SParam()
	{
		vlim = alima = Tj1 = Ta = Tv = 0;
		vmax = amax = jmax = 0;
	}
};

class TParam:public MParam
{
public:
	double vmax;
	double amax;

	//轨迹参数
	double vlim;			//实际达到最大速度
	double alima;			//实际到达最大加速度

	double Ta;				//加速时间
	double Tv;				//匀速时间
	virtual int position();
	virtual int position(int ts)  const;
	virtual double speed() const;
	virtual double speed(int ts) const;

	double tofdist(double dist) const;			//运动距离消耗时间 
	
	virtual ~TParam() {};

	TParam()
	{
		vmax=amax=vlim=alima=Ta=Tv = 0;
	}
};
 
//匀减速
class DParam : public MParam
{
public:
	double amax;			//最大减速度， 标量

	double v0;				//初始速度

	virtual int position();
	virtual int position(int ts)  const;
	virtual double speed() const;
	virtual double speed(int ts) const;
	virtual ~DParam() {};
	DParam()
	{
		v0 = amax = 0;
	}
};

//S型曲线
double Speed_S(double t, const SParam *param);
double Displace_S(double t, const SParam *param);
int Plan_S( SParam *param);

//梯形曲线
double Speed_T(double t, const TParam *tp);
double Displace_T(double t, const TParam *tp);
int Plan_T(TParam *tp);
int Plan_T( TParam *tp, double tlim);

//匀减速
double Displace_D(double t, const DParam *dp);
int Plan_D(DParam *dp);


#endif

