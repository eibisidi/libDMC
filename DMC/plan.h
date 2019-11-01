#ifndef PLAN_H
#define PLAN_H

#define DC_CYCLE_us 	(2000)							//2000us
#define CYCLES_PER_SEC 	(1000000 / DC_CYCLE_us)			//500 cycle/sec

//运动计划
class MParam
{
public:
	double q0;
	double q1;

	int	   sign;					// (q1-q0)符号
	int    cycles;					//T对应的周期数
	double T;						//总时间
	int	   elapsed;

	virtual ~MParam() {}
	virtual int position() = 0;
	virtual int getPos()  const= 0;
	virtual double getSpeed() const = 0;
	//bool positionReached(int q, bool bias = false) const;

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
	virtual int getPos() const;
	virtual double getSpeed() const;
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

	double Ta;				//加速时间
	double Tv;				//匀速时间
	virtual int position();
	virtual ~TParam() {};
	virtual int getPos() const;
	virtual double getSpeed() const;
	TParam()
	{
		vmax=amax=vlim=Ta=Tv = 0;
	}
};
 
//匀减速
class DParam : public MParam
{
public:
	double amax;			//最大减速度， 标量

	double v0;				//初始速度

	virtual int position();
	virtual int getPos() const;
	virtual double getSpeed() const;
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

//匀减速
double Displace_D(double t, const DParam *dp);
int Plan_D(DParam *dp);


#endif

