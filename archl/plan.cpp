#include "Plan.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

double sigma = 0.1;

unsigned int MParam::dist(int ts) const
{
	int pos;
	unsigned int dist;
	pos = position(ts);
	if (this->sign > 0) //正向运动
		dist = pos - (int)this->q0;
	else
		dist = (int)(-this->q0) - pos;
	return dist;
}

int	MParam::toDist(unsigned long key, bool longer)const
{
	int left,right;
	int mid;
	unsigned int d;
	left=0;
	right=this->cycles - 1;
	while(left<=right)
	{
		mid=left+(right-left)/2;
		d = dist(mid);
		if (d == key) return mid;		//exact match
		else if (key < d) right = mid - 1;
		else if (key > d) left = mid + 1;			 
	}

	if (longer)
		return left; //left时刻恰好大于key
	return right;
}


int TParam::position()
{
	int q_i = position(this->elapsed);

	++elapsed;
	return q_i;
}

int TParam::position(int ts) const
{
	int q_i;
	if (ts == this->cycles - 1)
		q_i = (int)((sign == 1)? q1 : -q1);
	else
	{
		double q_f;
		double t = ts * 1.0 / CYCLES_PER_SEC;
		q_f = ::Displace_T(t, this);
		q_i = (int)(q_f);				// don't round up
	}
	return q_i;
}

double TParam::speed() const
{
	double v;
	v = speed(this->elapsed);
	return v;
}

double TParam::speed(int ts) const
{
	double t = ts * 1.0 / CYCLES_PER_SEC;
	double v=::Speed_T(t, this);
	return v;
}

double TParam::tofdist(double dist) const
{
	double t;
	double Smin = (this->vmax * this->vmax) / this->amax;
	double S   = (this->q1 - this->q0);
	if ( S > Smin)
	{
		if (dist <= Smin/2)
			t = sqrt(2 * dist / this->amax);
		else if (dist > Smin/2 && dist <= (S - Smin/2))
			t = this->Ta + (dist - Smin/2) / (this->vmax);
		else
			t = this->Ta + this->Tv + this->Ta - sqrt(2*(S - dist) / this->amax);
	}
	else
	{
		if (dist <= (S / 2))
			t = sqrt((2 * dist) / this->amax);
		else
			t = this->Ta + this->Ta - sqrt(2 *(S - dist) / this->amax );
	}

	return t;
}

int TaParam::position()
{
	int q_i = position(this->elapsed);

	//printf("S[%d] = %d, q_i = %d, q1=%f.\n", elapsed, q_i, q1);
	++elapsed;
	return q_i;
}

int TaParam::position(int ts) const
{
	int q_i;
	if (ts == this->cycles - 1)
		q_i = (int)((sign == 1)? q1 : -q1);
	else
	{
		double q_f;
		double t = ts * 1.0 / CYCLES_PER_SEC;
		q_f = ::Displace_Ta(t, this);
		q_i = (int)(q_f);				// don't round up
	}
	return q_i;
}

double TaParam::speed() const
{
	double v;
	v = speed(this->elapsed);
	return v;
}

double TaParam::speed(int ts) const
{
	double t = ts * 1.0 / CYCLES_PER_SEC;
	double v=::Speed_Ta(t, this);
	return v;
}

double TaParam::tofdist(double dist) const
{//todo
	double t;
	double Smin = (this->vmax * this->vmax) / this->amax;
	double S   = (this->q1 - this->q0);
	if ( S > Smin)
	{
		if (dist <= Smin/2)
			t = sqrt(2 * dist / this->amax);
		else if (dist > Smin/2 && dist <= (S - Smin/2))
			t = this->Ta + (dist - Smin/2) / (this->vmax);
		else
			t = this->Ta + this->Tv + this->Ta - sqrt(2*(S - dist) / this->amax);
	}
	else
	{
		if (dist <= (S / 2))
			t = sqrt((2 * dist) / this->amax);
		else
			t = this->Ta + this->Ta - sqrt(2 *(S - dist) / this->amax );
	}

	return t;
}

int SParam::position()
{
	int q_i;
	q_i = position(this->elapsed);
	++elapsed;
	return q_i;
}

int SParam::position(int ts)  const
{
	int q_i;
	if (ts == this->cycles - 1)
		q_i = (int)((sign == 1)? q1 : -q1);
	else
	{
		double q_f;
		double t = ts * 1.0 / CYCLES_PER_SEC;

		q_f = ::Displace_S(t, this);
		q_i = (int)(q_f);				// don't round up
	}
	//printf("S[%d] = %d, q_i = %d, q1=%f.\n", elapsed, q_i, q1);
	return q_i;
}

double SParam::speed() const
{
	double v;
	v = speed(this->elapsed);
	return v;
}

double SParam::speed(int ts) const
{
	double t = ts * 1.0 / CYCLES_PER_SEC;
	double v=::Speed_S(t, this);
	return v;
}

int DParam::position()
{
	int q_i;
	q_i = position(this->elapsed);


	//printf("D [%d], q_i=%d, q1=%f.\n", elapsed, q_i, q1);
	elapsed++;
	return q_i;
}

int DParam::position(int ts) const
{
	int q_i;
	if (ts == this->cycles - 1)
		q_i = (int)((sign == 1)? q1 : -q1);

	else
	{
		double q_f;
		double t = (ts + 1)* 1.0 / CYCLES_PER_SEC;						//为避免两个下一个周期位置与本次相同，(elapsed+1)
		q_f = ::Displace_D(t, this);
		q_i = (int)(q_f);				// don't round up
	}
	//printf("S[%d] = %d, q_i = %d, q1=%f.\n", elapsed, q_i, q1);
	return q_i;
}

double DParam::speed() const
{
	double v;
	v = speed(this->elapsed);
	return v;
}

double DParam::speed(int ts) const
{//todo
	double t = ts * 1.0 / CYCLES_PER_SEC;
	double v=0;
	return v;
}

double Speed_S(double t, const SParam *param)
{
	assert(t >= 0);
	
	double v;

	double vlim=param->vlim;
	double alima=param->alima;
	double jmax = param->jmax;
	double Tj1 = param->Tj1;
	double Ta = param->Ta;
	double Tv = param->Tv;
	double T = param->T;

	if (t >= 0 && t < Tj1)							//加加速阶段
		v = jmax * t * t / 2;
	else if ( t >= Tj1 && t < (Ta - Tj1))			//匀加速
		v = alima * (t - Tj1 / 2);
	else if ( t >= (Ta - Tj1) && t < Ta)			//减加速
		v = vlim - jmax * (Ta - t) * (Ta - t)  / 2;	
	else if (t >= Ta && t < Ta + Tv)				//匀速
		v = vlim;
	else if (t >= T - Ta && t < T - Ta + Tj1)
		v = vlim - jmax * (t-T+Ta) *(t - T +Ta) / 2;
	else if (t >= T - Ta + Tj1 && t < T - Tj1)
		v = vlim - alima*(t-T+Ta-Tj1/2);
	else if (t >= T - Tj1 && t < T)
		v = jmax * (t-T)*(t-T) / 2;
	else
		v = 0;

	if (param->sign > 0)
		return v;
	else
		return -v;
}


//S型????度曲线轨迹函数
double Displace_S(double t, const SParam *param)
{
	assert(t >= 0);
	
	double q;
	double q0=param->q0;
	double q1=param->q1;

	double vlim=param->vlim;
	double alima=param->alima;
	double jmax = param->jmax;
	double Tj1 = param->Tj1;
	double Ta = param->Ta;
	double Tv = param->Tv;
	double T = param->T;

	if (t >= 0 && t < Tj1)
		q = q0 + jmax * t * t *t / 6;
	else if ( t >= Tj1 && t < (Ta - Tj1))
		q = q0 + alima / 6 *(3*t*t - 3 * Tj1 * t + Tj1 * Tj1);
	else if ( t >= (Ta - Tj1) && t < Ta)
		q = q0  + vlim * Ta / 2 - vlim * (Ta - t) + jmax * (Ta - t) * (Ta - t) * (Ta - t) / 6;
	else if (t >= Ta && t < Ta + Tv)
		q = q0 + vlim * Ta / 2 + vlim * (t - Ta);
	//else if (t >= T - Ta && t < T - Ta + Tj1)
	else if (t >= Ta + Tv && t < T - Ta + Tj1)
		q = q1 - vlim * Ta / 2 + vlim * (t - T + Ta) - jmax * (t - T + Ta)*(t - T + Ta)*(t - T + Ta) / 6;
	else if (t >= T - Ta + Tj1 && t < T - Tj1)
		q = q1 - vlim * Ta /2 + vlim * (t - T + Ta) - alima / 6 *(3*(t - T + Ta)*(t-T+Ta) - 3 * Tj1 *(t-T+Ta) + Tj1*Tj1);
	else if (t >= T - Tj1 && t < T)
		q = q1 - jmax * (T - t) * (T - t) * (T -t) / 6;
	else
		q = q1;

	if (param->sign > 0)
		return q;
	else
		return -q;
}

//规划S型曲线，计算时间
int Plan_S( SParam *param)
{
	double Tj1, Ta;

	if(fabs(param->vmax) < 1e-6)
		return -1;
	if (fabs(param->amax) < 1e-6)
		return -1;
	if (fabs(param->jmax) < 1e-6)
		return -1;
	//if (fabs(param->q1 - param->q0) < 1e-6)
	//	return -1;									//允许位移为0
	
	param->sign = (param->q1 > param->q0) ? (1) : (-1);
	if (param->sign < 0)
	{
		param->q0 = -(param->q0);
		param->q1 = -(param->q1);
	}
	
	if (param->vmax * param->jmax < param->amax * param->amax)
	{//(3.19)满足,amax不能达到
		Tj1 = sqrt(param->vmax / param->jmax);
		Ta  = 2 * Tj1;
		param->alima = param->jmax * Tj1;
	}
	else
	{//(3.19)不满足，amax能达???
		Tj1 = param->amax / param->jmax;
		Ta  = Tj1 + param->vmax / param->amax;
		param->alima = param->amax;
	}

	double Tv;	//匢??速运动时???
	Tv = (param->q1 - param->q0) / param->vmax - Ta / 2 - Ta / 2; //加????断和减速段对称，时间相???
	if (Tv > 0)
	{//case1,朢??大????度能达???
		param->vlim = param->vmax;
		param->Tj1 	= Tj1;
		param->Ta	= Ta;
		param->Tv	= Tv;
		param->T 	= Ta + Tv + Ta;
	}
	else
	{//case1,朢??大????度不能达到
		//printf("Binary solution.\n");
		//使用二分法减小最大????度求解
		double vl, vh, vmax;

		vl = 0;
		vh = param->vmax;

		do
		{
			vmax = (vl + vh) / 2;

			//璁＄畻Tv
			if (vmax * param->jmax < param->amax * param->amax)
			{//(3.19)满足,amax不能达到
				Tj1 = sqrt(vmax / param->jmax);
				Ta	= 2 * Tj1;
				param->alima = param->jmax * Tj1;
			}
			else
			{//(3.19)不满足，amax能达???
				Tj1 = param->amax / param->jmax;
				Ta	= Tj1 + vmax / param->amax;
				param->alima = param->amax;
			}
			Tv = (param->q1 - param->q0) / vmax - Ta / 2 - Ta / 2; //关于vmax的减函数

			if (Tv < 0)
			{//vmax仍然过大
				vh = vmax;
			}
			else if (Tv > sigma)
			{//vmax过小
				vl = vmax;
			}
			else{
			//可以接受的解
				param->vlim = vmax;
				param->Tj1	= Tj1;
				param->Ta	= Ta;
				param->Tv	= Tv;
				param->T	= Ta + Tv + Ta;

				break;
			}
			
		}while(1);
	}

	param->cycles = (int)(param->T * CYCLES_PER_SEC + 1);
	
	return 0;
}

double Speed_T(double t, const TParam *tp)
{
	assert(t >= 0);
	double v;
	double amax = tp->amax;
	double vmax = tp->vmax;
	double Ta = tp->Ta;
	double Tv	= tp->Tv;
	double T	= tp->T;
	
	if (t>= 0 && t < Ta)						//加????阶???
		v = amax * t;
	else if ( t >= Ta && t < (Ta + Tv)) 		//匢??速阶???
		v = tp->vmax;
	else if (t >= (Ta + Tv) && t < T)			//减????阶???
		v = vmax - amax * (t - Ta - Tv);
	else
		v = 0;

	if (tp->sign > 0)
		return v;
	else
		return -v;
}


//梯形速度曲线轨迹函数
double Displace_T(double t, const TParam *tp)
{
	assert(t >= 0);
	double q;
	double q0   = tp->q0;
	double q1	= tp->q1;
	double vlim = tp->vlim;
	double amax = tp->alima;
	double Ta = tp->Ta;
	double Tv   = tp->Tv;
	double T	= tp->T;
	
	//T???
	if (t>= 0 && t < Ta)						//加????阶???
		q = q0 + 0.5 * amax * t * t;
	else if ( t >= Ta && t < (Ta + Tv))			//匢??速阶???	,三角形此段为单点
		q = q0 + 0.5 * amax * Ta * Ta  + vlim * (t - Ta );
	else if (t >= (Ta + Tv) && t < T)			//减????阶???
		q = q1 -0.5 * amax * (T -t) * (T -t);
	else
		q = q1;

	if (tp->sign > 0)
		return q;
	else
		return -q;
}

//规划梯型曲线，计算时???
int Plan_T( TParam *tp)
{
	if(fabs(tp->vmax) < 1e-6)
		return -1;
	if (fabs(tp->amax) < 1e-6)
		return -1;
	//if (fabs(tp->q1 - tp->q0) < 1e-6)		//允许位移为0
	//	return -1;
	
	tp->sign = (tp->q1 > tp->q0) ? (1) : (-1);
	if (tp->sign < 0)
	{
		tp->q0 = -(tp->q0);
		tp->q1 = -(tp->q1);
	}

	double Smin = (tp->vmax * tp->vmax) / tp->amax;
	double Sv   = (tp->q1 - tp->q0) - Smin;
	if ( Sv > 0)
	{//朢??大????度可达???
		tp->vlim = tp->vmax;
		tp->alima = tp->amax;
		tp->Ta   = tp->vmax / tp->amax;
		tp->Tv	 = Sv / tp->vmax;
		tp->T	 = tp->Ta + tp->Tv + tp->Ta;
	}
	else
	{//朢??大????度不可达到
		tp->vlim = sqrt((tp->q1 - tp->q0) * tp->amax);
		tp->alima = tp->amax;
		tp->Ta	= sqrt((tp->q1 - tp->q0) / tp->amax);
		tp->Tv	= 0;
		tp->T	= tp->Ta + 0 + tp->Ta;

	}

	tp->cycles = (int)(tp->T * CYCLES_PER_SEC + 1);

	return 0;
}

int Plan_T( TParam *tp, double tlim)
{
	if(fabs(tp->vmax) < 1e-6)
		return -1;
	if (fabs(tp->amax) < 1e-6)
		return -1;
	if (fabs(tp->q1 - tp->q0) < 1e-6)				//不允许位移为0
		return -1;

	tp->sign = (tp->q1 > tp->q0) ? (1) : (-1);
	if (tp->sign < 0)
	{
		tp->q0 = -(tp->q0);
		tp->q1 = -(tp->q1);
	}


	double a, al,ah;
	double s;
	al = 0;
	ah = tp->amax;
	s = tp->q1 - tp->q0;
	
	while(true)
	{
		a = (al + ah)/2;
		if (s <= tp->vmax * tp->vmax / a)
		{
			tp->Ta = sqrt(s / a);
			tp->T  = tp->Ta + tp->Ta;

			if (ah - al > 0.001)
			{
				if (tp->T - tlim> sigma) //时间过长
					al = a;
				else if (tp->T < tlim )
					ah = a;
				else
				{
					tp->vlim = tp->Ta * a;
					tp->alima = a;
					tp->Ta  = tp->Ta;
					tp->Tv  = 0;
					tp->T	= tp->Ta + tp->Ta;
					break;
				}
			}
			else
			{
				tp->vlim = tp->Ta * a;
				tp->alima = a;
				tp->Ta  = tp->Ta;
				tp->Tv  = 0;
				tp->T	= tp->T;
				break;
			}
		}
		else
		{
			tp->T = tp->vmax / a + s / tp->vmax;
			if (tp->T > tlim)
			{
				tp->vlim = tp->vmax;
				tp->alima= a;
				tp->Ta	 = tp->vmax / a;
				tp->Tv = (s - tp->vmax * tp->vmax / a) / tp->vmax;
				tp->T	= tp->T;
				break;
			}
			else
				ah = a;
		}
	}

	tp->cycles = (int)(tp->T * CYCLES_PER_SEC + 1);

	return 0;
}

double Speed_Ta(double t, const TaParam *tp)
{//todo
	assert(t >= 0);
	double v;
	double amax = tp->amax;
	double vmax = tp->vmax;
	double Ta = tp->Ta;
	double Tv	= tp->Tv;
	double T	= tp->T;
	
	if (t>= 0 && t < Ta)						//加????阶???
		v = amax * t;
	else if ( t >= Ta && t < (Ta + Tv)) 		//匢??速阶???
		v = tp->vmax;
	else if (t >= (Ta + Tv) && t < T)			//减????阶???
		v = vmax - amax * (t - Ta - Tv);
	else
		v = 0;

	if (tp->sign > 0)
		return v;
	else
		return -v;
}

//非对称梯形速度曲线轨迹函数
double Displace_Ta(double t, const TaParam *tp)
{
	assert(t >= 0);
	double q;
	double q0   = tp->q0;
	double q1	= tp->q1;
	double vlim = tp->vlim;
	double amax = tp->alima;
	double dmax = tp->dlimd;
	double Ta = tp->Ta;
	double Tv   = tp->Tv;
	double T	= tp->T;
	
	if (t>= 0 && t < Ta)						//加速阶段
		q = q0 + 0.5 * amax * t * t;
	else if ( t >= Ta && t < (Ta + Tv))			//匀速阶段
		q = q0 + 0.5 * amax * Ta * Ta  + vlim * (t - Ta );
	else if (t >= (Ta + Tv) && t < T)			//减速阶段
		q = q1 -0.5 * dmax * (T -t) * (T -t);
	else
		q = q1;

	if (tp->sign > 0)
		return q;
	else
		return -q;
}

//规划非对称梯型曲线，计算时间
int Plan_Ta( TaParam *tp)
{
	if(fabs(tp->vmax) < 1e-6)
		return -1;
	if (fabs(tp->amax) < 1e-6)
		return -1;
	if (fabs(tp->dmax) < 1e-6)
		return -1;
	
	tp->sign = (tp->q1 > tp->q0) ? (1) : (-1);
	if (tp->sign < 0)
	{
		tp->q0 = -(tp->q0);
		tp->q1 = -(tp->q1);
	}

	double Smin = ((tp->vmax * tp->vmax) / tp->amax + (tp->vmax * tp->vmax) / tp->dmax) / 2;
	double Sv   = (tp->q1 - tp->q0) - Smin;
	if ( Sv > 0)
	{//最大速度可达
		tp->vlim = tp->vmax;
		tp->alima = tp->amax;
		tp->dlimd = tp->dmax;
		tp->Ta   = tp->vmax / tp->amax;
		tp->Tv	 = Sv / tp->vmax;
		tp->Td	 = tp->vmax / tp->dmax;
		tp->T	 = tp->Ta + tp->Tv + tp->Td;
	}
	else
	{//最大速度不可达
		tp->vlim = sqrt(2 * (tp->q1 - tp->q0)*(tp->amax)*(tp->dmax) / (tp->amax + tp->dmax));
		tp->alima = tp->amax;
		tp->dlimd = tp->dmax;
		tp->Ta	= tp->vlim / tp->amax;
		tp->Tv	= 0;
		tp->Td 	= tp->vlim / tp->dmax;
		tp->T	= tp->Ta + tp->Tv + tp->Td;
	}

	tp->cycles = (int)(tp->T * CYCLES_PER_SEC + 1);

	return 0;
}

//规划非对称梯型曲线，计算时间
int Plan_Ta( TaParam *tp, double tlim)
{
	if(fabs(tp->vmax) < 1e-6)
		return -1;
	if (fabs(tp->amax) < 1e-6)
		return -1;
	if (fabs(tp->dmax) < 1e-6)
		return -1;

	if (0 != Plan_Ta(tp))
		return -1;

	if (tp->T > tlim)	
		return 0;

	//不满足时间约束条件，逐渐减小最大速度
	TaParam tmpTp = *tp;
	double v, vl, vh;
	vl = 0;
	vh = tmpTp.vmax;

	while (vl < vh)
	{
		v  = vl + (vh - vl)/2;
		tmpTp.vmax = v;
		if (0 != Plan_Ta(&tmpTp))
			return -1;

		if(tmpTp.T < tlim) 					//过快
			vh = v;
		else if (tmpTp.T - tlim > sigma)	//过慢
			vl = v;
		else
			break;
	}

	if (tmpTp.T < tlim)
		return -1;

	*tp = tmpTp;
	
	return 0;
}

double Displace_D(double t, const DParam *dp)
{
	assert(t >= 0);
	double q;
	double q0   = dp->q0;
	double q1	= dp->q1;
	double amax = dp->amax;
	double v0   = dp->v0;
	double T	= dp->T;

	if (t>=0 && t < T)
		q = q0 + v0 * t - 0.5 * amax * t * t;
	else
		q = q1;

	if (dp->sign > 0)
		return q;
	else
		return -q;
}

//规划匢??减????曲???
int Plan_D( DParam *dp)
{
	if (fabs(dp->amax) < 1e-6)
		return -1;
	
	dp->sign = (dp->v0 > 0) ? (1) : (-1);
	if (dp->sign < 0)//正向运动
	{
		dp->v0 = -(dp->v0);
		dp->q0 = -(dp->q0);

	}
	dp->q1 = dp->q0 + 0.5 * (dp->v0)*(dp->v0)/(dp->amax);
	dp->q1 = (int)(dp->q1);									//取整
	dp->T = dp->v0 / (dp->amax);
	dp->cycles = (int)(dp->T * CYCLES_PER_SEC + 1);

	return 0;
}

