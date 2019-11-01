#include "Plan.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

double sigma = 0.1;

int TParam::position()
{
	int q_i = getPos();

	//printf("S[%d] = %d, q_i = %d, q1=%f.\n", elapsed, q_i, q1);
	++elapsed;
	return q_i;
}

int TParam::getPos() const
{
	int q_i;
	if (elapsed == cycles - 1)
		q_i = (int)((sign == 1)? q1 : -q1);
	else
	{
		double q_f;
		double t = elapsed * 1.0 / CYCLES_PER_SEC;
		q_f = ::Displace_T(t, this);
		q_i = (int)(q_f);				// don't round up
	}
	return q_i;
}

double TParam::getSpeed() const
{
	double v;
	double t = elapsed * 1.0 / CYCLES_PER_SEC;
	v = ::Speed_T(t, this);
	return v;
}

int SParam::position()
{
	int q_i;
	q_i = getPos();
	++elapsed;
	return q_i;
}

int SParam::getPos() const
{
	int q_i;
	if (elapsed == cycles - 1)
		q_i = (int)((sign == 1)? q1 : -q1);
	else
	{
		double q_f;
		double t = elapsed * 1.0 / CYCLES_PER_SEC;

		q_f = ::Displace_S(t, this);
		q_i = (int)(q_f);				// don't round up
	}
	//printf("S[%d] = %d, q_i = %d, q1=%f.\n", elapsed, q_i, q1);
	return q_i;
}

double SParam::getSpeed() const
{
	double v;
	double t = elapsed * 1.0 / CYCLES_PER_SEC;
	v = ::Speed_S(t, this);
	return v;
}

int DParam::position()
{
	int q_i;
	q_i = getPos();


	//printf("D [%d], q_i=%d, q1=%f.\n", elapsed, q_i, q1);
	elapsed++;
	return q_i;
}

int DParam::getPos() const
{
	int q_i;
	if (elapsed == cycles - 1)
		q_i = (int)((sign == 1)? q1 : -q1);

	else
	{
		double q_f;
		double t = (elapsed + 1)* 1.0 / CYCLES_PER_SEC;						//为避免两个下一个周期位置与本次相同，(elapsed+1)
		q_f = ::Displace_D(t, this);
		q_i = (int)(q_f);				// don't round up
	}
	//printf("S[%d] = %d, q_i = %d, q1=%f.\n", elapsed, q_i, q1);
	return q_i;
}

double DParam::getSpeed() const
{
	//todo 
	return 0;
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
	if (fabs(param->q1 - param->q0) < 1e-6)
		return -1;
	
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
	double amax = tp->amax;
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
	if (fabs(tp->q1 - tp->q0) < 1e-6)
		return -1;
	
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
		tp->Ta   = tp->vmax / tp->amax;
		tp->Tv	 = Sv / tp->vmax;
		tp->T	 = tp->Ta + tp->Tv + tp->Ta;
	}
	else
	{//朢??大????度不可达到
		tp->vlim = sqrt((tp->q1 - tp->q0) * tp->amax);
		tp->Ta	= sqrt((tp->q1 - tp->q0) / tp->amax);
		tp->Tv	= 0;
		tp->T	= tp->Ta + 0 + tp->Ta;

	}

	tp->cycles = (int)(tp->T * CYCLES_PER_SEC + 1);

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

