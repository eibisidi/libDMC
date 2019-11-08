#ifndef PLAN_H
#define PLAN_H

#define DC_CYCLE_us 	(2000)							//2000us
#define CYCLES_PER_SEC 	(1000000 / DC_CYCLE_us)			//500 cycle/sec

enum MoveType{
	MOVETYPE_T = 0,
	MOVETYPE_S,
};


//�˶��ƻ�
class MParam
{
public:
	double q0;
	double q1;

	int	   sign;					// (q1-q0)����
	int    cycles;					//T��Ӧ��������
	double T;						//��ʱ��
	int	   elapsed;					//[0~cycles)

	virtual ~MParam() {}
	virtual int position() = 0;						//�����һ���滮λ��
	virtual int position(int ts)  const= 0;			//���ָ��ʱ��滮λ��
	virtual double speed() const = 0;				//��õ�ǰ�ٶ�
	virtual double speed(int ts) const = 0;			//���ָ��ʱ���ٶ�

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

	//�켣����
	double vlim;
	double alima;

	double Tj1;				//�����ʱ��
	double Ta;				//����ʱ��
	double Tv;				//����ʱ��
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

	//�켣����
	double vlim;			//ʵ�ʴﵽ����ٶ�
	double alima;			//ʵ�ʵ��������ٶ�

	double Ta;				//����ʱ��
	double Tv;				//����ʱ��
	virtual int position();
	virtual int position(int ts)  const;
	virtual double speed() const;
	virtual double speed(int ts) const;

	double tofdist(double dist) const;			//�˶���������ʱ�� 
	
	virtual ~TParam() {};

	TParam()
	{
		vmax=amax=vlim=alima=Ta=Tv = 0;
	}
};
 
//�ȼ���
class DParam : public MParam
{
public:
	double amax;			//�����ٶȣ� ����

	double v0;				//��ʼ�ٶ�

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

//S������
double Speed_S(double t, const SParam *param);
double Displace_S(double t, const SParam *param);
int Plan_S( SParam *param);

//��������
double Speed_T(double t, const TParam *tp);
double Displace_T(double t, const TParam *tp);
int Plan_T(TParam *tp);
int Plan_T( TParam *tp, double tlim);

//�ȼ���
double Displace_D(double t, const DParam *dp);
int Plan_D(DParam *dp);


#endif

