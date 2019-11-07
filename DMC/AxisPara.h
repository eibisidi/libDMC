#ifndef AXISPARA_H
#define AXISPARA_H

#include "plan.h"

class BaseRef
{
public:
	int 			rc;
	int 			svon_count;					//�Ѿ����ŵĵ����Ŀ
	int				pos_reached_count;			//�Ѿ�����λ�õĵ����Ŀ

	int 			last_slaveidx;				//������������Ĵ�վ����������λ��

	int				error;						//������, -1����ĳ��������˶����ִ���
	int				planned;					//0��δ�滮  -1�滮ʧ�� 1�滮�ɹ�
	
	BaseRef();
	virtual ~BaseRef();

	virtual int startPlan() = 0;

	virtual bool moreCycles() const = 0;

	int getSvonCount() const;
	int getPosReachedCount() const;
	int getError() const;
	void setError();
	void reg_sv_on();
	bool sv_allon() const;
	void reg_pos_reached();
	bool pos_allreached() const;
	void duplicate();
	void release();
};

class LinearRef: public BaseRef
{
public:
	MParam		   *moveparam;					//�滮���
	double	 		max_dist;					//���������˶�����
	double			cur_ratio;					//��ǰ�˶�����, ��Χ[0.0,1.0]

	double			maxvel;						//����ٶȣ�����
	double 			maxa;						//�����ٶȣ�����
	double 			maxj;						//���Ӽ��ٶȣ���������S����Ч
	MoveType		movetype;					//�滮����S/T		

	LinearRef();
	virtual ~LinearRef();

	virtual int startPlan();
	virtual bool moreCycles() const;
	
	double  getDistanceRatio(int slave_index);	//��õ�ǰ�˶�������ȫ�̵ı���
	double getCurrentVel() const;						//��õ�ǰ�˶�����
	double getMaxDist() const;
};

class BaseMultiAxisPara
{
public:
	BaseRef			*ref;
	int				startpos;			//��ʼλ��
	int				dstpos;				//��ֹλ��

	BaseMultiAxisPara(BaseRef *baseref, int sp, int dp);
	virtual ~BaseMultiAxisPara() ;
	
	virtual bool startPlan() = 0;
	virtual int nextPosition(int slaveidx) = 0;
	virtual double getCurSpeed()  const = 0;	//���ص�ǰ�ٶȣ��з���

	bool moreCycles() const;
	bool positionReached(int q , int bias) const;

	int getSvonCount() const;
	int getPosReachedCount() const;
	int getError() const;
	void setError();
	void reg_sv_on();
	bool sv_allon() const;
	void reg_pos_reached();
	bool pos_allreached() const;
};

class LinearPara : public BaseMultiAxisPara
{
public:
	LinearPara(LinearRef *newLineRef, int sp, int dp);
	virtual ~LinearPara();

	virtual bool startPlan();
	virtual int nextPosition(int slaveidx);
	virtual double getCurSpeed() const;	//���ص�ǰ�ٶȣ��з���
};

#endif
