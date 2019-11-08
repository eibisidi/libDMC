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

//ֱ�߲岹
class LinearPara : public BaseMultiAxisPara
{
public:
	LinearPara(LinearRef *newLineRef, int sp, int dp);
	virtual ~LinearPara();

	virtual bool startPlan();
	virtual int nextPosition(int slaveidx);
	virtual double getCurSpeed() const;	//���ص�ǰ�ٶȣ��з���
};

class ArchlRef: public BaseRef
{
public:
	//Լ������
	double			maxvel;						//����ٶȣ�����
	double 			maxa;						//�����ٶȣ�����
	int	 			hu;							//��ֱ�������� >=0
	int	 			hh;							//�����޸�λ��
	int	 			hd;							//��ֱ�½����� >=0
	double	 		max_dist;					//���������˶�����
	int				zstartpos;					//Z����ʼ����λ��
	int				zdstpos;					//Z����ֹ����λ��
	
	ArchlRef();
	virtual ~ArchlRef();

	virtual int startPlan();
	virtual bool moreCycles() const;
	
	double  getLineDistanceRatio(int slave_index);					//���ֱ�߲岹�ο��ᵱǰ�˶�������ȫ�̵ı���
	double getLineCurrentVel() const;								//���ֱ�߲岹�ο��ᵱǰ�˶�����
	double getLineMaxDist() const;									//���ֱ�߲岹�ο����˶�����
	double getZCurrentSpeed() const;								//��ȡZ���ٶ�
	int	   getZPosition(int slave_index);							//��ȡZ��滮λ��
private:
	TParam		   	line_param;					//ֱ�߲岹�滮���

	TParam			up_param;					//Z�������׶ι滮���
	TParam			down_param;					//Z���½��׶ι滮���

	double 			t0;							//��������ʱ�䣬�˶�ʱ������ˮƽλ��
	double 			t1;							//�����½�ʱ�䣬�˶�ʱ�����ˮƽλ��
	
	int				elapsed;					//��ǰʱ��
	int				ts0;						//ˮƽֱ�߲岹�˶���ʼʱ��
	int				ts1;						//����hh��ʼ����ʱ��
};


//Z�Ṱ�Ų岹
class ArchlMultiAxisPara : public BaseMultiAxisPara
{
public:
	ArchlMultiAxisPara(ArchlRef *newArchlRef, int sp, int dp, bool z);
	virtual ~ArchlMultiAxisPara();

	virtual bool startPlan();
	virtual int nextPosition(int slaveidx);		//�����һ���滮λ��
	virtual double getCurSpeed() const; 		//���ص�ǰ�ٶȣ��з���
private:
	bool 	is_zaxis;							//��ǰ��ΪZ��
};

#endif
