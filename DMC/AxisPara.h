#ifndef AXISPARA_H
#define AXISPARA_H

#include "plan.h"

class BaseRef
{
public:
	int 			rc;
	int 			svon_count;					//已经励磁的电机数目
	int				pos_reached_count;			//已经到达位置的电机数目

	int 			last_slaveidx;				//相关轴索引最大的从站，用来计算位置

	int				error;						//错误标记, -1代表某个相关轴运动出现错误
	int				planned;					//0尚未规划  -1规划失败 1规划成功
	
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
	MParam		   *moveparam;					//规划结果
	double	 		max_dist;					//相关轴最大运动距离
	double			cur_ratio;					//当前运动比率, 范围[0.0,1.0]

	double			maxvel;						//最大速度，标量
	double 			maxa;						//最大加速度，标量
	double 			maxj;						//最大加加速度，标量，仅S型有效
	MoveType		movetype;					//规划类型S/T		

	LinearRef();
	virtual ~LinearRef();

	virtual int startPlan();
	virtual bool moreCycles() const;
	
	double  getDistanceRatio(int slave_index);	//获得当前运动距离与全程的比例
	double getCurrentVel() const;						//获得当前运动速率
	double getMaxDist() const;
};

class BaseMultiAxisPara
{
public:
	BaseRef			*ref;
	int				startpos;			//起始位置
	int				dstpos;				//终止位置

	BaseMultiAxisPara(BaseRef *baseref, int sp, int dp);
	virtual ~BaseMultiAxisPara() ;
	
	virtual bool startPlan() = 0;
	virtual int nextPosition(int slaveidx) = 0;
	virtual double getCurSpeed()  const = 0;	//返回当前速度，有符号

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
	virtual double getCurSpeed() const;	//返回当前速度，有符号
};

#endif
