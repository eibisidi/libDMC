#ifndef AXISPARA_H
#define AXISPARA_H

//多轴插补运动

#include "plan.h"
#include <map>

class BaseMultiAxisPara;

class BaseRef
{
public:
	int 			rc;
	int 			svon_count;					//已经励磁的电机数目
	int				pos_reached_count;			//已经到达位置的电机数目

	int 			last_slaveidx;				//相关轴索引最大的从站，用来计算位置

	int				error;						//错误标记, -1代表某个相关轴运动出现错误
	int				planned;					//0尚未规划  -1规划失败 1规划成功

	std::map<int, BaseMultiAxisPara *> paras;	//所有引用该Ref的运动对象，按轴号索引
	BaseRef();
	virtual ~BaseRef();

	virtual int startPlan() = 0;

	virtual int totalCycles() const = 0;
	virtual bool lastCycle() const = 0;

	int getSvonCount() const;
	int getPosReachedCount() const;
	int getError() const;
	void setError();
	void reg_sv_on(int slaveidx);
	bool sv_allon() const;
	void reg_pos_reached();
	bool pos_allreached() const;
	void duplicate(BaseMultiAxisPara *para, int slaveidx);
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
	virtual int totalCycles() const;
	virtual bool lastCycle() const;
	
	double  getDistanceRatio(int slave_index);	//获得当前运动距离与全程的比例
	double getCurrentVel() const;						//获得当前运动速率
	double getMaxDist() const;
};

class MultiAxisRequest;

class BaseMultiAxisPara
{
public:
	BaseRef			*ref;
	MultiAxisRequest *req;

	int				startpos;			//起始位置
	int				dstpos;				//终止位置
	

	BaseMultiAxisPara(BaseRef *baseref, MultiAxisRequest * mar, int axis, int sp, int dp);
	virtual ~BaseMultiAxisPara() ;
	
	virtual bool startPlan() = 0;
	virtual int nextPosition(int slaveidx) = 0;
	virtual double getCurSpeed()  const = 0;	//返回当前速度，有符号

	int	 totalCycles() const;				//返回规划总周期数
	bool lastCycle() const;
	bool positionReached(int q , int bias) const;

	int getSvonCount() const;
	int getPosReachedCount() const;
	int getError() const;
	void setError();
	void reg_sv_on(int slaveidx);
	bool sv_allon() const;
	void reg_pos_reached();
	bool pos_allreached() const;
};

//直线插补
class LinearPara : public BaseMultiAxisPara
{
public:
	LinearPara(LinearRef *newLineRef, MultiAxisRequest *mar, int axis, int sp, int dp);
	virtual ~LinearPara();

	virtual bool startPlan();
	virtual int nextPosition(int slaveidx);
	virtual double getCurSpeed() const;	//返回当前速度，有符号
};

class ArchlRef: public BaseRef
{
public:
	//约束条件
	double			maxvel;						//最大速度，标量
	double 			maxa;						//最大加速度，标量
	int	 			hu;							//垂直上升距离 >=0
	int	 			hh;							//绝对限高位置
	int	 			hd;							//垂直下降距离 >=0
	double	 		max_dist;					//相关轴最大运动距离
	int				zstartpos;					//Z轴起始绝对位置
	int				zdstpos;					//Z轴终止绝对位置
	
	ArchlRef();
	virtual ~ArchlRef();

	virtual int startPlan();
	virtual int totalCycles() const;
	virtual bool lastCycle() const;
	
	double  getLineDistanceRatio(int slave_index);					//获得直线插补参考轴当前运动距离与全程的比例
	double getLineCurrentVel() const;								//获得直线插补参考轴当前运动速率
	double getLineMaxDist() const;									//获得直线插补参考轴运动距离
	double getZCurrentSpeed() const;								//获取Z轴速度
	int	   getZPosition(int slave_index);							//获取Z轴规划位置
private:
	TParam		   	line_param;					//直线插补规划结果

	TParam			up_param;					//Z轴上升阶段规划结果
	TParam			down_param;					//Z轴下降阶段规划结果

	//double 			t0;							//加速上升时间，此段时间内无水平位移
	//double 			t1;							//加速下降时间，此段时间后无水平位移
	
	int				elapsed;					//当前时刻
	int				ts0;						//水平直线插补运动开始时刻
	int				ts1;						//到达hh后开始下落时刻
};


//Z轴拱门插补
class ArchlMultiAxisPara : public BaseMultiAxisPara
{
public:
	ArchlMultiAxisPara(ArchlRef *newArchlRef, MultiAxisRequest *mar, int axis, int sp, int dp, bool z);
	virtual ~ArchlMultiAxisPara();

	virtual bool startPlan();
	virtual int nextPosition(int slaveidx);		//获得下一个规划位置
	virtual double getCurSpeed() const; 		//返回当前速度，有符号
private:
	bool 	is_zaxis;							//当前轴为Z轴
};

//匀加速
class AccRef: public BaseRef
{
public:
	//约束条件
	double			maxv;						//最大速度，矢量量
	double 			tAcc;						//加速时间
	
	AccRef();
	virtual ~AccRef();

	virtual int startPlan();
	virtual int totalCycles() const;
	virtual bool lastCycle() const;

	int getElpased(int slave_index);
	
private:
	int    cycles;					//T对应的周期数
	int	   elapsed;					//[0~cycles)
};

class AccMultiAxisPara : public BaseMultiAxisPara
{
public:
	AccMultiAxisPara(AccRef *newAcclRef, MultiAxisRequest *mar, int axis, long maxvel, int sp);
	virtual ~AccMultiAxisPara();

	virtual bool startPlan();
	virtual int nextPosition(int slaveidx); 	//获得下一个规划位置
	virtual double getCurSpeed() const; 		//返回当前速度，有符号
private:
	long 	maxv;
	int 	lastpos;
};

#endif
