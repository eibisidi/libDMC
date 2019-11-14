#include "AxisPara.h"
#include "CLogSingle.h"

#include <stdlib.h>

BaseRef::BaseRef()
{
	rc			= 0;
	svon_count	= 0;
	pos_reached_count	= 0;
	last_slaveidx		= 0;
	error		= 0;
	planned 	= 0;
}

BaseRef::~BaseRef()
{

}

int BaseRef::getSvonCount() const
{
	return svon_count;
}

int BaseRef::getPosReachedCount() const
{
	return pos_reached_count;
}

int BaseRef::getError() const
{
	return error;
}

void BaseRef::setError()
{
	error = -1;
}

void BaseRef::reg_sv_on(int slaveidx)
{
	++svon_count;
	if (slaveidx > this->last_slaveidx)
		this->last_slaveidx = slaveidx;
}

bool BaseRef::sv_allon() const
{
	return svon_count == rc;
}

void  BaseRef::reg_pos_reached()
{
	++pos_reached_count;
}

bool  BaseRef::pos_allreached() const
{
	return pos_reached_count == svon_count;
}

void  BaseRef::duplicate(BaseMultiAxisPara *para)
{
	++rc;
	paras.insert(para);
}

void  BaseRef::release()
{
	if (--rc == 0)
	{
		delete this;
	}
}

LinearRef::LinearRef()
{
	moveparam  = NULL;
	max_dist	= 0;
	cur_ratio	= 0;
	maxvel		= 0;
	maxa		= 0;
	maxj		= 0;
	movetype	= MOVETYPE_T;
}

LinearRef::~LinearRef() 
{
	if (moveparam)
	{
		delete moveparam;
		moveparam  = NULL;
	}
}

int LinearRef::startPlan()
{
	if (0 == planned)
	{
		if (MOVETYPE_T == movetype)
		{
			TParam *newT = new TParam;
			newT->q0   = 0; 	
			newT->q1   = this->max_dist;
			newT->vmax = this->maxvel;
			newT->amax = this->maxa;
			this->moveparam = newT;
			if(-1 == ::Plan_T(newT))
			{
				CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
						newT->q0, newT->q1, newT->vmax, newT->amax);
				planned = -1;
			}
			else
				planned = 1;
		}
		else
		{
			SParam *newS = new SParam;
			newS->q0   = 0; 	
			newS->q1   = max_dist;
			newS->vmax = maxvel;
			newS->amax = maxa;
			newS->jmax = maxj;
			moveparam = newS;
			if (-1 == ::Plan_S(newS))
			{
				CLogSingle::logError("Plan_S failed, q0=%f, q1=%f, vmax=%f, amax=%f, jmax=%f.", __FILE__, __LINE__,
						newS->q0, newS->q1, newS->vmax, newS->amax, newS->jmax);
				planned = -1;
			}
			else
				planned = 1;
		}
		
	}
	return planned;
}

int LinearRef::totalCycles() const
{
	return (moveparam->cycles);
}

bool LinearRef::moreCycles() const
{
	return (moveparam->cycles > moveparam->elapsed);
}

double	LinearRef::getDistanceRatio(int slave_index)
{
	if (slave_index == this->last_slaveidx)
	{
		this->cur_ratio = this->moveparam->position() / this->max_dist;
	}

	return this->cur_ratio;
}

double LinearRef::getCurrentVel() const 					//获得当前运动速率 
{
	double vel = this->moveparam->speed();
	return vel;
}

double  LinearRef::getMaxDist() const
{
	return this->max_dist;
}

BaseMultiAxisPara::BaseMultiAxisPara(BaseRef *baseref, MultiAxisRequest * mar, int sp, int dp)
{
	ref = baseref;
	ref->duplicate(this);
	req		= mar;
	startpos = sp;
	dstpos	 = dp;
}

BaseMultiAxisPara::~BaseMultiAxisPara() 
{
	if (ref)
		ref->release();
}

int  BaseMultiAxisPara::totalCycles() const
{
	int cycles = this->ref->totalCycles();
	return cycles;
}

bool BaseMultiAxisPara::moreCycles() const
{
	bool more = this->ref->moreCycles();
	return more;
}

bool BaseMultiAxisPara::positionReached(int q , int bias) const
{
	if (bias)
		return (::abs(q - this->dstpos) < bias);
	else
		return q == this->dstpos;
}

int BaseMultiAxisPara::getSvonCount() const
{
	int svoncount = this->ref->getSvonCount();
	return svoncount;
}

int BaseMultiAxisPara::getPosReachedCount() const
{
	int posReachedCount = this->ref->getPosReachedCount();
	return posReachedCount;
}

int	 BaseMultiAxisPara::getError() const
{
	int error = this->ref->getError();
	return error;
}

void BaseMultiAxisPara::setError()
{
	this->ref->setError();
}

void BaseMultiAxisPara::reg_sv_on(int slaveidx)
{
	this->ref->reg_sv_on(slaveidx);
}

bool BaseMultiAxisPara::sv_allon() const
{
	bool allon = this->ref->sv_allon();
	return allon;
}

void BaseMultiAxisPara::reg_pos_reached()
{
	this->ref->reg_pos_reached();
}

bool BaseMultiAxisPara::pos_allreached() const
{
	bool allreached = this->ref->pos_allreached();
	return allreached;
}

LinearPara::LinearPara(LinearRef *newLineRef, MultiAxisRequest *mar, int sp, int dp)
	:BaseMultiAxisPara(newLineRef,mar, sp, dp)
{
}

LinearPara::~LinearPara()
{
}

bool LinearPara::startPlan()
{
	bool success = (1 == this->ref->startPlan());	//0尚未规划， -1规划失败 1规划成功
	return success;
}

int LinearPara::nextPosition(int slaveidx)
{
	int		nextpos;
	LinearRef *linearRef = dynamic_cast<LinearRef *> (this->ref);
	
	double distRatio = linearRef->getDistanceRatio(slaveidx); // > 0
	
	if (!linearRef->moreCycles())
	//避免浮点数计算误差
		nextpos = this->dstpos;
	else
		nextpos = (int)(this->startpos +  distRatio * (this->dstpos - this->startpos) );

	return nextpos;
}

double LinearPara::getCurSpeed() const
{
	double v = 0;
	LinearRef 	*linearRef 	= dynamic_cast<LinearRef *> (this->ref);
	double 		maxdist		= linearRef->getMaxDist();
	if (maxdist > 1E-6)
	{
		double		velref 		= linearRef->getCurrentVel();			//基准参考速度
	 	v= (this->dstpos - this->startpos) * velref / maxdist;
	}
	return v;
}

ArchlRef::ArchlRef()
{
	maxvel = 0;
	maxa   = 0;
	max_dist = 0;

	//t0 = 0;
	//t1 = 0;
	elapsed = 0;
	ts0 = 0;
	ts1 = 0;
}

ArchlRef::~ArchlRef()
{
}

int ArchlRef::startPlan()
{
	if (0 == planned)
	{
		do{
			//针对Z轴进行参数校验证
			if (this->hh > this->zstartpos)
			{
				if (this->zstartpos + this->hu > this->hh
					|| this->zdstpos + this->hd > this->hh)	//正向运动超限高
				{
					CLogSingle::logError("ArchlRef::startPlan failed, zstartpos=%d, zdstpos=%d, hu=%d, hh=%d, hd=%d.", __FILE__, __LINE__,
						this->zstartpos, this->zdstpos, this->hu, this->hh, this->hd);
					planned = -1;
					break;
				}
			}
			else
			{
				if (this->zstartpos - this->hu < this->hh
					|| this->zdstpos - this->hd < this->hh) //负向运动超限高
				{
					CLogSingle::logError("ArchlRef::startPlan failed, zstartpos=%d, zdstpos=%d, hu=%d, hh=%d, hd=%d.", __FILE__, __LINE__,
						this->zstartpos, this->zdstpos, this->hu, this->hh, this->hd);
					planned = -1;
					break;
				}
			}
		
			//规划Z轴上升
			up_param.q0 = this->zstartpos;
			up_param.q1 = this->hh;
			up_param.amax = this->maxa;
			up_param.vmax = this->maxvel;
			if (-1 == ::Plan_T(&this->up_param))
			{
				CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
						this->up_param.q0, this->up_param.q1, this->up_param.vmax, this->up_param.amax);
				planned = -1;
				break;
			}

			//规划Z轴下降
			down_param.q0 = this->hh;
			down_param.q1 = this->zdstpos;
			down_param.amax = this->maxa;
			down_param.vmax = this->maxvel;
			if (-1 == ::Plan_T(&this->down_param))
			{
				CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
						this->down_param.q0, this->down_param.q1, this->down_param.vmax, this->down_param.amax);
				planned = -1;
				break;
			}

			if (this->max_dist > 1E-6)
			{//水平位移最大值非0
			
				double t0 = up_param.tofdist(this->hu);
				double t1 = down_param.tofdist(abs(this->hh - this->zdstpos) - hd);

				
				//规划直线插补
				double limt = (up_param.T - t0) + t1;
				this->line_param.q0   = 0; 	
				this->line_param.q1   = this->max_dist;
				this->line_param.vmax = this->maxvel;
				this->line_param.amax = this->maxa;
				
				if(-1 == ::Plan_T(&this->line_param, limt))
				{
					CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f, limt=%f.", __FILE__, __LINE__,
							this->line_param.q0, this->line_param.q1, this->line_param.vmax, this->line_param.amax, limt);
					planned = -1;
					break;
				}

				this->ts0 = (int)(CYCLES_PER_SEC * t0 + 1);
				this->ts1 = (int)(CYCLES_PER_SEC * (t0 + line_param.T - t1) + 1);
			}
			else
			{//水平位移最大值为0
				this->ts0 = 0;
				this->ts1 = up_param.cycles;
			}
			
			CLogSingle::logInformation("Archl Result For zstartpos=%d, zdstpos=%d, hh=%d, hu=%d, hd=%d, max_dist=%f, maxa=%f, maxvel=%f.", __FILE__, __LINE__, 
						this->zstartpos, this->zdstpos, this->hh, this->hu, this->hd, this->max_dist, this->maxa, this->maxvel);
			CLogSingle::logInformation("up-cycles=%d, down-cycles=%d, line-cycles=%d, ts=%d, total-cycles=%d, total-time=%fs.", __FILE__, __LINE__, 
									this->up_param.cycles, this->down_param.cycles, this->line_param.cycles, this->ts1, this->ts1 + this->down_param.cycles,  (this->ts1 + this->down_param.cycles) * 1.0 / CYCLES_PER_SEC);
			
			planned = 1;
		}while(0);
		
	}
	return planned;
}

int ArchlRef::totalCycles() const
{
	return (this->ts1 + this->down_param.cycles);
}


bool ArchlRef::moreCycles() const
{
	return (this->elapsed < (this->ts1 + this->down_param.cycles));
}

double	ArchlRef::getLineDistanceRatio(int slave_index)
{
	double			cur_ratio;

	if (this->elapsed < this->ts0)
		cur_ratio = 0.0;												//还未开始
	else if (this->elapsed >= (this->ts0 + this->line_param.cycles ))
		cur_ratio = 1.0;
	else
		cur_ratio = this->line_param.position(this->elapsed-this->ts0) / this->max_dist;

	//最后一个轴，增加当前时刻
	if (slave_index == this->last_slaveidx)
		++this->elapsed;

	return cur_ratio;
}

double ArchlRef::getLineCurrentVel() const							//获得当前运动速率
{
	double vel;
	if (this->elapsed < this->ts0)										//垂直上升
		vel = 0;
	else if (this->elapsed >= (this->ts0 + this->line_param.cycles ))	//垂直下降
		vel = 0;
	else
		vel = this->line_param.speed(this->elapsed - this->ts0);
	
	return vel;
}

double ArchlRef::getLineMaxDist() const
{
	return this->max_dist;
}

double ArchlRef::getZCurrentSpeed() const								//获取Z轴速度
{
	double speed;
	if (this->elapsed < this->up_param.cycles)			//上升阶段
		speed = this->up_param.speed(this->elapsed);
	else if (this->elapsed >= this->up_param.cycles && this->elapsed < this->ts1)
		speed = 0;
	else	//下降阶段
		speed = this->down_param.speed(this->elapsed - this->ts1);

	return speed;
}

int   ArchlRef::getZPosition(int slave_index)
{
	int pos;
	if (this->elapsed < this->up_param.cycles)			//上升阶段
		pos = this->up_param.position(this->elapsed);
	else if (this->elapsed >= this->up_param.cycles && this->elapsed < this->ts1)
		pos = this->hh;
	else	//下降阶段
		pos = this->down_param.position(this->elapsed - this->ts1);
	
	//最后一个轴，增加当前时刻
	if (slave_index == this->last_slaveidx)
		++this->elapsed;

	return pos;
}

ArchlMultiAxisPara::ArchlMultiAxisPara(ArchlRef *newArchlRef, MultiAxisRequest *mar, int sp, int dp, bool z)
	:BaseMultiAxisPara(newArchlRef, mar, sp, dp)
{
	is_zaxis = z;
}
	
ArchlMultiAxisPara::~ArchlMultiAxisPara()
{
	
}

bool ArchlMultiAxisPara::startPlan()
{
	bool success = (1 == this->ref->startPlan());	//0尚未规划， -1规划失败 1规划成功
	return success;
}

int ArchlMultiAxisPara::nextPosition(int slaveidx) 	//获得下一个规划位置
{
	int 	nextpos;
	ArchlRef *archlRef = dynamic_cast<ArchlRef *> (this->ref);

	if (!archlRef->moreCycles())//避免浮点数计算误差
		nextpos = this->dstpos;
	else
	{
		if (this->is_zaxis)
		{//Z轴
			nextpos = archlRef->getZPosition(slaveidx);
		}
		else
		{//直线插补
			double distRatio = archlRef->getLineDistanceRatio(slaveidx); // >= 0
			nextpos = (int)(this->startpos +  distRatio * (this->dstpos - this->startpos) );
		}
	}
	return nextpos;
}

double ArchlMultiAxisPara::getCurSpeed() const 		//返回当前速度，有符号
{
	double v = 0;
	ArchlRef *archlRef = dynamic_cast<ArchlRef *> (this->ref);
	if (this->is_zaxis)//Z轴
		v = archlRef->getZCurrentSpeed();
	else
	{	
		double		maxdist 	= archlRef->getLineMaxDist();
		if (maxdist > 1E-6)	//速度非零
		{
			double		velref		= archlRef->getLineCurrentVel();			//基准参考速度
			v = (this->dstpos - this->startpos) * velref / maxdist;
		}
	}
	
	return v;
}
