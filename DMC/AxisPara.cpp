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
	if (slaveidx > last_slaveidx)
		last_slaveidx = slaveidx;
	++svon_count;
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

void  BaseRef::duplicate()
{
	++rc;
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
	double vel = this->moveparam->getSpeed();
	return vel;
}

double  LinearRef::getMaxDist() const
{
	return this->max_dist;
}

BaseMultiAxisPara::BaseMultiAxisPara(BaseRef *baseref, int sp, int dp)
{
	ref = baseref;
	ref->duplicate();
	startpos = sp;
	dstpos	 = dp;
}

BaseMultiAxisPara::~BaseMultiAxisPara() 
{
	if (ref)
		ref->release();
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

LinearPara::LinearPara(LinearRef *newLineRef, int sp, int dp)
	:BaseMultiAxisPara(newLineRef, sp, dp)
{
}

LinearPara::~LinearPara()
{
}

bool LinearPara::startPlan()
{
	bool success = (0 == this->ref->startPlan());
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

double LinearPara::getCurSpeed()
{
	LinearRef 	*linearRef 	= dynamic_cast<LinearRef *> (this->ref);
	double		velref 		= linearRef->getCurrentVel();			//基准参考速度
	double 		maxdist		= linearRef->getMaxDist();
	
	double v = (this->dstpos - this->startpos) * velref / maxdist;
	return v;
}
