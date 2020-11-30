#include "AxisPara.h"

#include <stdlib.h>
#include <cassert>

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
				//CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
				//		newT->q0, newT->q1, newT->vmax, newT->amax);
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
				//CLogSingle::logError("Plan_S failed, q0=%f, q1=%f, vmax=%f, amax=%f, jmax=%f.", __FILE__, __LINE__,
				//		newS->q0, newS->q1, newS->vmax, newS->amax, newS->jmax);
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

bool LinearRef::lastCycle() const
{//�Ƿ�Ϊ���һ������
	 return (moveparam->elapsed >= moveparam->cycles - 1);
//	return (moveparam->cycles > moveparam->elapsed);
}

double	LinearRef::getDistanceRatio(int slave_index)
{
	if (slave_index == this->last_slaveidx)
	{
		this->cur_ratio = this->moveparam->position() / this->max_dist;
	}

	return this->cur_ratio;
}

double LinearRef::getCurrentVel() const 					//��õ�ǰ�˶����� 
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

bool BaseMultiAxisPara::lastCycle() const
{
	bool more = this->ref->lastCycle();
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
	bool success = (1 == this->ref->startPlan());	//0��δ�滮�� -1�滮ʧ�� 1�滮�ɹ�
	return success;
}

int LinearPara::nextPosition(int slaveidx)
{
	int		nextpos;
	LinearRef *linearRef = dynamic_cast<LinearRef *> (this->ref);
	
	double distRatio = linearRef->getDistanceRatio(slaveidx); // > 0
	
	if (linearRef->lastCycle())
	//���⸡�����������
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
		double		velref 		= linearRef->getCurrentVel();			//��׼�ο��ٶ�
	 	v= (this->dstpos - this->startpos) * velref / maxdist;
	}
	return v;
}

ArchlRef::ArchlRef()
{
	maxvel = 0;
	zmaxvel = 0;
	maxa   = 0;
	maxd	= 0;
	zmaxa	= 0;
	max_dist = 0;
	mirrored	= 0;

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
			if (	(this -> hh > this->zstartpos && this->hh > this->zdstpos)
				|| (this -> hh < this->zstartpos && this->hh < this->zdstpos))
			{
				//������Z�Ṱ���˶������Z����в���У��֤
				if (abs(this->hh - this->zstartpos) < hu
					 || abs(this->hh - this->zdstpos) < hd)
				{
					planned = -1;
					break;
				}
			}
			else
			{//z�������һ�������˶�
				this->mirrored = 1;
				//��֤������Z���Ƿ��ǹ����˶�
				if (abs(this->hh - getMirroredZ(this->zdstpos)) < hu
					 || abs(this->hh - getMirroredZ(this->zdstpos)) < hd)
				{
					planned = -1;
					break;
			 	}
			}
		
			//�滮Z������
			up_param.q0 = this->zstartpos;
			up_param.q1 = this->hh;
			up_param.amax = this->zmaxa;
			up_param.vmax = this->zmaxvel;
			if (-1 == ::Plan_T(&this->up_param))
			{
				//CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
				//		this->up_param.q0, this->up_param.q1, this->up_param.vmax, this->up_param.amax);
				planned = -1;
				break;
			}

			//�滮Z���½�
			down_param.q0 = this->hh;
			down_param.q1 = getMirroredZ(this->zdstpos);
			down_param.amax = this->zmaxa;
			down_param.vmax = this->zmaxvel;
			if (-1 == ::Plan_T(&this->down_param))
			{
				//CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
				//		this->down_param.q0, this->down_param.q1, this->down_param.vmax, this->down_param.amax);
				planned = -1;
				break;
			}

			if (this->max_dist > 1E-6)
			{//ˮƽλ�����ֵ��0
			
				double t0 = up_param.tofdist(this->hu);
				double t1 = down_param.tofdist(abs(this->hh - getMirroredZ(this->zdstpos)) - hd);
				
				//�滮ֱ�߲岹
				double limt = (up_param.T - t0) + t1;
				this->line_param.q0   = 0; 	
				this->line_param.q1   = this->max_dist;
				this->line_param.vmax = this->maxvel;
				this->line_param.amax = this->maxa;
				this->line_param.dmax = this->maxd;
				
				if(-1 == ::Plan_Ta(&this->line_param, limt))
				{
					//CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f, limt=%f.", __FILE__, __LINE__,
					//		this->line_param.q0, this->line_param.q1, this->line_param.vmax, this->line_param.amax, limt);
					planned = -1;
					break;
				}

				if (t0 > 1E-6)
					this->ts0 = (int)(CYCLES_PER_SEC * t0 + 1);							//��һ��֤���������߶�
				else
					this->ts0 = 0;

				if ((t0 + line_param.T - t1) > 1E-6)
					this->ts1 = (int)(CYCLES_PER_SEC * (t0 + line_param.T - t1) + 1);
				else
					this->ts1 = 0;
			}
			else
			{//ˮƽλ�����ֵΪ0
				this->ts0 = 0;
				this->ts1 = up_param.cycles;
			}
			
			//CLogSingle::logInformation("Archl Result For zstartpos=%d, zdstpos=%d, hh=%d, hu=%d, hd=%d, max_dist=%f, maxa=%f, maxvel=%f.", __FILE__, __LINE__, 
			//			this->zstartpos, this->zdstpos, this->hh, this->hu, this->hd, this->max_dist, this->maxa, this->maxvel);
			//CLogSingle::logInformation("up-cycles=%d, down-cycles=%d, line-cycles=%d, ts=%d, total-cycles=%d, total-time=%fs.", __FILE__, __LINE__, 
			//						this->up_param.cycles, this->down_param.cycles, this->line_param.cycles, this->ts1, this->ts1 + this->down_param.cycles,  (this->ts1 + this->down_param.cycles) * 1.0 / CYCLES_PER_SEC);
			//printf("**************************\n");
			//printf("up-cycles=%d, down-cycles=%d, total=%d\n", up_param.cycles, down_param.cycles, up_param.cycles + down_param.cycles);
			//printf("line-cycles=%d, time-limit=%f\n", line_param.cycles, timelimit *(CYCLES_PER_SEC));
			//printf("alima=%f, vlim=%f, amax=%f, vmax=%f\n", line_param.alima, line_param.vlim, line_param.amax, line_param.vmax);
			//printf("**************************\n");

			//printf("totoal-cycles=%d\n", totalCycles());

			planned = 1;
		}while(0);
		
	}
	return planned;
}

int ArchlRef::totalCycles() const
{
	assert(this->ts1 + this->down_param.cycles >= 1);
	return (this->ts1 + this->down_param.cycles);
}

bool ArchlRef::lastCycle() const
{
	return (this->elapsed >= totalCycles() - 1);
}

double	ArchlRef::getLineDistanceRatio(int slave_index)
{
	double			cur_ratio;

	if (this->elapsed < this->ts0)
		cur_ratio = 0.0;												//��δ��ʼ
	else if (this->elapsed >= (this->ts0 + this->line_param.cycles ))
		cur_ratio = 1.0;
	else
		cur_ratio = this->line_param.position(this->elapsed-this->ts0) / this->max_dist;

	//���һ���ᣬ���ӵ�ǰʱ��
	if (slave_index == this->last_slaveidx)
		++this->elapsed;

	return cur_ratio;
}

double ArchlRef::getLineCurrentVel() const							//��õ�ǰ�˶�����
{
	double vel;
	if (this->elapsed < this->ts0)										//��ֱ����
		vel = 0;
	else if (this->elapsed >= (this->ts0 + this->line_param.cycles ))	//��ֱ�½�
		vel = 0;
	else
		vel = this->line_param.speed(this->elapsed - this->ts0);
	
	return vel;
}

double ArchlRef::getLineMaxDist() const
{
	return this->max_dist;
}

double ArchlRef::getZCurrentSpeed() const								//��ȡZ���ٶ�
{
	double speed;
	if (this->elapsed < this->up_param.cycles)			//�����׶�
		speed = this->up_param.speed(this->elapsed);
	else if (this->elapsed >= this->up_param.cycles && this->elapsed < this->ts1)
		speed = 0;
	else	//�½��׶�
		speed = this->down_param.speed(this->elapsed - this->ts1);

	return speed;
}

int   ArchlRef::getZPosition(int slave_index)
{
	int pos;
	if (this->elapsed < this->up_param.cycles)			//�����׶�
		pos = this->up_param.position(this->elapsed);
	else if (this->elapsed >= this->up_param.cycles && this->elapsed < this->ts1)
		pos = this->hh;
	else	//�½��׶�
		pos = getMirroredZ(this->down_param.position(this->elapsed - this->ts1));
	
	//���һ���ᣬ���ӵ�ǰʱ��
	if (slave_index == this->last_slaveidx)
		++this->elapsed;

	return pos;
}

int   ArchlRef:: getMirroredZ(int z)
{
	if (this->mirrored)
	{
		return (this->hh - z + this->hh);
	}

	return z;
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
	bool success = (1 == this->ref->startPlan());	//0��δ�滮�� -1�滮ʧ�� 1�滮�ɹ�
	return success;
}

int ArchlMultiAxisPara::nextPosition(int slaveidx) 	//�����һ���滮λ��
{
	int 	nextpos;
	ArchlRef *archlRef = dynamic_cast<ArchlRef *> (this->ref);

	if (archlRef->lastCycle())//���⸡�����������
		nextpos = this->dstpos;
	else
	{
		if (this->is_zaxis)
		{//Z��
			nextpos = archlRef->getZPosition(slaveidx);
		}
		else
		{//ֱ�߲岹
			double distRatio = archlRef->getLineDistanceRatio(slaveidx); // >= 0
			nextpos = (int)(this->startpos +  distRatio * (this->dstpos - this->startpos) );
		}
	}
	return nextpos;
}

double ArchlMultiAxisPara::getCurSpeed() const 		//���ص�ǰ�ٶȣ��з���
{
	double v = 0;
	ArchlRef *archlRef = dynamic_cast<ArchlRef *> (this->ref);
	if (this->is_zaxis)//Z��
		v = archlRef->getZCurrentSpeed();
	else
	{	
		double		maxdist 	= archlRef->getLineMaxDist();
		if (maxdist > 1E-6)	//�ٶȷ���
		{
			double		velref		= archlRef->getLineCurrentVel();			//��׼�ο��ٶ�
			v = (this->dstpos - this->startpos) * velref / maxdist;
		}
	}
	
	return v;
}
