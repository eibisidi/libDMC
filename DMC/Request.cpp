#include "Request.h"
#include "DmcManager.h"
#include "CLogSingle.h"
#include <assert.h>
#include <ctime>

#define RETRIES (BATCH_WRITE * 50)
#define MAX_ATTEMPTS (5)				
#define MAKE_DWORD(h,l) ((h << 16) | (l))

BaseRequest::BaseRequest()
{
	rechecks 	= RETRIES;
	attempts	= 0;
	reqState	= REQUEST_STATE_BUSY;
	dmc 		= &DmcManager::instance();
	slave_idx	= 0;
	cmdData   = respData = NULL;
}

void DStopRequest::fsm_state_done(DStopRequest *req)
{
}

void DStopRequest::fsm_state_svoff(DStopRequest *req)
{
	if(SV_OFF != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}

	if (req->dmc->isDriverOn(req->slave_idx)
			&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{			
			CLogSingle::logError("DStopRequest::fsm_state_svoff timeouts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		} 
		else
		{
			CLogSingle::logWarning("DStopRequest::fsm_state_svoff Retries. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//fprintf(stdout, "SVOFF StatusWord = 0x%x\n", req->respData->Parm);

	req->fsmstate		= DStopRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_CMD_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
}

void DStopRequest::fsm_state_csp(DStopRequest *req)
{
	if (req->dParam.cycles > req->dParam.elapsed)
	{
		req->cmdData->CMD	= CSP;
		req->cmdData->Data1 = req->dParam.position();
		return;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return;
	}

	if ( !req->positionReached(req->respData->Data1) 
		&& req->rechecks--)
	{
		return;
	}

	if (req->rechecks <= 0						//等待位置到达超时
			&& (!req->dmc->isServo(req->slave_idx)) || (!req->positionReached(req->respData->Data1, req->dmc->getServoPosBias())))
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("DStopRequest::fsm_state_csp timeout. nowpos=%d, dstpos=%d.", __FILE__, __LINE__, (int)req->respData->Data1, req->dstpos);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("DStopRequest::fsm_state_csp retries. nowpos=%d, dstpos=%d.", __FILE__, __LINE__, (int)req->respData->Data1, req->dstpos);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		
		return;
	}

	//fprintf(stdout, "dec Reached. CurrentPos=%d, q1 = %f.\n", req->respData->Data1, req->dParam.q1);

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx, req->dstpos);

	if (req->serveOff)
	{//需关闭电机使能
		req->fsmstate		= DStopRequest::fsm_state_svoff;
		req->cmdData->CMD	= SV_OFF;
		req->rechecks		= RETRIES;
	}
	else
	{//不需关闭电机使能
		req->fsmstate		= fsm_state_done;
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_CMD_STOP);
		req->reqState		= REQUEST_STATE_SUCCESS;
	}
}

void DStopRequest::fsm_state_start(DStopRequest *req)
{
	if (!req->startPlan())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	req->cmdData	= req->dmc->getCmdData(req->slave_idx);
	req->respData	= req->dmc->getRespData(req->slave_idx);
	req->fsmstate	 	= fsm_state_csp;
	req->cmdData->CMD 	= CSP;
	
	req->cmdData->Data1 = req->dParam.position();
}

bool DStopRequest::startPlan()
{
	dParam.q0 = this->startpos;
	dParam.v0 = this->startSpeed;
	dParam.amax = this->maxa;		
	
	if (-1 == ::Plan_D(&dParam))
	{
		CLogSingle::logError("Plan_D failed, axis=%d, q0=%f, v0=%f, amax=%f.", __FILE__, __LINE__,
						this->slave_idx, dParam.q0, dParam.v0, dParam.amax);
		return false;
	}
	this->dstpos = (int)(dParam.sign * dParam.q1);
	return true;
}

bool  DStopRequest::positionReached(int q , int bias) const
{
	if (bias)
		return (::abs(q - this->dstpos) < bias);
	else
		return q == this->dstpos;
}

void DStopRequest::exec()
{
	fsmstate(this);
}

void MoveRequest::fsm_state_done(MoveRequest *req)
{
	//shall not be called
}

void MoveRequest::fsm_state_csp(MoveRequest *req)
{
	if (req->moveparam->cycles > req->moveparam->elapsed)
	{
		req->cmdData->CMD 	= CSP;
		req->cmdData->Data1 = req->curpos = req->moveparam->position();
		return;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return;
	}

	if ( !req->positionReached(req->respData->Data1) 
		&& req->rechecks--)
	{
		return;
	}
		
	if (req->rechecks <= 0						//等待位置到达超时
		&& (!req->dmc->isServo(req->slave_idx)) || (!req->positionReached(req->respData->Data1, req->dmc->getServoPosBias())))
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("MoveRequest::fsm_state_csp timeout. axis=%d, nowpos=%d, lastsentpos=%d, dstpos=%d.", __FILE__, __LINE__,req->slave_idx, (int)req->respData->Data1, req->curpos, req->dstpos);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MoveRequest::fsm_state_csp retries. axis=%d, nowpos=%d, lastsentpos=%d, dstpos=%d.", __FILE__, __LINE__,req->slave_idx, (int)req->respData->Data1, req->curpos, req->dstpos);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx,req->dstpos);
	
	req->fsmstate	 	= fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
}

void MoveRequest::fsm_state_svon(MoveRequest *req)
{
	//fprintf(stdout, "addr = %p, retry = %d, Cmd = 0x%x \n", req->respData, req->rechecks, req->respData->CMD);
	if(SV_ON != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}
	//fprintf(stdout, "SVON StatusWord = 0x%x, CurrentPos=%d.\n", req->respData->Parm,req->respData->Data1);

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("MoveRequest::fsm_state_svon timeouts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MoveRequest::fsm_state_svon retries. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//fprintf(stdout, "SVON StatusWord = 0x%x, CurrentPos=%d.\n", req->respData->Parm,req->respData->Data1);

	if (!req->startPlan())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	req->fsmstate	 	= fsm_state_csp;
	req->cmdData->CMD 	= CSP;
	
	req->cmdData->Data1 = req->curpos = req->moveparam->position();
	req->rechecks		= RETRIES;
}

void MoveRequest::fsm_state_sdowr_cspmode(MoveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || CSP_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("MoveRequest::fsm_state_sdowr_cspmode timeouts. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MoveRequest::fsm_state_sdowr_cspmode retries. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);

	if (req->dmc->isDriverOn(req->slave_idx))
	{//电机已经励磁
		if (!req->startPlan())
		{
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
			req->reqState = REQUEST_STATE_FAIL;
			return;
		}
		req->fsmstate	 	= MoveRequest::fsm_state_csp;
		req->cmdData->CMD 	= CSP;	
		req->cmdData->Data1 = req->curpos = req->moveparam->position();
	}
	else
	{
		req->fsmstate		= MoveRequest::fsm_state_svon;
		req->cmdData->CMD		= SV_ON;
	}
	req->rechecks		= RETRIES;
}

void MoveRequest::fsm_state_wait_sdowr_cspmode(MoveRequest *req)
{
	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("MoveRequest::fsm_state_wait_sdowr_cspmode timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MoveRequest::fsm_state_wait_sdowr_cspmode retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= MoveRequest::fsm_state_sdowr_cspmode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = CSP_MODE;	
	req->rechecks		= RETRIES;
}

void MoveRequest::fsm_state_start(MoveRequest *req)
{	
	req->startpos = req->dmc->getDriverCmdPos(req->slave_idx);//获得起始位置
	if (req->abs)
		req->dstpos = req->dist;
	else
		req->dstpos = req->startpos + req->dist;

	if(req->dmc->isDriverOpCsp(req->slave_idx))
	{//已经处于CSP模式
		req->cmdData 	= req->dmc->getCmdData(req->slave_idx);
		req->respData	= req->dmc->getRespData(req->slave_idx);
		if (req->dmc->isDriverOn(req->slave_idx))
		{//电机已经励磁
			if (!req->startPlan())
			{
				req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
				req->reqState = REQUEST_STATE_FAIL;
				return;
			}
		
			req->fsmstate	 	= MoveRequest::fsm_state_csp;
			req->cmdData->CMD 	= CSP;
			req->cmdData->Data1 = req->curpos = req->moveparam->position();
		}
		else
		{
			req->fsmstate		= MoveRequest::fsm_state_svon;
			req->cmdData->CMD		= SV_ON;
		}
	}
	else 
	{//处于非CSP模式，先切换到CSP模式		
		if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
		{
			req->fsmstate		= MoveRequest::fsm_state_sdowr_cspmode;
			req->cmdData->CMD	= SDO_WR;
			req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
			req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
			req->cmdData->Data2 = CSP_MODE;						
		}
		else
			req->fsmstate		= MoveRequest::fsm_state_wait_sdowr_cspmode;		
	}
}

bool MoveRequest::startPlan()
{
	if (MOVETYPE_T == this->movetype)
	{
		TParam *newT = new TParam;
		newT->q0   = this->startpos;		
		newT->q1   = this->dstpos;
		newT->vmax = this->maxvel;
		newT->amax = this->maxa;
		if (-1 == ::Plan_T(newT))
		{
			CLogSingle::logError("Plan_T failed, axis=%d, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
						this->slave_idx, newT->q0, newT->q1, newT->vmax, newT->amax);
			return false;
		}
		this->moveparam = newT;
	}
	else
	{
		SParam *newS = new SParam;
		newS->q0   = this->startpos;		
		newS->q1   = this->dstpos;
		newS->vmax = this->maxvel;
		newS->amax = this->maxa;
		newS->jmax = this->maxj;			//最大加加速度
		if (-1 == ::Plan_S(newS))
		{
			CLogSingle::logError("Plan_S failed, axis=%d, q0=%f, q1=%f, vmax=%f, amax=%f, jmax=%f.", __FILE__, __LINE__,
						this->slave_idx, newS->q0, newS->q1, newS->vmax, newS->amax, newS->jmax);
			return false;
		}
		this->moveparam = newS;
	}

	return true;
}

bool  MoveRequest::positionReached(int q , int bias) const
{
	if (bias)
		return (::abs(q - this->dstpos) < bias);
	else
		return q == this->dstpos;
}

double  MoveRequest::getCurSpeed() const
{
	if (NULL == this->moveparam)
		return 0;

	return this->moveparam->speed();
}

int     MoveRequest::getCurPos()const
{
	return this->curpos;
}

void MoveRequest::exec()
{
	fsmstate(this);
}

void ClrAlarmRequest::fsm_state_done(ClrAlarmRequest *req)
{
	//shall not be called
}

void ClrAlarmRequest::fsm_state_sdord_errcode(ClrAlarmRequest *req)
{
	if(SDO_RD != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x603F0000 != req->respData->Data1 || 0x00 != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("ClrAlarmRequest::fsm_state_sdord_errcode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{			
			CLogSingle::logWarning("ClrAlarmRequest::fsm_state_sdord_errcode retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= ClrAlarmRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
}

void ClrAlarmRequest::fsm_state_wait_sdord_errcode(ClrAlarmRequest *req)
{
	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("ClrAlarmRequest::fsm_state_wait_sdord_errcode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("ClrAlarmRequest::fsm_state_wait_sdord_errcode retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= ClrAlarmRequest::fsm_state_sdord_errcode;
	req->cmdData->CMD	= SDO_RD;
	req->cmdData->Parm	= 0x0200 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x603F0000;
	req->rechecks		= RETRIES;
}

void ClrAlarmRequest::fsm_state_alm_clr(ClrAlarmRequest *req)
{
	if(ALM_CLR != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("ClrAlarmRequest::fsm_state_alm_clr timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("ClrAlarmRequest::fsm_state_wait_sdord_errcode retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
	{		
		req->fsmstate		= ClrAlarmRequest::fsm_state_sdord_errcode;
		req->cmdData->CMD	= SDO_RD;
		req->cmdData->Parm	= 0x0200 | (req->slave_idx & 0xFF); 			//size | slaveidx
		req->cmdData->Data1 = 0x603F0000;
		req->rechecks		= RETRIES;
	}
	else
		req->fsmstate  = ClrAlarmRequest::fsm_state_wait_sdord_errcode;
}

void ClrAlarmRequest::fsm_state_start(ClrAlarmRequest *req)
{	
	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= ClrAlarmRequest::fsm_state_wait_sdord_errcode;
	req->cmdData->CMD	= ALM_CLR;
}

void ClrAlarmRequest::exec()
{
	fsmstate(this);
}

void InitSlaveRequest::fsm_state_done(InitSlaveRequest *req)
{
	//shall not be called
}

void InitSlaveRequest::fsm_state_sdowr(InitSlaveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  MAKE_DWORD(req->iter->sdo_index,req->iter->sdo_subindex) != req->respData->Data1 || req->iter->sdo_value != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			CLogSingle::logError("InitSlaveRequest::fsm_state_sdowr timeout, axis=%d, sdo=0x%?x.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("InitSlaveRequest::fsm_state_sdowr retries, axis=%d, sdo=0x%?x.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	CLogSingle::logInformation("InitSlaveRequest::fsm_state_sdowr succeed, axis=%d, sdo=0x%?x, value=%?d.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex), req->iter->sdo_value);

	if (++req->iter == req->iterEnd)
	{
		//free sdo
		req->dmc->freeSdoCmdResp(req);
		req->cmdData		= req->dmc->getCmdData(req->slave_idx);
		req->respData		= req->dmc->getRespData(req->slave_idx);
		req->fsmstate		= InitSlaveRequest::fsm_state_done;
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
		req->reqState		= REQUEST_STATE_SUCCESS;
	}
	else
	{
		req->fsmstate		= InitSlaveRequest::fsm_state_sdowr;
		req->cmdData->CMD	= SDO_WR;
		req->cmdData->Parm	= (req->iter->sdo_size << 8) | (req->slave_idx & 0xFF); 			//size | slaveidx
		req->cmdData->Data1 = MAKE_DWORD(req->iter->sdo_index,req->iter->sdo_subindex); 		//index 0x6091 subindex 0x0001
		req->cmdData->Data2 = (req->iter->sdo_value);										//分辨率比值	
		req->rechecks		= RETRIES;
	}
}

void InitSlaveRequest::fsm_state_wait_sdowr(InitSlaveRequest *req)
{
	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{		
			CLogSingle::logError("InitSlaveRequest::fsm_state_wait_sdowr timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logError("InitSlaveRequest::fsm_state_wait_sdowr retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= InitSlaveRequest::fsm_state_sdowr;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= (req->iter->sdo_size << 8) | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = MAKE_DWORD(req->iter->sdo_index,req->iter->sdo_subindex);
	req->cmdData->Data2 = (req->iter->sdo_value);
	req->rechecks		= RETRIES;
}

void InitSlaveRequest::fsm_state_start(InitSlaveRequest *req)
{	
	if (req->iter == req->iterEnd)
	{
		req->fsmstate		= InitSlaveRequest::fsm_state_done;
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
		req->reqState		= REQUEST_STATE_SUCCESS;
		return;
	}
		
	if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
	{
		req->fsmstate		= InitSlaveRequest::fsm_state_sdowr;
		req->cmdData->CMD	= SDO_WR;
		req->cmdData->Parm	= (req->iter->sdo_size << 8) | (req->slave_idx & 0xFF); 			//size | slaveidx
		req->cmdData->Data1 = (req->iter->sdo_index << 16) | (req->iter->sdo_subindex);			//index 0x6091 subindex 0x0001
		req->cmdData->Data2 = (req->iter->sdo_value);								
	}
	else
		req->fsmstate		= InitSlaveRequest::fsm_state_wait_sdowr;		
}

void InitSlaveRequest::exec()
{
	fsmstate(this);
}

void ServoOnRequest::fsm_state_done(ServoOnRequest *req)
{
}

void ServoOnRequest::fsm_state_svon(ServoOnRequest *req)
{
	if(SV_ON != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}

	//printf("status = 0x%x\n", req->dmc->getDriverStatus(req->slave_idx));

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("ServoOnRequest::fsm_state_svon timeouts, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("ServoOnRequest::fsm_state_svon retries, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//初始化电机位置
	req->dmc->setDriverCmdPos(req->slave_idx, (long)req->respData->Data1);

	req->fsmstate	 	= ServoOnRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;

}

void ServoOnRequest::fsm_state_svoff(ServoOnRequest *req)
{
	if(SV_OFF != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}

	if (!req->dmc->isDriverOff(req->slave_idx)
		&& req->rechecks--)
	{
		return;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("ServoOnRequest::fsm_state_svoff timeout, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("ServoOnRequest::fsm_state_svoff timeout, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= ServoOnRequest::fsm_state_svon;
	req->cmdData->CMD	= SV_ON;
	req->rechecks		= RETRIES;
}

void ServoOnRequest::fsm_state_sdowr_cspmode(ServoOnRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || CSP_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("ServoOnRequest::fsm_state_sdowr_cspmode timeout.", __FILE__, __LINE__);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("ServoOnRequest::fsm_state_sdowr_cspmode retries.", __FILE__, __LINE__);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= ServoOnRequest::fsm_state_svoff;
	req->cmdData->CMD	= SV_OFF;
	req->rechecks		= RETRIES;
}

void ServoOnRequest::fsm_state_wait_sdowr_cspmode(ServoOnRequest *req)
{
	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("ServoOnRequest::fsm_state_wait_sdowr_cspmode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logError("ServoOnRequest::fsm_state_wait_sdowr_cspmode retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= ServoOnRequest::fsm_state_sdowr_cspmode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = CSP_MODE;	
	req->rechecks		= RETRIES;
}

void ServoOnRequest::fsm_state_start(ServoOnRequest *req)
{	
	if(req->dmc->isDriverOpCsp(req->slave_idx))
	{//已经处于CSP模式
		req->cmdData 	= req->dmc->getCmdData(req->slave_idx);
		req->respData	= req->dmc->getRespData(req->slave_idx);

		req->fsmstate		= ServoOnRequest::fsm_state_svoff;
		req->cmdData->CMD		= SV_OFF;
	}
	else 
	{//处于非CSP模式，先切换到CSP模式		
		if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
		{
			req->fsmstate		= ServoOnRequest::fsm_state_sdowr_cspmode;
			req->cmdData->CMD	= SDO_WR;
			req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
			req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
			req->cmdData->Data2 = CSP_MODE;						
		}
		else
			req->fsmstate		= ServoOnRequest::fsm_state_wait_sdowr_cspmode;		
	}
}

void ServoOnRequest::exec()
{
	fsmstate(this);
}

int LineRef::startPlan(double maxvel, double maxa, double maxj, MoveType movetype)
{
	if (0 == _planned)	//尚未规划
	{
		if (MOVETYPE_T == movetype)
		{
			TParam *newT = new TParam;
			newT->q0   = 0;		
			newT->q1   = _max_dist;
			newT->vmax = maxvel;
			newT->amax = maxa;
			_moveparam = newT;
			if(-1 == ::Plan_T(newT))
			{
				CLogSingle::logError("Plan_T failed, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
						newT->q0, newT->q1, newT->vmax, newT->amax);
				_planned = -1;
			}
			else
				_planned = 1;
		}
		else
		{//S型规划
			SParam *newS = new SParam;
			newS->q0   = 0;		
			newS->q1   = _max_dist;
			newS->vmax = maxvel;
			newS->amax = maxa;
			newS->jmax = maxj;			//最大加加速度
			_moveparam = newS;
			if (-1 == ::Plan_S(newS))
			{
				CLogSingle::logError("Plan_S failed, q0=%f, q1=%f, vmax=%f, amax=%f, jmax=%f.", __FILE__, __LINE__,
						newS->q0, newS->q1, newS->vmax, newS->amax, newS->jmax);
				_planned = -1;
			}
			else
				_planned = 1;
		}
		
	}
	return _planned;
}


double  LineRef::getDistanceRatio(int slave_index)
{
	if (slave_index == _last_slaveidx)
	{
		_cur_ratio = _moveparam->position() / _max_dist;
	}

	return _cur_ratio;
}

double LineRef::getCurrentVel() const
{
	return _moveparam->speed();
}

double LineRef::getRefDistance() const
{
	return _max_dist;
}

void LineRequest::fsm_state_done(LineRequest *req)
{
}

void  LineRequest::fsm_state_wait_all_pos_reached(LineRequest *req)
{
	if(0 != req->ref->getError())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if ( !req->ref->pos_allreached() 
		&& req->rechecks--)
	{
		return;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_wait_all_pos_reached timeouts, axis=%d, %d != %d.", __FILE__, __LINE__, req->slave_idx, req->ref->_svon_count, req->ref->_pos_reached_count);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_wait_all_pos_reached retries, axis=%d, %d != %d.", __FILE__, __LINE__, req->slave_idx, req->ref->_svon_count, req->ref->_pos_reached_count);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= LineRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
}

void  LineRequest::fsm_state_csp(LineRequest *req)
{
	if(0 != req->ref->getError())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if (req->ref->moreCycles())
	{
		req->cmdData->CMD	= CSP;
		req->cmdData->Data1 = req->getPosition();
		return;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return;
	}

	if ( !req->positionReached(req->respData->Data1) 
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0						//等待位置到达超时
		&& (!req->dmc->isServo(req->slave_idx)) || (!req->positionReached(req->respData->Data1, req->dmc->getServoPosBias())))
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_csp, nowpos=%d, dstpos=%d.", __FILE__, __LINE__, (int)req->respData->Data1, req->dstpos);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_csp, nowpos=%d, dstpos=%d.", __FILE__, __LINE__, (int)req->respData->Data1, req->dstpos);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx, req->dstpos);

	req->ref->reg_pos_reached();										//位置已经到达
	req->fsmstate		= LineRequest::fsm_state_wait_all_pos_reached;
	req->rechecks	 	= RETRIES;										//!!!重置，检测目标是否到达时，已经可能消耗过多
}

void  LineRequest::fsm_state_wait_all_svon(LineRequest *req)
{
	if(0 != req->ref->getError())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if (!req->ref->sv_allon()
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_wait_all_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_wait_all_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	//所有电机均已经励磁
	if (!req->startPlan())
	{//规划失败
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->ref->setError();
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	req->fsmstate	 	= LineRequest::fsm_state_csp;
	req->cmdData->CMD 	= CSP;
	req->cmdData->Data1 = req->getPosition();
	req->rechecks		= RETRIES;
}

void  LineRequest::fsm_state_svon(LineRequest *req)
{
	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	//fprintf(stdout, "addr = %p, retry = %d, Cmd = 0x%x \n", req->respData, req->rechecks, req->respData->CMD);
	if(SV_ON != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}
	//fprintf(stdout, "SVON StatusWord = 0x%x, CurrentPos=%d.\n", req->respData->Parm,req->respData->Data1);

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_svon timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_svon timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//fprintf(stdout, "SVON StatusWord = 0x%x, CurrentPos=%d.\n", req->respData->Parm,req->respData->Data1);

	req->ref->reg_sv_on(req->slave_idx, req->getDistance());
	req->fsmstate		= LineRequest::fsm_state_wait_all_svon;
	req->rechecks		= RETRIES;
}

void  LineRequest::fsm_state_sdowr_cspmode(LineRequest *req)
{
	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || CSP_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_sdowr_cspmode timeout, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->ref->setError();		
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_sdowr_cspmode timeout, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//fprintf(stdout, "cspmode = %d.\n",req->respData->Data2);

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= LineRequest::fsm_state_svon;
	req->cmdData->CMD	= SV_ON;
	req->rechecks		= RETRIES;
}

void  LineRequest::fsm_state_wait_sdowr_cspmode(LineRequest *req)
{
	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_wait_sdowr_cspmode timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->ref->setError();					
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_wait_sdowr_cspmode retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= LineRequest::fsm_state_sdowr_cspmode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = CSP_MODE; 
	req->rechecks		= RETRIES;
}

void  LineRequest::fsm_state_start(LineRequest *req)
{	
	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	//起始终止位置赋初值
	req->startpos = req->dmc->getDriverCmdPos(req->slave_idx);
	//printf("axis=%d startpos=%d.\n", req->slave_idx, req->startpos);
	if (req->abs)
		req->dstpos = req->dist;
	else
		req->dstpos = req->startpos + req->dist;

	if(req->dmc->isDriverOpCsp(req->slave_idx))
	{//已经处于CSP模式
		req->cmdData	= req->dmc->getCmdData(req->slave_idx);
		req->respData	= req->dmc->getRespData(req->slave_idx);
		if (req->dmc->isDriverOn(req->slave_idx))
		{//电机已经励磁
			req->ref->reg_sv_on(req->slave_idx, req->getDistance());
			req->fsmstate		= LineRequest::fsm_state_wait_all_svon;
		}
		else
		{//电机尚未励磁
			req->fsmstate		= LineRequest::fsm_state_svon;
			req->cmdData->CMD		= SV_ON;
		}
	}
	else 
	{//处于非CSP模式，先切换到CSP模式		
		if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
		{
			req->fsmstate		= LineRequest::fsm_state_sdowr_cspmode;
			req->cmdData->CMD	= SDO_WR;
			req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
			req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
			req->cmdData->Data2 = CSP_MODE; 					
		}
		else
			req->fsmstate		= LineRequest::fsm_state_wait_sdowr_cspmode;		
	}
}

bool LineRequest::startPlan()
{
	assert(this->dir == 0);

	//计算运动方向
	if (this->abs) //绝对运动方式
		this->dir = (this->dist > this->startpos) ? (1):(-1);
	else			//相对运动方式
		this->dir = (this->dist > 0) ? (1): (-1);

	this->distRatio = getDistance() / (this->ref->getRefDistance());

	if (-1 == this->ref->startPlan(this->maxvel, this->maxa, this->maxj, this->movetype))
		return false;

	return true;
}

unsigned int LineRequest::getDistance()
{
	if (!this->abs)	//相对运动方式
		return (this->dist > 0)? (this->dist) : (-this->dist);	
	else			//绝对运动方式
		return ((this->dist > this->startpos) ? (this->dist - this->startpos):(this->startpos - this->dist));
}

int  LineRequest::getPosition()
{
	double distRatio;

	distRatio = this->ref->getDistanceRatio(this->slave_idx); // > 0
	
	if (this->ref->isLastCycle())
	{//避免浮点数计算误差，不使用ref_dist
		curpos = this->dstpos;
	}
	else
	{
		if (this->abs)
			curpos = (int)(this->startpos +  distRatio * (this->dist - this->startpos) );
		else
			curpos = (int)(this->startpos + distRatio * (this->dist));
	}

	return curpos;
}

bool	LineRequest::positionReached(int q, int bias)
{
	if (bias)
		return (::abs(q - this->dstpos) < bias);
	else
		return q == this->dstpos;
}

double LineRequest::getCurSpeed() const 		//当前规划的速度
{
	double velref;		//基准参考速度
	double v;
	velref = this->ref->getCurrentVel();

	v = (this->dir) * (this->distRatio) * velref;
	return v;
}

int    LineRequest::getCurPos()const			//当前规划的位置
{
	return this->curpos;
}

void LineRequest::exec()
{
	fsmstate(this);
}

void MultiAxisRequest::fsm_state_done(MultiAxisRequest *req)
{
}

void  MultiAxisRequest::fsm_state_wait_all_pos_reached(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if ( !req->axispara->pos_allreached() 
		&& req->rechecks--)
	{
		return;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_wait_all_pos_reached timeouts, axis=%d, %d != %d.", __FILE__, __LINE__,
						req->slave_idx, req->axispara->getSvonCount(), req->axispara->getPosReachedCount());
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_wait_all_pos_reached retries, axis=%d, %d != %d.", __FILE__, __LINE__, 
				req->slave_idx, req->axispara->getSvonCount(), req->axispara->getPosReachedCount());
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= MultiAxisRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
}

void  MultiAxisRequest::fsm_state_csp(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if (req->axispara->moreCycles())
	{
		req->cmdData->CMD	= CSP;
		req->cmdData->Data1 = req->curpos = req->axispara->nextPosition(req->slave_idx);
		return;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return;
	}

	if ( !req->positionReached(req->respData->Data1) 
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0						//等待位置到达超时
		&& (!req->dmc->isServo(req->slave_idx)) || (!req->positionReached(req->respData->Data1, req->dmc->getServoPosBias())))
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("LineRequest::fsm_state_csp, axis=%d, nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->slave_idx, (int)req->respData->Data1, req->axispara->dstpos);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("LineRequest::fsm_state_csp retries, axis=%d nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->slave_idx, (int)req->respData->Data1, req->axispara->dstpos);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx, req->axispara->dstpos);

	req->axispara->reg_pos_reached();									//位置已经到达
	req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_pos_reached;
	req->rechecks	 	= RETRIES;										//!!!重置，检测目标是否到达时，已经可能消耗过多
}

void  MultiAxisRequest::fsm_state_wait_all_svon(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if (!req->axispara->sv_allon()
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("MultiAxisRequest::fsm_state_wait_all_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MultiAxisRequest::fsm_state_wait_all_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	//所有电机均已经励磁
	if (!req->startPlan())
	{//规划失败
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->axispara->setError();
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	req->fsmstate	 	= MultiAxisRequest::fsm_state_csp;
	req->cmdData->CMD 	= CSP;
	req->cmdData->Data1 = req->curpos = req->axispara->nextPosition(req->slave_idx);
	req->rechecks		= RETRIES;
}

void  MultiAxisRequest::fsm_state_svon(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	//fprintf(stdout, "addr = %p, retry = %d, Cmd = 0x%x \n", req->respData, req->rechecks, req->respData->CMD);
	if(SV_ON != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}
	//fprintf(stdout, "SVON StatusWord = 0x%x, CurrentPos=%d.\n", req->respData->Parm,req->respData->Data1);

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("MultiAxisRequest::fsm_state_svon timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MultiAxisRequest::fsm_state_svon timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//fprintf(stdout, "SVON StatusWord = 0x%x, CurrentPos=%d.\n", req->respData->Parm,req->respData->Data1);

	req->axispara->reg_sv_on();
	req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_svon;
	req->rechecks		= RETRIES;
}

void  MultiAxisRequest::fsm_state_sdowr_cspmode(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || CSP_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("MultiAxisRequest::fsm_state_sdowr_cspmode timeout, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();		
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MultiAxisRequest::fsm_state_sdowr_cspmode timeout, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//fprintf(stdout, "cspmode = %d.\n",req->respData->Data2);

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= MultiAxisRequest::fsm_state_svon;
	req->cmdData->CMD	= SV_ON;
	req->rechecks		= RETRIES;
}

void  MultiAxisRequest::fsm_state_wait_sdowr_cspmode(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("MultiAxisRequest::fsm_state_wait_sdowr_cspmode timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();					
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("MultiAxisRequest::fsm_state_wait_sdowr_cspmode retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= MultiAxisRequest::fsm_state_sdowr_cspmode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = CSP_MODE; 
	req->rechecks		= RETRIES;
}

void  MultiAxisRequest::fsm_state_start(MultiAxisRequest *req)
{	
	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	//起始终止位置赋初值
	//todo!!!!!!
	if(req->dmc->isDriverOpCsp(req->slave_idx))
	{//已经处于CSP模式
		req->cmdData	= req->dmc->getCmdData(req->slave_idx);
		req->respData	= req->dmc->getRespData(req->slave_idx);
		if (req->dmc->isDriverOn(req->slave_idx))
		{//电机已经励磁
			req->axispara->reg_sv_on();
			req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_svon;
		}
		else
		{//电机尚未励磁
			req->fsmstate		= MultiAxisRequest::fsm_state_svon;
			req->cmdData->CMD		= SV_ON;
		}
	}
	else 
	{//处于非CSP模式，先切换到CSP模式		
		if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
		{
			req->fsmstate		= MultiAxisRequest::fsm_state_sdowr_cspmode;
			req->cmdData->CMD	= SDO_WR;
			req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
			req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
			req->cmdData->Data2 = CSP_MODE; 					
		}
		else
			req->fsmstate		= MultiAxisRequest::fsm_state_wait_sdowr_cspmode;		
	}
}

bool MultiAxisRequest::startPlan()
{
	return this->axispara->startPlan();
}

bool  MultiAxisRequest::positionReached(int q , int bias) const
{
	//return true;
	bool reached = this->axispara->positionReached(q, bias);
	return reached;
}

double MultiAxisRequest::getCurSpeed() const 		//当前规划的速度
{
	double v = this->axispara->getCurSpeed();
	return v;
}

int MultiAxisRequest::getCurPos()const			//当前规划的位置
{
	return this->curpos;
}

void MultiAxisRequest::exec()
{
	fsmstate(this);
}

MultiAxisRequest::MultiAxisRequest(int axis, LinearRef *newLinearRef, int pos, bool abs)
{
	int startpos, dstpos;
	this->slave_idx = axis;
	this->fsmstate = fsm_state_start;

	startpos = DmcManager::instance().getDriverCmdPos(axis);
	if (abs)
		dstpos = pos;
	else
		dstpos = startpos + pos;
	
	this->axispara = new LinearPara(newLinearRef, startpos, dstpos);

	double dist = (dstpos > startpos) ? (dstpos - startpos) : (startpos - dstpos);	
	if (dist > newLinearRef->max_dist)
		newLinearRef->max_dist = dist;				//参考轴运动距离

	if (axis > newLinearRef->last_slaveidx)
		newLinearRef->last_slaveidx = axis;
}

MultiAxisRequest::MultiAxisRequest(int axis, ArchlRef *newArchlRef, int pos, bool abs, bool z)			//Z轴拱门插补
{
	int startpos, dstpos;
	this->slave_idx = axis;
	this->fsmstate = fsm_state_start;

	startpos = DmcManager::instance().getDriverCmdPos(axis);

	if (abs)
		dstpos = pos;
	else
		dstpos = startpos + pos;

	this->axispara = new ArchlMultiAxisPara(newArchlRef, startpos, dstpos, z);

	if (z)
	{
		newArchlRef->zstartpos = startpos;
		newArchlRef->zdstpos   = dstpos;
	}
	else
	{//直线插补轴
		double dist = (dstpos > startpos) ? (dstpos - startpos) : (startpos - dstpos);
		if (dist > newArchlRef->max_dist)
			newArchlRef->max_dist = dist;				//参考轴运动距离
	}
	if (axis > newArchlRef->last_slaveidx)
		newArchlRef->last_slaveidx = axis;
}

void HomeMoveRequest::fsm_state_done(HomeMoveRequest *req)
{
}

void HomeMoveRequest::fsm_state_aborthome(HomeMoveRequest *req)
{
	if(ABORT_HOME != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}

	//req->dmc->setDriverCmdPos(req->slave_idx, 0);

	req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
	req->reqState = REQUEST_STATE_FAIL;
}

void HomeMoveRequest::fsm_state_gohome(HomeMoveRequest *req)
{
	if(GO_HOME != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	bool timeout = req->homeTimeout();				//当前是否超时
		
	if (!req->dmc->isDriverHomed(req->slave_idx)
		&& !timeout)		
	{
		return;
	}

	if (req->rechecks <= 0
		|| timeout)
	{
		CLogSingle::logError("HomeMoveRequest::fsm_state_gohome timeouts, axis=%d, status=0x%?x, curpos=%?d.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx), req->dmc->getCurpos(req->slave_idx));
		req->fsmstate		= HomeMoveRequest::fsm_state_aborthome;
		req->cmdData->CMD	= ABORT_HOME;
		req->rechecks		= RETRIES;
		return;
	}

	CLogSingle::logInformation("Homed, axis=%d.", __FILE__, __LINE__, req->slave_idx);
	req->dmc->setDriverCmdPos(req->slave_idx, 0);

	req->fsmstate		= HomeMoveRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_O_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
}

void HomeMoveRequest::fsm_state_sdowr_acc(HomeMoveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm || 0x609A0000 != req->respData->Data1 ||  req->acc != req->respData->Data2)
		&& req->rechecks-- )
	{
		return;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_sdowr_acc timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("HomeMoveRequest::fsm_state_sdowr_acc retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->fsmstate		= HomeMoveRequest::fsm_state_gohome;
	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->cmdData->CMD	= GO_HOME;
	req->rechecks		= RETRIES;
}

void HomeMoveRequest::fsm_state_sdowr_lowspeed(HomeMoveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm || 0x60990002 != req->respData->Data1 ||  req->low_speed != req->respData->Data2)
		&& req->rechecks-- )
	{
		return;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_sdowr_lowspeed timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("HomeMoveRequest::fsm_state_sdowr_lowspeed retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_acc;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x609A0000;						//index 0x609A subindex 0x0000
	req->cmdData->Data2 = req->acc; 						//加速度		
	req->rechecks		= RETRIES;
}

void HomeMoveRequest::fsm_state_sdowr_highspeed(HomeMoveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm || 0x60990001 != req->respData->Data1 ||  req->high_speed != req->respData->Data2)
		&& req->rechecks-- )
	{
		return;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_sdowr_highspeed timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("HomeMoveRequest::fsm_state_sdowr_highspeed retries, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_lowspeed;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x60990002;						//index 0x6099 subindex 0x0002
	req->cmdData->Data2 = req->low_speed; 					//低速				
	req->rechecks		= RETRIES;
}

void HomeMoveRequest::fsm_state_sdowr_homemethod(HomeMoveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm || 0x60980000 != req->respData->Data1 ||  req->home_method != req->respData->Data2)
		&& req->rechecks-- )
	{
		return;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_sdowr_homemethod timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_sdowr_homemethod timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_highspeed;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x60990001;						//index 0x6099 subindex 0x0001
	req->cmdData->Data2 = req->high_speed; 					//高速		
	req->rechecks		= RETRIES;

}

void HomeMoveRequest::fsm_state_sdowr_homemode(HomeMoveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || HOMING_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("HomeMoveRequest::fsm_state_sdowr_homemode retries, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homemethod;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x60980000;						//index 0x6098 subindex 0x0000
	req->cmdData->Data2 = req->home_method; 				//回原点方式
	req->rechecks		= RETRIES;
}

void HomeMoveRequest::fsm_state_wait_sdowr_homemode(HomeMoveRequest *req)
{
	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_wait_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logWarning("HomeMoveRequest::fsm_state_wait_sdowr_homemode retries, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homemode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm  = 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = HOMING_MODE;					
	req->rechecks		= RETRIES;
}

void HomeMoveRequest::fsm_state_svon(HomeMoveRequest *req)
{
	if(SV_ON != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			CLogSingle::logError("HomeMoveRequest::fsm_state_svon retries, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
	{
		req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homemode;
		req->cmdData->CMD	= SDO_WR;
		req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
		req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
		req->cmdData->Data2 = HOMING_MODE;						
	}
	else
		req->fsmstate		= HomeMoveRequest::fsm_state_wait_sdowr_homemode;		
	req->rechecks		= RETRIES;
}

void HomeMoveRequest::fsm_state_start(HomeMoveRequest *req)
{
	if (req->dmc->isDriverOn(req->slave_idx))
	{//电机已经励磁
		if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
		{
			req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homemode;
			req->cmdData->CMD	= SDO_WR;
			req->cmdData->Parm  = 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
			req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
			req->cmdData->Data2 = HOMING_MODE;						
		}
		else
			req->fsmstate		= HomeMoveRequest::fsm_state_wait_sdowr_homemode;
	}
	else
	{
		req->fsmstate 		= HomeMoveRequest::fsm_state_svon;
		req->cmdData 		= req->dmc->getCmdData(req->slave_idx);
		req->respData 		= req->dmc->getRespData(req->slave_idx);
		req->cmdData->CMD		= SV_ON;
	}
}

bool HomeMoveRequest::homeTimeout() const
{
	return (starttime.elapsed() > (home_timeout * 1000000));
}

void HomeMoveRequest::exec()
{
	fsmstate(this);
}

void ReadIoRequest::fsm_state_done(ReadIoRequest *req)
{
}

void ReadIoRequest::fsm_state_io_rd(ReadIoRequest *req)
{
	if(IO_RD != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{		
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("ReadIoRequest::fsm_state_io_rd timeouts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->reqState = REQUEST_STATE_FAIL;
			req->dmc->setIoRS(req->slave_idx, IORS_TIMEOUT);
		}
		else
		{
			CLogSingle::logWarning("ReadIoRequest::fsm_state_io_rd retries, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= ReadIoRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->dmc->setIoRS(req->slave_idx, IORS_SUCCESS);
}

void ReadIoRequest::fsm_state_start(ReadIoRequest *req)
{
	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate 		= ReadIoRequest::fsm_state_io_rd;
	req->cmdData->CMD 	= IO_RD;
}

void ReadIoRequest::exec()
{
	fsmstate(this);
}

void WriteIoRequest::fsm_state_done(WriteIoRequest *req)
{
}

void WriteIoRequest::fsm_state_io_wr(WriteIoRequest *req)
{
	if(IO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			CLogSingle::logError("WriteIoRequest::fsm_state_io_wr timeouts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->reqState = REQUEST_STATE_FAIL;
			req->dmc->setIoRS(req->slave_idx, IORS_TIMEOUT);
		}
		else
		{
			CLogSingle::logWarning("WriteIoRequest::fsm_state_io_wr timeouts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= WriteIoRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->dmc->setIoRS(req->slave_idx, IORS_SUCCESS);
}

void WriteIoRequest::fsm_state_start(WriteIoRequest *req)
{
	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate 		= WriteIoRequest::fsm_state_io_wr;
	req->cmdData->CMD 	= IO_WR;
	req->cmdData->Data1 = req->output;
}

void WriteIoRequest::exec()
{
	fsmstate(this);
}
