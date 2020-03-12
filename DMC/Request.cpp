#include "Request.h"
#include "DmcManager.h"
#include "CLogSingle.h"

#define RETRIES (10 * 50)
#define MAX_ATTEMPTS (5)				
#define MAKE_DWORD(h,l) ((h << 16) | (l))

#define CSP_DATA2_DUMMY (0xFF)
#define RESP_CMD_CODE(respData) ((respData)->CMD & 0xFF)

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
	//shall not be called
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
			LOGSINGLE_ERROR("DStopRequest::fsm_state_svoff timeouts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		} 
		else
		{
			LOGSINGLE_INFORMATION("DStopRequest::fsm_state_svoff reattempts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= DStopRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_CMD_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
}

void DStopRequest::fsm_state_csp(DStopRequest *req)
{
	if(req->dmc->m_rdWrManager.peekQueue(req->slave_idx))
	{//尚未发送完毕
		return;
	}

	//发送已经完成
	if(req->stopInfo.valid)
	{
		if(CSP != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData )
			&& req->rechecks--)
		{
			return;
		}

		int posBias = (req->dmc->isServo(req->slave_idx)) ? (req->dmc->getServoPosBias()) : 0;

		if ( !req->positionReached(req->respData->Data1, posBias) 
			&& req->rechecks--)
		{
			return;
		}

		if (req->rechecks <= 0)						//等待位置到达超时
		{
			if (++req->attempts > MAX_ATTEMPTS)
			{	
				LOGSINGLE_ERROR("DStopRequest::fsm_state_csp timeout. nowpos=%d, endpos=%d.", __FILE__, __LINE__, (int)req->respData->Data1, req->stopInfo.endpos);
				req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
				req->reqState = REQUEST_STATE_FAIL;
			}
			else
			{
				LOGSINGLE_INFORMATION("DStopRequest::fsm_state_csp reattempts. attempts=%d. nowpos=%d, endpos=%d.", __FILE__, __LINE__, req->attempts, (int)req->respData->Data1, req->stopInfo.endpos);
				req->rechecks = RETRIES;
				req->cmdData->CMD 	= CSP;
				req->cmdData->Data1 = req->stopInfo.endpos;	//重发最终位置
				req->cmdData->Data2 = CSP_DATA2_DUMMY;		//特殊处理让充发位置可以进入待发送队列				
			}
			return;
		}
	}

	//LOGSINGLE_INFORMATION("axis = %d Dec Reached, valid=%b, endpos=%d.", __FILE__, __LINE__, req->slave_idx, req->stopInfo.valid, req->stopInfo.endpos);

	//目标位置已到达,更新新的绝对位置
	if(req->stopInfo.valid)
		req->dmc->setDriverCmdPos(req->slave_idx, req->stopInfo.endpos);
	else
		req->dmc->setDriverCmdPos(req->slave_idx, req->dmc->getCurpos(req->slave_idx));
	
	if (req->serveOff)
	{//需关闭电机使能
		req->fsmstate		= DStopRequest::fsm_state_svoff;
		req->cmdData->CMD	= SV_OFF;
	}
	else
	{//不需关闭电机使能
		req->fsmstate		= fsm_state_done;
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_CMD_STOP);
		req->reqState		= REQUEST_STATE_SUCCESS;
	}
	req->rechecks		= RETRIES;
	req->attempts		= 0;
}

void DStopRequest::fsm_state_start(DStopRequest *req)
{
	req->cmdData	= req->dmc->getCmdData(req->slave_idx);
	req->respData	= req->dmc->getRespData(req->slave_idx);
	req->fsmstate	 	= fsm_state_csp;
	req->cmdData->CMD 	= CSP;
	
	//将减速命令加入到队列中
	req->dmc->m_rdWrManager.declStop(req->slave_idx, &req->stopInfo);
}

bool  DStopRequest::positionReached(int q , int bias) const
{
	if (bias)
		return (::abs(q - this->stopInfo.endpos) < bias);
	else
		return q == this->stopInfo.endpos;
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
	if(req->dmc->m_rdWrManager.peekQueue(req->slave_idx))
	{//尚未发送完毕
		return;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return;
	}
		
	int posBias = (req->dmc->isServo(req->slave_idx)) ? (req->dmc->getServoPosBias()) : 0;

	if ( !req->positionReached(req->respData->Data1, posBias) 
		&& req->rechecks--)
	{
		return;
	}
		
	if (req->rechecks <= 0)						//等待位置到达超时
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("MoveRequest::fsm_state_csp timeout. axis=%d, nowpos=%d, dstpos=%d.", __FILE__, __LINE__,req->slave_idx, (int)req->respData->Data1, req->dstpos);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_csp reattempts.attempts=%d. axis=%d, nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->attempts, req->slave_idx, (int)req->respData->Data1, req->dstpos);
			req->cmdData->CMD   = CSP;
			req->cmdData->Data1 = req->dstpos;
			req->cmdData->Data2 = CSP_DATA2_DUMMY;	//特殊处理让充发位置可以进入待发送队列
			req->rechecks = RETRIES;
		}
		return;
	}

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx,req->dstpos);
	
	req->fsmstate	 	= fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
}

void MoveRequest::fsm_state_svon(MoveRequest *req)
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
			LOGSINGLE_ERROR("MoveRequest::fsm_state_svon timeouts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_svon reattempts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	if (!req->startPlan())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	req->fsmstate	 	= fsm_state_csp;
	req->cmdData->CMD 	= CSP;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	pushCspPoints(req);
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
			LOGSINGLE_ERROR("MoveRequest::fsm_state_sdowr_cspmode timeouts. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_sdowr_cspmode reattemps. axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
		pushCspPoints(req);
	}
	else
	{
		req->fsmstate		= MoveRequest::fsm_state_svon;
		req->cmdData->CMD		= SV_ON;
	}
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("MoveRequest::fsm_state_wait_sdowr_cspmode timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_wait_sdowr_cspmode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
}

void MoveRequest::fsm_state_start(MoveRequest *req)
{	
	req->startpos = req->dmc->getDriverCmdPos(req->slave_idx);//获得起始位置
	if (req->abs)
		req->dstpos = req->dist;
	else
		req->dstpos = req->startpos + req->dist;

	LOGSINGLE_INFORMATION("MoveRequest::fsm_state_start startpos = %d, dstpos=%d.", __FILE__, __LINE__, req->startpos, req->dstpos);

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
			pushCspPoints(req);
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
			LOGSINGLE_ERROR("Plan_T failed, axis=%d, q0=%f, q1=%f, vmax=%f, amax=%f.", __FILE__, __LINE__,
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
			LOGSINGLE_ERROR("Plan_S failed, axis=%d, q0=%f, q1=%f, vmax=%f, amax=%f, jmax=%f.", __FILE__, __LINE__,
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

void MoveRequest::pushCspPoints(MoveRequest *req)
{
	int cycles = req->moveparam->cycles;

	Item *items = new Item[cycles];
	for (int row = 0; row < cycles; ++row)
	{
		items[row].index 		= req->slave_idx;
		items[row].cmdData.CMD 	= CSP;
		items[row].cmdData.Data1=req->moveparam->position();
	}

	//将最后一个位置点进行存储
	req->cmdData->CMD 	= CSP;
	req->cmdData->Data1 	= items[cycles-1].cmdData.Data1; 

	req->dmc->logCspPoints(items, cycles, 1);	//输出规划结果到日志
	req->dmc->m_rdWrManager.pushItems(items, cycles, 1);
	delete [] items;
	
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
			LOGSINGLE_ERROR("ClrAlarmRequest::fsm_state_sdord_errcode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{			
			LOGSINGLE_INFORMATION("ClrAlarmRequest::fsm_state_sdord_errcode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("ClrAlarmRequest::fsm_state_wait_sdord_errcode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ClrAlarmRequest::fsm_state_wait_sdord_errcode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= ClrAlarmRequest::fsm_state_sdord_errcode;
	req->cmdData->CMD	= SDO_RD;
	req->cmdData->Parm	= 0x0200 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x603F0000;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("ClrAlarmRequest::fsm_state_alm_clr timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ClrAlarmRequest::fsm_state_wait_sdord_errcode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	}
	else
		req->fsmstate  = ClrAlarmRequest::fsm_state_wait_sdord_errcode;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("InitSlaveRequest::fsm_state_sdowr timeout, axis=%d, sdo=0x%?x.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("InitSlaveRequest::fsm_state_sdowr reattempts, axis=%d, sdo=0x%?x.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	LOGSINGLE_INFORMATION("InitSlaveRequest::fsm_state_sdowr succeed, axis=%d, sdo=0x%?x, value=%?d.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex), req->iter->sdo_value);

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
	}
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("InitSlaveRequest::fsm_state_wait_sdowr timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("InitSlaveRequest::fsm_state_wait_sdowr reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
	//shall not be called
}

void ServoOnRequest::fsm_state_svon(ServoOnRequest *req)
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
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_svon timeouts, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_svon reattempts, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
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
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_svoff timeout, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_svoff reattempts, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= ServoOnRequest::fsm_state_svon;
	req->cmdData->CMD	= SV_ON;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_sdowr_cspmode timeouts. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_sdowr_cspmode reattempts. axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_wait_sdowr_cspmode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_wait_sdowr_cspmode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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

void MultiAxisRequest::fsm_state_done(MultiAxisRequest *req)
{
	//shall not be called
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
			LOGSINGLE_ERROR("LineRequest::fsm_state_wait_all_pos_reached timeouts, axis=%d, svoncount(%d) != posreach(%d).", __FILE__, __LINE__,
						req->slave_idx, req->axispara->getSvonCount(), req->axispara->getPosReachedCount());
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("LineRequest::fsm_state_wait_all_pos_reached reattempts, axis=%d, svoncount(%d) != posreach(%d).", __FILE__, __LINE__, 
				req->slave_idx, req->axispara->getSvonCount(), req->axispara->getPosReachedCount());
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= MultiAxisRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
}

void  MultiAxisRequest::fsm_state_csp(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

	if(req->dmc->m_rdWrManager.peekQueue(req->slave_idx))
	{//尚未发送完毕
		return;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return;
	}

	int posBias = (req->dmc->isServo(req->slave_idx)) ? (req->dmc->getServoPosBias()) : 0;

	if ( !req->positionReached(req->respData->Data1, posBias) 
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)						//等待位置到达超时
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_csp, axis=%d, nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->slave_idx, (int)req->respData->Data1, req->axispara->dstpos);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_csp reattempts. attemps=%d, axis=%d nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->attempts, req->slave_idx, (int)req->respData->Data1, req->axispara->dstpos);
			req->cmdData->CMD = CSP;
			req->cmdData->Data1 = req->axispara->dstpos;
			req->cmdData->Data2 = CSP_DATA2_DUMMY;	//特殊处理让充发位置可以进入待发送队列
			req->rechecks = RETRIES;
		}
		return;
	}

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx, req->axispara->dstpos);

	req->axispara->reg_pos_reached();									//位置已经到达
	req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_pos_reached;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_wait_all_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_wait_all_svon reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
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
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	//多轴插入CSP规划点
	pushCspPoints(req);
}

void  MultiAxisRequest::fsm_state_svon(MultiAxisRequest *req)
{
	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->dmc->setMoveState(req->slave_idx, MOVESTATE_ERR);
		req->reqState = REQUEST_STATE_FAIL;
		return;
	}

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
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_svon timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_svon reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->axispara->reg_sv_on(req->slave_idx);
	req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_svon;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_sdowr_cspmode timeouts, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();		
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_sdowr_cspmode reattempts, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= MultiAxisRequest::fsm_state_svon;
	req->cmdData->CMD	= SV_ON;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_wait_sdowr_cspmode timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->axispara->setError();					
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_wait_sdowr_cspmode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
	if(req->dmc->isDriverOpCsp(req->slave_idx))
	{//已经处于CSP模式
		req->cmdData	= req->dmc->getCmdData(req->slave_idx);
		req->respData	= req->dmc->getRespData(req->slave_idx);
		if (req->dmc->isDriverOn(req->slave_idx))
		{//电机已经励磁
			req->axispara->reg_sv_on(req->slave_idx);
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

void MultiAxisRequest::pushCspPoints(MultiAxisRequest *req)
{
	if (req->slave_idx != req->axispara->ref->last_slaveidx)
		return;

	//最后一个从站将所有的规划点假如发送队列
	const std::set<BaseMultiAxisPara *> &paras = req->axispara->ref->paras;

	int cycles = req->axispara->totalCycles();
	size_t axises = paras.size();

	Item *items = new Item[cycles * axises];
	for (int row = 0; row < cycles; ++row)
	{
		int col = 0;
		for (std::set<BaseMultiAxisPara *>::const_iterator iter = paras.begin();
				iter != paras.end();
				++iter, ++col)
		{
			BaseMultiAxisPara *para = *iter;
			items[row * axises + col].index    		= para->req->slave_idx;							//从站地址
			items[row * axises + col].cmdData.CMD  	= CSP;
			items[row * axises + col].cmdData.Data1  = para->nextPosition(para->req->slave_idx);	//CSP目的位置
		}
	}

	req->dmc->logCspPoints(items, cycles, axises);	//输出规划结果到日志

	req->dmc->m_rdWrManager.pushItems(items, cycles, axises);

	delete [] items;
}

bool MultiAxisRequest::startPlan()
{
	return this->axispara->startPlan();
}

bool  MultiAxisRequest::positionReached(int q , int bias) const
{
	bool reached = this->axispara->positionReached(q, bias);
	return reached;
}

void MultiAxisRequest::exec()
{
	fsmstate(this);
}

MultiAxisRequest::MultiAxisRequest(int axis, LinearRef *newLinearRef, int pos, bool abs)
{
	int startpos, dstpos;

	startpos = DmcManager::instance().getDriverCmdPos(axis);
	if (abs)
		dstpos = pos;
	else
		dstpos = startpos + pos;
	
	this->slave_idx = axis;
	this->fsmstate = fsm_state_start;
	this->axispara = new LinearPara(newLinearRef, this, startpos, dstpos);

	double dist = (dstpos > startpos) ? (dstpos - startpos) : (startpos - dstpos);	
	if (dist > newLinearRef->max_dist)
		newLinearRef->max_dist = dist;				//参考轴运动距离
}

MultiAxisRequest::MultiAxisRequest(int axis, ArchlRef *newArchlRef, int pos, bool abs, bool z)			//Z轴拱门插补
{
	int startpos, dstpos;
	startpos = DmcManager::instance().getDriverCmdPos(axis);

	if (abs)
		dstpos = pos;
	else
		dstpos = startpos + pos;
	
	this->slave_idx = axis;
	this->fsmstate = fsm_state_start;
	this->axispara = new ArchlMultiAxisPara(newArchlRef, this, startpos, dstpos, z);

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
}

void HomeMoveRequest::fsm_state_done(HomeMoveRequest *req)
{
	//shall not be called
}

void HomeMoveRequest::fsm_state_aborthome(HomeMoveRequest *req)
{
	if(ABORT_HOME != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return;
	}

	long curpos =  req->dmc->getCurpos(req->slave_idx);
	req->dmc->setDriverCmdPos(req->slave_idx, curpos);			//回失败，将命令位置置为当前位置

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
		LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_gohome timeouts, axis=%d, status=0x%?x, curpos=%?d.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx), req->dmc->getCurpos(req->slave_idx));
		req->fsmstate		= HomeMoveRequest::fsm_state_aborthome;
		req->cmdData->CMD	= ABORT_HOME;
		req->rechecks		= RETRIES;
		return;
	}

	LOGSINGLE_INFORMATION("Homed, axis=%d.", __FILE__, __LINE__, req->slave_idx);
	req->dmc->setDriverCmdPos(req->slave_idx, 0);			//回原点后，将命令位置置为0

	req->fsmstate		= HomeMoveRequest::fsm_state_done;
	req->dmc->setMoveState(req->slave_idx, MOVESTATE_O_STOP);
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_acc timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_acc reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_lowspeed timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_lowspeed reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_highspeed timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_highspeed reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_homemethod timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_homemethod reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
}

void HomeMoveRequest::fsm_state_sdowr_homeoffset(HomeMoveRequest *req)
{
	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x607C0000 != req->respData->Data1 || 0 != req->respData->Data2)
		&& req->rechecks--)
	{
		return;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_homeoffset timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_homeoffset reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_homemode reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homeoffset;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x607C0000;						//index 0x607C subindex 0x0000
	req->cmdData->Data2 = 0; 								//原点偏移
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_wait_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_wait_sdowr_homemode reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->setMoveState(req->slave_idx, MOVESTATE_TIMEOUT);
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_svon reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
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
	req->attempts		= 0;
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
	//shall not be called
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
			LOGSINGLE_ERROR("ReadIoRequest::fsm_state_io_rd timeouts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->reqState = REQUEST_STATE_FAIL;
			req->dmc->setIoRS(req->slave_idx, IORS_TIMEOUT);
		}
		else
		{
			LOGSINGLE_INFORMATION("ReadIoRequest::fsm_state_io_rd reattempts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= ReadIoRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->dmc->setIoRS(req->slave_idx, IORS_SUCCESS);
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
	//shall not be called
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
			LOGSINGLE_ERROR("WriteIoRequest::fsm_state_io_wr timeouts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->reqState = REQUEST_STATE_FAIL;
			req->dmc->setIoRS(req->slave_idx, IORS_TIMEOUT);
		}
		else
		{
			LOGSINGLE_INFORMATION("WriteIoRequest::fsm_state_io_wr reattempts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return;
	}

	req->fsmstate		= WriteIoRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->dmc->setIoRS(req->slave_idx, IORS_SUCCESS);
	req->rechecks		= RETRIES;
	req->attempts		= 0;
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
