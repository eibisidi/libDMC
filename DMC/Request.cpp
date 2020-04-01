#include "Request.h"
#include "DmcManager.h"


#define RETRIES (10 * 50)
#define MAX_ATTEMPTS (5)				
#define MAKE_DWORD(h,l) ((h << 16) | (l))


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

FsmRetType DStopRequest::fsm_state_done(DStopRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType DStopRequest::fsm_state_svoff(DStopRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;
	if(SV_OFF != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (req->dmc->isDriverOn(req->slave_idx)
			&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{			
			LOGSINGLE_ERROR("DStopRequest::fsm_state_svoff timeouts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->reqState = REQUEST_STATE_FAIL;
			retval = MOVESTATE_TIMEOUT;
		} 
		else
		{
			LOGSINGLE_INFORMATION("DStopRequest::fsm_state_svoff reattempts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= DStopRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
	retval = MOVESTATE_CMD_STOP;
	return MOVESTATE_CMD_STOP;
}

FsmRetType DStopRequest::fsm_state_csp(DStopRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;
	if(req->dmc->m_rdWrManager.peekQueue(req->slave_idx))
	{//尚未发送完毕
		return retval;
	}

	//发送已经完成
	if(req->stopInfo.valid)
	{
		if(CSP != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData )
			&& req->rechecks--)
		{
			return retval;
		}

		int posBias = (req->dmc->isServo(req->slave_idx)) ? (req->dmc->getServoPosBias()) : 0;

		if ( !req->positionReached(req->respData->Data1, posBias) 
			&& req->rechecks--)
		{
			return retval;
		}

		if (req->rechecks <= 0)						//等待位置到达超时
		{
			if (++req->attempts > MAX_ATTEMPTS)
			{	
				LOGSINGLE_ERROR("DStopRequest::fsm_state_csp timeout. nowpos=%d, endpos=%d.", __FILE__, __LINE__, (int)req->respData->Data1, req->stopInfo.endpos);
				req->reqState = REQUEST_STATE_FAIL;
				retval = MOVESTATE_TIMEOUT;
			}
			else
			{
				LOGSINGLE_INFORMATION("DStopRequest::fsm_state_csp reattempts. attempts=%d. nowpos=%d, endpos=%d.", __FILE__, __LINE__, req->attempts, (int)req->respData->Data1, req->stopInfo.endpos);
				req->rechecks = RETRIES;
				req->cmdData->CMD 	= CSP;
				req->cmdData->Data1 = req->stopInfo.endpos;	//重发最终位置
			}
			return retval;
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
		retval = MOVESTATE_CMD_STOP;
		req->reqState		= REQUEST_STATE_SUCCESS;
	}
	req->rechecks		= RETRIES;
	req->attempts		= 0;
	return retval;
}

FsmRetType DStopRequest::fsm_state_start(DStopRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	req->cmdData	= req->dmc->getCmdData(req->slave_idx);
	req->respData	= req->dmc->getRespData(req->slave_idx);
	req->fsmstate	 	= fsm_state_csp;
	req->cmdData->CMD 	= CSP;
	
	//将减速命令加入到队列中
	req->dmc->m_rdWrManager.declStop(req->slave_idx, &req->stopInfo);

	return retval;
}

bool  DStopRequest::positionReached(int q , int bias) const
{
	if (bias)
		return (::abs(q - this->stopInfo.endpos) < bias);
	else
		return q == this->stopInfo.endpos;
}

FsmRetType DStopRequest::exec()
{
	return fsmstate(this);
}

FsmRetType MoveRequest::fsm_state_done(MoveRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType MoveRequest::fsm_state_csp(MoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(req->dmc->m_rdWrManager.peekQueue(req->slave_idx))
	{//尚未发送完毕
		return retval;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return retval;
	}
		
	int posBias = (req->dmc->isServo(req->slave_idx)) ? (req->dmc->getServoPosBias()) : 0;

	if ( !req->positionReached(req->respData->Data1, posBias) 
		&& req->rechecks--)
	{
		return retval;
	}
		
	if (req->rechecks <= 0)						//等待位置到达超时
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("MoveRequest::fsm_state_csp timeout. axis=%d, nowpos=%d, dstpos=%d.", __FILE__, __LINE__,req->slave_idx, (int)req->respData->Data1, req->dstpos);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_csp reattempts.attempts=%d. axis=%d, nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->attempts, req->slave_idx, (int)req->respData->Data1, req->dstpos);
			req->cmdData->CMD   = CSP;
			req->cmdData->Data1 = req->dstpos;
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx,req->dstpos);
	
	req->fsmstate	 	= fsm_state_done;
	retval 				= MOVESTATE_STOP;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType MoveRequest::fsm_state_svon(MoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SV_ON != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return retval;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("MoveRequest::fsm_state_svon timeouts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_svon reattempts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	if (!req->startPlan())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	req->fsmstate	 	= fsm_state_csp;
	req->cmdData->CMD 	= GET_STATUS;		/*非同步命令*/
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	pushCspPoints(req);

	return retval;
}

FsmRetType MoveRequest::fsm_state_sdowr_cspmode(MoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || CSP_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("MoveRequest::fsm_state_sdowr_cspmode timeouts. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_sdowr_cspmode reattemps. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);

	if (req->dmc->isDriverOn(req->slave_idx))
	{//电机已经励磁
		if (!req->startPlan())
		{
			req->reqState = REQUEST_STATE_FAIL;
			return MOVESTATE_ERR;
		}
		req->fsmstate	 	= MoveRequest::fsm_state_csp;
		req->cmdData->CMD 	= GET_STATUS;	/*非同步命令*/
		pushCspPoints(req);
	}
	else
	{
		req->fsmstate		= MoveRequest::fsm_state_svon;
		req->cmdData->CMD		= SV_ON;
	}
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType MoveRequest::fsm_state_wait_sdowr_cspmode(MoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("MoveRequest::fsm_state_wait_sdowr_cspmode timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MoveRequest::fsm_state_wait_sdowr_cspmode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= MoveRequest::fsm_state_sdowr_cspmode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = CSP_MODE;	
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType MoveRequest::fsm_state_start(MoveRequest *req)
{	
	FsmRetType retval = MOVESTATE_BUSY;

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
				req->reqState = REQUEST_STATE_FAIL;
				return MOVESTATE_ERR;
			}
		
			req->fsmstate	 	= MoveRequest::fsm_state_csp;
			req->cmdData->CMD 	= GET_STATUS;/*非同步命令*/
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

	return retval;
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

	Item *head = NULL;
	Item *tail = NULL;
	Item *newItem = NULL;

	for(int row = 0; row < cycles; ++row)
	{
		newItem = new Item;
		newItem->index 			= req->slave_idx;
		newItem->cmdData.CMD	= CSP;
		newItem->cmdData.Data1	= req->moveparam->position();
		newItem->cmdData.Data2	= 0;
		
		if (0 == row)
		{
			tail = head = newItem;
		}
		else
		{
			tail->next 		= newItem;
			newItem->prev 	= tail;
			newItem->next 	= head;
			head->prev		= newItem;
		}
		tail = newItem;
	}

	req->cmdData->CMD	= GET_STATUS;

	//req->dmc->logCspPoints(items, cycles, 1);	//todo输出规划结果到日志
	req->dmc->pushItems(&head, cycles, 1, false);
}

FsmRetType MoveRequest::exec()
{
	return fsmstate(this);
}

FsmRetType ClrAlarmRequest::fsm_state_done(ClrAlarmRequest *req)
{
	return MOVESTATE_NONE;
}

FsmRetType ClrAlarmRequest::fsm_state_sdord_errcode(ClrAlarmRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_RD != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x603F0000 != req->respData->Data1 || 0x00 != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("ClrAlarmRequest::fsm_state_sdord_errcode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{			
			LOGSINGLE_INFORMATION("ClrAlarmRequest::fsm_state_sdord_errcode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= ClrAlarmRequest::fsm_state_done;
	retval				= MOVESTATE_STOP; 
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType ClrAlarmRequest::fsm_state_wait_sdord_errcode(ClrAlarmRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("ClrAlarmRequest::fsm_state_wait_sdord_errcode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ClrAlarmRequest::fsm_state_wait_sdord_errcode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= ClrAlarmRequest::fsm_state_sdord_errcode;
	req->cmdData->CMD	= SDO_RD;
	req->cmdData->Parm	= 0x0200 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x603F0000;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType ClrAlarmRequest::fsm_state_alm_clr(ClrAlarmRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(ALM_CLR != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("ClrAlarmRequest::fsm_state_alm_clr timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ClrAlarmRequest::fsm_state_wait_sdord_errcode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
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

	return retval;
}

FsmRetType ClrAlarmRequest::fsm_state_start(ClrAlarmRequest *req)
{	
	FsmRetType retval = MOVESTATE_BUSY;

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= ClrAlarmRequest::fsm_state_wait_sdord_errcode;
	req->cmdData->CMD	= ALM_CLR;

	return retval;
}

FsmRetType ClrAlarmRequest::exec()
{
	return fsmstate(this);
}

FsmRetType InitSlaveRequest::fsm_state_done(InitSlaveRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType InitSlaveRequest::fsm_state_sdowr(InitSlaveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  MAKE_DWORD(req->iter->sdo_index,req->iter->sdo_subindex) != req->respData->Data1 || req->iter->sdo_value != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{	
			LOGSINGLE_ERROR("InitSlaveRequest::fsm_state_sdowr timeout, axis=%d, sdo=0x%?x.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex));
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("InitSlaveRequest::fsm_state_sdowr reattempts, axis=%d, sdo=0x%?x.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	LOGSINGLE_INFORMATION("InitSlaveRequest::fsm_state_sdowr succeed, axis=%d, sdo=0x%?x, value=%?d.", __FILE__, __LINE__, req->slave_idx, MAKE_DWORD(req->iter->sdo_index, req->iter->sdo_subindex), req->iter->sdo_value);

	if (++req->iter == req->iterEnd)
	{
		//free sdo
		req->dmc->freeSdoCmdResp(req);
		req->cmdData		= req->dmc->getCmdData(req->slave_idx);
		req->respData		= req->dmc->getRespData(req->slave_idx);
		req->fsmstate		= InitSlaveRequest::fsm_state_done;
		retval = MOVESTATE_STOP;
		req->reqState		= REQUEST_STATE_SUCCESS;
	}
	else
	{
		req->fsmstate		= InitSlaveRequest::fsm_state_sdowr;
		req->cmdData->CMD	= SDO_WR;
		req->cmdData->Parm	= (req->iter->sdo_size << 8) | (req->slave_idx & 0xFF); 			//size | slaveidx
		req->cmdData->Data1 = MAKE_DWORD(req->iter->sdo_index,req->iter->sdo_subindex); 	
		req->cmdData->Data2 = (req->iter->sdo_value);										
	}
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType InitSlaveRequest::fsm_state_wait_sdowr(InitSlaveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{		
			LOGSINGLE_ERROR("InitSlaveRequest::fsm_state_wait_sdowr timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("InitSlaveRequest::fsm_state_wait_sdowr reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= InitSlaveRequest::fsm_state_sdowr;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= (req->iter->sdo_size << 8) | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = MAKE_DWORD(req->iter->sdo_index,req->iter->sdo_subindex);
	req->cmdData->Data2 = (req->iter->sdo_value);
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType InitSlaveRequest::fsm_state_start(InitSlaveRequest *req)
{	
	FsmRetType retval = MOVESTATE_BUSY;

	if (req->iter == req->iterEnd)
	{
		req->fsmstate		= InitSlaveRequest::fsm_state_done;
		retval 				= MOVESTATE_STOP;
		req->reqState		= REQUEST_STATE_SUCCESS;
		return retval;
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

	return retval;
}

FsmRetType InitSlaveRequest::exec()
{
	return fsmstate(this);
}

FsmRetType ServoOnRequest::fsm_state_done(ServoOnRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType ServoOnRequest::fsm_state_svon(ServoOnRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SV_ON != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return retval;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_svon timeouts, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			retval 			= MOVESTATE_TIMEOUT;
			req->reqState 	= REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_svon reattempts, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//初始化电机位置
	req->dmc->setDriverCmdPos(req->slave_idx, (long)req->respData->Data1);

	req->fsmstate	 	= ServoOnRequest::fsm_state_done;
	retval 				= MOVESTATE_STOP;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType ServoOnRequest::fsm_state_svoff(ServoOnRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SV_OFF != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (!req->dmc->isDriverOff(req->slave_idx)
		&& req->rechecks--)
	{
		return retval;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_svoff timeout, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			retval 			= MOVESTATE_TIMEOUT;
			req->reqState 	= REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_svoff reattempts, axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= ServoOnRequest::fsm_state_svon;
	req->cmdData->CMD	= SV_ON;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType ServoOnRequest::fsm_state_sdowr_cspmode(ServoOnRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || CSP_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_sdowr_cspmode timeouts. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval 			= MOVESTATE_TIMEOUT;
			req->reqState 	= REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_sdowr_cspmode reattempts. axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= ServoOnRequest::fsm_state_svoff;
	req->cmdData->CMD	= SV_OFF;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType ServoOnRequest::fsm_state_wait_sdowr_cspmode(ServoOnRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("ServoOnRequest::fsm_state_wait_sdowr_cspmode timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("ServoOnRequest::fsm_state_wait_sdowr_cspmode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= ServoOnRequest::fsm_state_sdowr_cspmode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = CSP_MODE;	
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType ServoOnRequest::fsm_state_start(ServoOnRequest *req)
{	
	FsmRetType retval = MOVESTATE_BUSY;

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

	return retval;
}

FsmRetType ServoOnRequest::exec()
{
	return fsmstate(this);
}

FsmRetType MultiAxisRequest::fsm_state_done(MultiAxisRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType MultiAxisRequest::fsm_state_wait_all_sent(MultiAxisRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;
	
	if(0 != req->axispara->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(req->dmc->m_rdWrManager.peekQueue(req->slave_idx))
	{//尚未发送完毕
		return retval;
	}
	
	req->fsmstate		= MultiAxisRequest::fsm_state_done;
	retval				= MOVESTATE_RUNNING;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiAxisRequest::fsm_state_wait_all_pos_reached(MultiAxisRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->axispara->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if ( !req->axispara->pos_allreached() 
		&& req->rechecks--)
	{
		return retval;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("LineRequest::fsm_state_wait_all_pos_reached timeouts, axis=%d, svoncount(%d) != posreach(%d).", __FILE__, __LINE__,
						req->slave_idx, req->axispara->getSvonCount(), req->axispara->getPosReachedCount());
			retval = MOVESTATE_TIMEOUT;
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("LineRequest::fsm_state_wait_all_pos_reached reattempts, axis=%d, svoncount(%d) != posreach(%d).", __FILE__, __LINE__, 
				req->slave_idx, req->axispara->getSvonCount(), req->axispara->getPosReachedCount());
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= MultiAxisRequest::fsm_state_done;
	retval 				= MOVESTATE_STOP;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiAxisRequest::fsm_state_csp(MultiAxisRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->axispara->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(req->dmc->m_rdWrManager.peekQueue(req->slave_idx))
	{//尚未发送完毕
		return retval;
	}

	if(CSP != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return retval;
	}

	int posBias = (req->dmc->isServo(req->slave_idx)) ? (req->dmc->getServoPosBias()) : 0;

	if ( !req->positionReached(req->respData->Data1, posBias) 
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)						//等待位置到达超时
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_csp, axis=%d, nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->slave_idx, (int)req->respData->Data1, req->axispara->dstpos);
			retval = MOVESTATE_TIMEOUT;
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_csp reattempts. attempts=%d, axis=%d nowpos=%d, dstpos=%d.", __FILE__, __LINE__, req->attempts, req->slave_idx, (int)req->respData->Data1, req->axispara->dstpos);
			req->cmdData->CMD = CSP;
			req->cmdData->Data1 = req->axispara->dstpos;
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//目标位置已到达,更新新的绝对位置
	req->dmc->setDriverCmdPos(req->slave_idx, req->axispara->dstpos);

	req->axispara->reg_pos_reached();									//位置已经到达
	req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_pos_reached;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiAxisRequest::fsm_state_wait_all_svon(MultiAxisRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->axispara->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if (!req->axispara->sv_allon()
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_wait_all_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_wait_all_svon reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//所有电机均已经励磁
	if (!req->startPlan())
	{//规划失败
		req->axispara->setError();
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	//多轴插入CSP规划点
	pushCspPoints(req);

	if (req->keep)
	{
		req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_sent;
	}
	else
	{
		req->fsmstate	 	= MultiAxisRequest::fsm_state_csp;
	}	
	req->cmdData->CMD	= GET_STATUS;							/*同步命令*/
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiAxisRequest::fsm_state_svon(MultiAxisRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(SV_ON != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return retval;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_svon timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->axispara->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_svon reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->axispara->reg_sv_on(req->slave_idx);
	req->fsmstate		= MultiAxisRequest::fsm_state_wait_all_svon;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiAxisRequest::fsm_state_sdowr_cspmode(MultiAxisRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || CSP_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_sdowr_cspmode timeouts, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->axispara->setError();		
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_sdowr_cspmode reattempts, axis=%d\n", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate		= MultiAxisRequest::fsm_state_svon;
	req->cmdData->CMD	= SV_ON;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiAxisRequest::fsm_state_wait_sdowr_cspmode(MultiAxisRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiAxisRequest::fsm_state_wait_sdowr_cspmode timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->axispara->setError();					
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAxisRequest::fsm_state_wait_sdowr_cspmode reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= MultiAxisRequest::fsm_state_sdowr_cspmode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = CSP_MODE; 
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiAxisRequest::fsm_state_start(MultiAxisRequest *req)
{	
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->axispara->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
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

	return retval;
}

void MultiAxisRequest::pushCspPoints(MultiAxisRequest *req)
{
	if (req->slave_idx != req->axispara->ref->last_slaveidx)
		return;

	//最后一个从站将所有的规划点加入发送队列
	const std::map<int, BaseMultiAxisPara *> &paras = req->axispara->ref->paras;

	int cycles = req->axispara->totalCycles();
	size_t axises = paras.size();

	Item **itemLists = new Item *[axises];
	for (int row = 0; row < cycles; ++row)
	{
		int col = 0;
		for (std::map<int, BaseMultiAxisPara *>::const_iterator iter = paras.begin();
				iter != paras.end();
				++iter, ++col)
		{
			BaseMultiAxisPara *para = iter->second;

			Item *newItem = new Item;
			newItem->index			= para->req->slave_idx; 						//从站地址
			newItem->cmdData.CMD	= CSP;
			newItem->cmdData.Data1	= para->nextPosition(para->req->slave_idx);		//CSP目的位置
			newItem->cmdData.Data2	= 0;
			
			if (0 == row)
			{
				itemLists[col] = newItem;
			}
			else
			{
				Item * tail = itemLists[col]->prev;
				Item * head = itemLists[col];

				tail->next 		= newItem;
				newItem->prev	= tail;
				newItem->next	= head;
				head->prev		= newItem;
			}
		}
	}

	//req->dmc->logCspPoints(itemLists, cycles, axises);	//输出规划结果到日志

	req->dmc->pushItems(itemLists, cycles, axises, true/*同步*/, req->keep/*持续匀速运动*/);

	delete [] itemLists;
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

FsmRetType MultiAxisRequest::exec()
{
	return fsmstate(this);
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
	this->axispara = new LinearPara(newLinearRef, this, axis, startpos, dstpos);

	double dist = (dstpos > startpos) ? (dstpos - startpos) : (startpos - dstpos);	
	if (dist > newLinearRef->max_dist)
		newLinearRef->max_dist = dist;				//参考轴运动距离

	this->keep = false;
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
	this->axispara = new ArchlMultiAxisPara(newArchlRef, this, axis, startpos, dstpos, z);

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

	this->keep = false;
}

MultiAxisRequest::MultiAxisRequest(int axis, AccRef *newAccRef, long maxvel)
{
	int startpos;
	startpos = DmcManager::instance().getDriverCmdPos(axis);
	
	this->slave_idx = axis;
	this->fsmstate 	= fsm_state_start;
	this->axispara	= new AccMultiAxisPara(newAccRef, this, axis, maxvel, startpos);

	this->keep = true;
}

FsmRetType HomeMoveRequest::fsm_state_done(HomeMoveRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType HomeMoveRequest::fsm_state_aborthome(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(ABORT_HOME != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	long curpos =  req->dmc->getCurpos(req->slave_idx);
	req->dmc->setDriverCmdPos(req->slave_idx, curpos);			//回失败，将命令位置置为当前位置

	retval =  MOVESTATE_TIMEOUT;
	req->reqState = REQUEST_STATE_FAIL;
	
	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_gohome(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(GO_HOME != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval; 
	}

	bool timeout = req->homeTimeout();				//当前是否超时
		
	if (!req->dmc->isDriverHomed(req->slave_idx)
		&& !timeout)		
	{
		return retval;
	}

	if (req->rechecks <= 0
		|| timeout)
	{
		LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_gohome timeouts, axis=%d, status=0x%?x, curpos=%?d.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx), req->dmc->getCurpos(req->slave_idx));
		req->fsmstate		= HomeMoveRequest::fsm_state_aborthome;
		req->cmdData->CMD	= ABORT_HOME;
		req->rechecks		= RETRIES;
		return retval;
	}

	LOGSINGLE_INFORMATION("Homed, axis=%d.", __FILE__, __LINE__, req->slave_idx);
	req->dmc->setDriverCmdPos(req->slave_idx, 0);			//回原点后，将命令位置置为0

	req->fsmstate		= HomeMoveRequest::fsm_state_done;
	retval				= MOVESTATE_O_STOP;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_sdowr_acc(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm || 0x609A0000 != req->respData->Data1 ||  req->acc != req->respData->Data2)
		&& req->rechecks-- )
	{
		return retval;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_acc timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_acc reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->fsmstate		= HomeMoveRequest::fsm_state_gohome;
	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->cmdData->CMD	= GO_HOME;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_sdowr_lowspeed(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm || 0x60990002 != req->respData->Data1 ||  req->low_speed != req->respData->Data2)
		&& req->rechecks-- )
	{
		return retval;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_lowspeed timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_lowspeed reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_acc;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x609A0000;						//index 0x609A subindex 0x0000
	req->cmdData->Data2 = req->acc; 						//加速度		
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_sdowr_highspeed(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm || 0x60990001 != req->respData->Data1 ||  req->high_speed != req->respData->Data2)
		&& req->rechecks-- )
	{
		return retval;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_highspeed timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_highspeed reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_lowspeed;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x60990002;						//index 0x6099 subindex 0x0002
	req->cmdData->Data2 = req->low_speed; 					//低速				
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_sdowr_homemethod(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm || 0x60980000 != req->respData->Data1 ||  req->home_method != req->respData->Data2)
		&& req->rechecks-- )
	{
		return retval;
	}
		
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_homemethod timeouts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_homemethod reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_highspeed;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x60990001;						//index 0x6099 subindex 0x0001
	req->cmdData->Data2 = req->high_speed; 					//高速		
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_sdowr_homeoffset(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x607C0000 != req->respData->Data1 || 0 != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_homeoffset timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_homeoffset reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homemethod;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x60980000;						//index 0x6098 subindex 0x0000
	req->cmdData->Data2 = req->home_method; 				//回原点方式
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_sdowr_homemode(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || HOMING_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_homemode reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homeoffset;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm	= 0x0400 | (req->slave_idx & 0xFF); //size | slaveidx
	req->cmdData->Data1 = 0x607C0000;						//index 0x607C subindex 0x0000
	req->cmdData->Data2 = 0; 								//原点偏移
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_wait_sdowr_homemode(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_wait_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval		= MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_wait_sdowr_homemode reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= HomeMoveRequest::fsm_state_sdowr_homemode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm  = 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = HOMING_MODE;					
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_svon(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(SV_ON != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return retval;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_svon reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
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

	return retval;
}

FsmRetType HomeMoveRequest::fsm_state_start(HomeMoveRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

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

	return retval;
}

bool HomeMoveRequest::homeTimeout() const
{
	return (starttime.elapsed() > (home_timeout * 1000000));
}

FsmRetType HomeMoveRequest::exec()
{
	return fsmstate(this);
}

FsmRetType MultiHomeRequest::fsm_state_done(MultiHomeRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType MultiHomeRequest::fsm_state_gohome(MultiHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(GO_HOME != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval; 
	}

	bool timeout = req->homeTimeout();				//当前是否超时
		
	if (!req->dmc->isDriverHomed(req->slave_idx)
		&& !timeout)		
	{
		return retval;
	}

	if (req->rechecks <= 0
		|| timeout)
	{
		LOGSINGLE_ERROR("MultiHomeRequest::fsm_state_gohome timeouts, axis=%d, status=0x%?x, curpos=%?d.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx), req->dmc->getCurpos(req->slave_idx));
		retval = MOVESTATE_TIMEOUT;
		req->ref->setError();
		req->reqState = REQUEST_STATE_FAIL;
		return retval;
	}

	LOGSINGLE_INFORMATION("Homed, axis=%d.", __FILE__, __LINE__, req->slave_idx);
	req->dmc->setDriverCmdPos(req->slave_idx, 0);			//回原点后，将命令位置置为0

	req->fsmstate		= MultiHomeRequest::fsm_state_done;
	retval				= MOVESTATE_O_STOP;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiHomeRequest::fsm_state_wait_all_sync(MultiHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if (!req->ref->all_sync()
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiHomeRequest::fsm_state_wait_all_svon timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiHomeRequest::fsm_state_wait_all_svon reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= MultiHomeRequest::fsm_state_gohome;
	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->cmdData->CMD	= GET_STATUS;							/*同步命令*/
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	pushMultiHome(req);

	return retval;
}

FsmRetType MultiHomeRequest::fsm_state_wait_1s(MultiHomeRequest *req)
{//等待0x6060运动模式修改生效

	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if (req->starttime.elapsed() < 1000000)
	{
		return retval;
	}

	req->ref->reg_sync(req->slave_idx);
	req->fsmstate		= MultiHomeRequest::fsm_state_wait_all_sync;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
	return retval;
}

FsmRetType MultiHomeRequest::fsm_state_sdowr_homemode(MultiHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(SDO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
		&& req->rechecks--)
	{
		return retval;
	}

	if ((req->slave_idx != req->respData->Parm ||  0x60600000 != req->respData->Data1 || HOMING_MODE != req->respData->Data2)
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("HomeMoveRequest::fsm_state_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("HomeMoveRequest::fsm_state_sdowr_homemode reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	//free sdo
	req->dmc->freeSdoCmdResp(req);

	req->starttime.update();									//刷新时间
	req->fsmstate		= MultiHomeRequest::fsm_state_wait_1s;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType MultiHomeRequest::fsm_state_wait_sdowr_homemode(MultiHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if (!req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData)
		&& req->rechecks --)
	{
		return retval;
	}

	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiHomeRequest::fsm_state_wait_sdowr_homemode timeouts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			retval		= MOVESTATE_TIMEOUT;
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiHomeRequest::fsm_state_wait_sdowr_homemode reattempts, axis = %d.", __FILE__, __LINE__, req->slave_idx);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= MultiHomeRequest::fsm_state_sdowr_homemode;
	req->cmdData->CMD	= SDO_WR;
	req->cmdData->Parm  = 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
	req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
	req->cmdData->Data2 = HOMING_MODE;					
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType  MultiHomeRequest::fsm_state_svon(MultiHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(SV_ON != RESP_CMD_CODE(req->respData)
			&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (!req->dmc->isDriverOn(req->slave_idx)
		&& req->rechecks--)
	{
		return retval;
	}
			
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiHomeRequest::fsm_state_svon timeout, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiHomeRequest::fsm_state_svon reattempts, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
	{
		req->fsmstate		= MultiHomeRequest::fsm_state_sdowr_homemode;
		req->cmdData->CMD	= SDO_WR;
		req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
		req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
		req->cmdData->Data2 = HOMING_MODE;						
	}
	else
		req->fsmstate		= MultiHomeRequest::fsm_state_wait_sdowr_homemode;		
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType MultiHomeRequest::fsm_state_start(MultiHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;
	
	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if (req->dmc->isDriverOn(req->slave_idx))
	{//电机已经励磁
		if (req->dmc->getSdoCmdResp(req, &req->cmdData, &req->respData))
		{
			req->fsmstate		= MultiHomeRequest::fsm_state_sdowr_homemode;
			req->cmdData->CMD	= SDO_WR;
			req->cmdData->Parm	= 0x0100 | (req->slave_idx & 0xFF); 			//size | slaveidx
			req->cmdData->Data1 = 0x60600000;									//index 0x6060 subindex 0x0000
			req->cmdData->Data2 = HOMING_MODE;						
		}
		else
			req->fsmstate		= MultiHomeRequest::fsm_state_wait_sdowr_homemode;
	}
	else
	{
		req->fsmstate		= MultiHomeRequest::fsm_state_svon;
		req->cmdData		= req->dmc->getCmdData(req->slave_idx);
		req->respData		= req->dmc->getRespData(req->slave_idx);
		req->cmdData->CMD		= SV_ON;
	}

	return retval;
}


bool MultiHomeRequest::homeTimeout() const
{
	return (starttime.elapsed() > (home_timeout * 1000000));
}

void MultiHomeRequest::pushMultiHome(MultiHomeRequest *req)
{
	if (!req->ref->isLast(req->slave_idx))
		return;

	const std::set<int>& axises = req->ref->getAxises();
	size_t count = axises.size();

	Item **itemLists = new Item*[count];

	int col = 0;
	for (std::set<int>::const_iterator iter = axises.begin();
			iter != axises.end();
			++iter, ++col)
	{
		Item * newItem = new Item;
		newItem->index    		= *iter;							//从站地址
		newItem->cmdData.CMD  	= GO_HOME;
		newItem->cmdData.Data1 	= 0;
		newItem->cmdData.Data2 	= 0;

		itemLists[col] = newItem;
	}

	req->dmc->pushItems(itemLists, 1, count, true);

	req->ref->setStarted();//已经开始回零

	delete [] itemLists;
}

FsmRetType MultiHomeRequest::exec()
{
	return fsmstate(this);
}

MultiHomeRequest::MultiHomeRequest(int axis, MultiHomeRef *newRef, int to)
{
	this->slave_idx = axis;
	this->fsmstate = fsm_state_start;
	this->ref = newRef;
	this->ref->duplicate();
	this->home_timeout = to;
}

FsmRetType MultiAbortHomeRequest::fsm_state_done(MultiAbortHomeRequest *req)
{
	//shall not be called
	return MOVESTATE_NONE;
}

FsmRetType MultiAbortHomeRequest::fsm_state_svoff(MultiAbortHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}
		
	if(SV_OFF != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}

	if (req->dmc->isDriverOn(req->slave_idx)
			&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{			
			LOGSINGLE_ERROR("MultiAbortHomeRequest::fsm_state_svoff timeouts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
			retval = MOVESTATE_TIMEOUT;
		} 
		else
		{
			LOGSINGLE_INFORMATION("MultiAbortHomeRequest::fsm_state_svoff reattempts. axis=%d, status=0x%?x.", __FILE__, __LINE__, req->slave_idx, req->dmc->getDriverStatus(req->slave_idx));
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= MultiAbortHomeRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;
	retval = MOVESTATE_CMD_STOP;
	return MOVESTATE_CMD_STOP;
}

FsmRetType MultiAbortHomeRequest::fsm_state_aborthome(MultiAbortHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;

	if(0 != req->ref->getError())
	{
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if(ABORT_HOME != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData )
		&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0) 
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("MultiAbortHomeRequest::fsm_state_aborthome, axis=%d.", __FILE__, __LINE__, req->slave_idx);
			retval = MOVESTATE_TIMEOUT;
			req->ref->setError();
			req->reqState = REQUEST_STATE_FAIL;
		}
		else
		{
			LOGSINGLE_INFORMATION("MultiAbortHomeRequest::fsm_state_aborthome reattempts. attemps=%d, axis=%d.", __FILE__, __LINE__, req->attempts, req->slave_idx);
			req->cmdData->CMD = ABORT_HOME;
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= MultiAbortHomeRequest::fsm_state_svoff;
	req->cmdData->CMD	= SV_OFF;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType MultiAbortHomeRequest::fsm_state_start(MultiAbortHomeRequest *req)
{
	FsmRetType retval = MOVESTATE_BUSY;
	
	if(0 != req->ref->getError())
	{//其它电机已经出现错误
		req->reqState = REQUEST_STATE_FAIL;
		return MOVESTATE_ERR;
	}

	if (req->ref->isFirst(req->slave_idx))
	{
		pushMultiAbortHome(req);
	}

	req->fsmstate		= MultiAbortHomeRequest::fsm_state_aborthome;
	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->cmdData->CMD	= GET_STATUS;							/*同步命令*/

	return retval;
}

void MultiAbortHomeRequest::pushMultiAbortHome(MultiAbortHomeRequest *req)
{
	if (!req->ref->isFirst(req->slave_idx))
		return;

	const std::set<int>& axises = req->ref->getAxises();
	size_t count = axises.size();

	Item **itemLists = new Item*[count];

	int col = 0;
	for (std::set<int>::const_iterator iter = axises.begin();
			iter != axises.end();
			++iter, ++col)
	{

		Item * newItem = new Item;
		newItem->index			= *iter;							//从站地址
		newItem->cmdData.CMD	= ABORT_HOME;
		newItem->cmdData.Data1	= 0;
		newItem->cmdData.Data2	= 0;

		itemLists[col] = newItem;
	}

	req->dmc->pushItems(itemLists, 1, count, true);

	delete [] itemLists;
}

FsmRetType MultiAbortHomeRequest::exec()
{
	return fsmstate(this);
}

MultiAbortHomeRequest::MultiAbortHomeRequest(int axis, MultiAbortHomeRef *newRef)
{
	this->slave_idx = axis;
	this->fsmstate	= fsm_state_start;
	this->ref 		= newRef;
	this->ref->duplicate(axis);
}

FsmRetType ReadIoRequest::fsm_state_done(ReadIoRequest *req)
{
	//shall not be called
	return IORS_NONE;
}

FsmRetType ReadIoRequest::fsm_state_io_rd(ReadIoRequest *req)
{
	FsmRetType retval = IORS_BUSY;

	if(IO_RD != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{		
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("ReadIoRequest::fsm_state_io_rd timeouts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->reqState = REQUEST_STATE_FAIL;
			retval		  = IORS_TIMEOUT;
		}
		else
		{
			LOGSINGLE_INFORMATION("ReadIoRequest::fsm_state_io_rd reattempts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= ReadIoRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	retval				= IORS_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType ReadIoRequest::fsm_state_start(ReadIoRequest *req)
{
	FsmRetType retval = IORS_BUSY;

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate 		= ReadIoRequest::fsm_state_io_rd;
	req->cmdData->CMD 	= IO_RD;

	return retval;
}

FsmRetType ReadIoRequest::exec()
{
	return fsmstate(this);
}

FsmRetType WriteIoRequest::fsm_state_done(WriteIoRequest *req)
{
	//shall not be called
	return IORS_NONE;
}

FsmRetType WriteIoRequest::fsm_state_io_wr(WriteIoRequest *req)
{
	FsmRetType retval = IORS_BUSY;

	if(IO_WR != RESP_CMD_CODE(req->respData)
		&& GET_STATUS != RESP_CMD_CODE(req->respData)
			&& req->rechecks--)
	{
		return retval;
	}
	
	if (req->rechecks <= 0)
	{
		if (++req->attempts > MAX_ATTEMPTS)
		{
			LOGSINGLE_ERROR("WriteIoRequest::fsm_state_io_wr timeouts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->reqState = REQUEST_STATE_FAIL;
			retval 			= IORS_TIMEOUT;
		}
		else
		{
			LOGSINGLE_INFORMATION("WriteIoRequest::fsm_state_io_wr reattempts, slave = %d.", __FILE__, __LINE__, req->slave_idx);
			req->dmc->restoreLastCmd(req->cmdData);
			req->rechecks = RETRIES;
		}
		return retval;
	}

	req->fsmstate		= WriteIoRequest::fsm_state_done;
	req->reqState		= REQUEST_STATE_SUCCESS;
	req->dmc->setIoOutput(req->slave_idx, req->output);	
	retval				= IORS_SUCCESS;
	req->rechecks		= RETRIES;
	req->attempts		= 0;

	return retval;
}

FsmRetType WriteIoRequest::fsm_state_start(WriteIoRequest *req)
{
	FsmRetType retval = IORS_BUSY;

	req->cmdData		= req->dmc->getCmdData(req->slave_idx);
	req->respData		= req->dmc->getRespData(req->slave_idx);
	req->fsmstate 		= WriteIoRequest::fsm_state_io_wr;
	req->cmdData->CMD 	= IO_WR;
	req->cmdData->Data1 = req->output;

	return retval;
}

FsmRetType WriteIoRequest::exec()
{
	return fsmstate(this);
}
