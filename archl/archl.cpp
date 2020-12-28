#include "AxisPara.h"
#include <fstream>
/*
功 能：多轴绝对坐标的Z轴拱门运动。
输入参 数： TotalAxis： 插补轴数 >=1
		*startpos   起始点位置, startpos[0]为起始Z轴点位
		*endpos		终止点位置, endpos[0]为终止Z轴点位
		raxis		R轴索引， 如果 (r==-1)代表不存在R轴
		zmaxvel 	Z轴最大行速度，单位： pps
		maxvel		水平方向轴最大运行速度，单位： pps
		ztacc		Z轴加减速时间，单位:s
		Tacc： 		水平方向轴加速时间，单位：s。
		Tdec:  		水平方向轴减速时间，单位：s
		hh:			最高绝对高度
		hu:			相对上升高度，相对于起始点 >= 0
		hd:			相对下降高度，相对于终止点 >= 0
输出参数：	 ppResult 	规划结果，行数为关键点个数，列数为轴数
		  pCycles:  关键点个数
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
int archl(short totalAxis, const long *beginPos, const long *endPos, int raxis, long zmaxvel,long maxvel, 
				double ztacc, double Tacc, double Tdec,
				long hh, long hu, long hd, 
				long **ppResult, double *pTime, int *pCycles)
{
	if (totalAxis <= 0
			|| Tacc < 1E-6
			|| maxvel < 1E-6
			|| hu < 0
			|| hd < 0
			|| NULL == ppResult 
			|| NULL == pTime
			|| NULL == pCycles
			|| raxis >= totalAxis
			|| raxis == 0)
			return -1;
	
	int count =0;
	for ( ;count < totalAxis;++count)
	{
		if (beginPos[count] != endPos[count])
			break;
	}

	if (count == totalAxis)
		return 0;
			
	unsigned long	retValue = 0;
	ArchlRef		 *newArchlRef = NULL;

	newArchlRef = new ArchlRef;
	if(NULL == newArchlRef)
	{
		retValue = -1;
		goto DONE;
	}

	newArchlRef->maxvel = maxvel;
	newArchlRef->zmaxvel = zmaxvel;
	newArchlRef->maxa	= maxvel / Tacc;
	newArchlRef->maxd   = maxvel / Tdec;
	newArchlRef->zmaxa  = zmaxvel / ztacc;
	newArchlRef->hu 	= hu;
	newArchlRef->hh 	= hh;
	newArchlRef->hd 	= hd;
	newArchlRef->raxis	= raxis;
	
	for(int i = 0 ; i < totalAxis; ++i)
	{
		long startpos = beginPos[i];
		long dstpos	  = endPos[i];
		if (i==0)
		{
			newArchlRef->zstartpos = startpos;
			newArchlRef->zdstpos   = dstpos;
		}
		else
		{//直线插补轴
			double dist = (dstpos > startpos) ? (dstpos - startpos) : (startpos - dstpos);
			bool   do_update = false;
			if (i == raxis
				&& 2 == totalAxis)
			{//该轴为R轴，除了R轴+Z轴没有其它的轴
				do_update = true;
			}
			else if ( i != raxis
				&& dist > newArchlRef->max_dist)
				do_update = true;
			
			if (do_update)
				newArchlRef->max_dist = dist;				//参考轴运动距离
		}

		newArchlRef->reg_sv_on(i);
	}

	if (newArchlRef->zstartpos == hh)
		newArchlRef->hu = 0;			//此情况下hu无意义
	if (newArchlRef->zdstpos == hh)
		newArchlRef->hd = 0;			//此情况下hd无意义

	if (1 != newArchlRef->startPlan())
	{//规划失败
		retValue = -1;
		goto DONE;
	}

	int moments[ARCH_POINTS_CNT] = {0};
	int moment_cnt = 0;
	if (newArchlRef->max_dist > 1E-6)
	{//水平位移最大值非0
		moments[0] = 0;														//起始点
		moments[1] = newArchlRef->ts0;										//水平方向启动
		moments[2] = newArchlRef->up_param.cycles;							//Z轴到达hh
		moments[3] = newArchlRef->ts1;										//Z轴开始下降
		moments[4] = newArchlRef->ts0 + newArchlRef->line_param.cycles;		//水平方向停止
		moments[5] = newArchlRef->totalCycles() - 1;						//终止点

		moment_cnt = 6;
	}
	else
	{//水平位移最大值==0
		moments[0] = 0;									//起始点
		moments[1] = newArchlRef->up_param.cycles;		//Z轴到达hh
		moments[2] = newArchlRef->totalCycles() - 1;	//终止点
		moment_cnt = 3;
	}

	//去掉可能重复的点，否则雷赛控制器报错
	int save_cur, cur;
	for (cur = save_cur = 1; cur < moment_cnt; ++cur)
	{
		if (moments[cur] > moments[save_cur - 1])
		{
			moments[save_cur] = moments[cur];
			++save_cur;
		}
	}
	moment_cnt = save_cur;

	*ppResult = new long[moment_cnt * totalAxis];
	*pCycles  = moment_cnt;
	if (NULL == *ppResult)
	{
		retValue = -1;
		goto DONE;
	}

	for (int c = 0; c < moment_cnt; ++c)
	{
		newArchlRef->elapsed = moments[c];
		pTime[c] = moments[c] * 1.0 / CYCLES_PER_SEC;		//更新时间以秒为单位
		for(int i = 0; i < totalAxis; ++i)
		{
			long nextpos = 0;
			if (newArchlRef->lastCycle())//避免浮点数计算误差
				nextpos = endPos[i];
			else
			{
				if (0 == i)
				{//Z轴
					nextpos = newArchlRef->getZPosition(i);
				}
				else
				{//直线插补
					double distRatio = newArchlRef->getLineDistanceRatio(i); // >= 0
					nextpos = (int)(beginPos[i] +  distRatio * (endPos[i] - beginPos[i]) );
				}
			}
			(*ppResult)[c*totalAxis + i] = nextpos;
		}
	}
	
DONE:
	if (newArchlRef)
	{
		delete newArchlRef;
		newArchlRef = NULL;
	}
	return retValue;
}

int main()
{
	long startpos[] = {5520/*Z轴起始*/, 			1393, 		2391,3880};				//起始点坐标
	long endpos[]   = {2380/*Z轴终止*/, 	1868, 	22056, 4678};			//终止点坐标
	//long endpos[]   = {620/*Z轴终止*/, 	1868, 	22056, 4678};			//终止点坐标
	int cycles = ARCH_POINTS_CNT;
	double time[ARCH_POINTS_CNT];
	long *pResult = NULL;

	int totalAxis = sizeof(startpos)/sizeof(startpos[0]);
	
	if (0 == archl(totalAxis, startpos, endpos, 2, 150000, 150000/*最大速率*/, 0.1/*Z轴加速时间*/,
						0.08/*加速时间*/, 0.32/*减速时间*/,
						1500, 1200, 400, &pResult, time, &cycles))
	{
		std::ofstream ofs;
		ofs.open("DMCPOINTS.log",std::fstream::out | std::fstream::trunc);
		for(int c = 0; c < cycles; ++c)
		{
			for(int i = 0; i <totalAxis ; ++ i)
			{
				ofs << pResult[c * totalAxis + i] << "    ";
			}
			ofs << time[c] << "\n";
		}
	}

	if (pResult)
	{
		delete pResult;
		pResult = NULL;
	}
	return 0;
}
