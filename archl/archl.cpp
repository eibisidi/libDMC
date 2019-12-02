#include "AxisPara.h"
#include <fstream>
/*
功 能：多轴相对坐标的Z轴拱门运动。
输入参 数： TotalAxis： 插补轴数 >=1
		*startpos   起始点位置, startpos[0]为起始Z轴点位
		*endpos		终止点位置, endpos[0]为终止Z轴点位
		MaxVel： 最大行速度，单位： pps；
		Tacc： 加速时间，单位： s。
		hh:最高绝对高度
		hu:相对上升高度，相对于起始点 >= 0
		hd:相对下降高度，相对于终止点 >= 0
输出参数：ppResult 规划结果，行数为周期数，列数为轴数
		pCycles:   周期数
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
int archl(short totalAxis, const long *beginPos, const long *endPos, long zmaxvel,
				long maxvel, double ztacc, double Tacc, long hh, long hu, long hd, long ** ppResult, int * pCycles)
{
	if (totalAxis <= 0
			|| Tacc < 1E-6
			|| maxvel < 1E-6
			|| hu < 0
			|| hd < 0
			|| NULL == ppResult 
			|| NULL == pCycles)
			return -1;
	
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
	newArchlRef->zmaxa  = zmaxvel / ztacc;
	newArchlRef->hu 	= hu;
	newArchlRef->hh 	= hh;
	newArchlRef->hd 	= hd;
	
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
			if (dist > newArchlRef->max_dist)
				newArchlRef->max_dist = dist;				//参考轴运动距离
		}

		newArchlRef->reg_sv_on(i);
	}

	if (1 != newArchlRef->startPlan())
	{//规划失败
		retValue = -1;
		goto DONE;
	}

	int totalCycles = newArchlRef->totalCycles();
	*ppResult = new long[totalCycles * totalAxis];
	*pCycles  = totalCycles;
	if (NULL == *ppResult)
	{	
		retValue = -1;
		goto DONE;
	}
	
	for(int c = 0; c < totalCycles; ++c)
	{
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
	long startpos[3] = {2000/*Z轴起始*/, 			464, 		13000};				//起始点坐标
	long endpos[3]   = {3000/*Z轴终止*/, 	464, 	22509};			//终止点坐标
	int cycles = 0;
	long *pResult = NULL;

	int totalAxis = sizeof(startpos)/sizeof(startpos[0]);
	if (0 == archl(totalAxis, startpos, endpos, 200000, 150000/*最大速率*/, 0.1, 0.13/*加速时间*/, 
						1000, 500, 500, &pResult, &cycles))
	{
		std::ofstream ofs;
		ofs.open("DMCPOINTS.log",std::fstream::out | std::fstream::trunc);
		for(int c = 0; c < cycles; ++c)
		{
			for(int i = 0; i <totalAxis ; ++ i)
			{
				ofs << pResult[c * totalAxis + i] << "    ";
			}
			ofs << "\n";
		}
	}

	if (pResult)
	{
		delete pResult;
		pResult = NULL;
	}
	return 0;
}
