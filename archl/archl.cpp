#include "AxisPara.h"
#include <fstream>
/*
�� �ܣ�������������Z�Ṱ���˶���
����� ���� TotalAxis�� �岹���� >=1
		*startpos   ��ʼ��λ��, startpos[0]Ϊ��ʼZ���λ
		*endpos		��ֹ��λ��, endpos[0]Ϊ��ֹZ���λ
		raxis		R�������� ��� (r==-1)��������R��
		zmaxvel 	Z��������ٶȣ���λ�� pps
		maxvel		ˮƽ��������������ٶȣ���λ�� pps
		ztacc		Z��Ӽ���ʱ�䣬��λ:s
		Tacc�� 		ˮƽ���������ʱ�䣬��λ��s��
		Tdec:  		ˮƽ���������ʱ�䣬��λ��s
		hh:			��߾��Ը߶�
		hu:			��������߶ȣ��������ʼ�� >= 0
		hd:			����½��߶ȣ��������ֹ�� >= 0
���������	 ppResult 	�滮���������Ϊ�ؼ������������Ϊ����
		  pCycles:  �ؼ������
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
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
		{//ֱ�߲岹��
			double dist = (dstpos > startpos) ? (dstpos - startpos) : (startpos - dstpos);
			bool   do_update = false;
			if (i == raxis
				&& 2 == totalAxis)
			{//����ΪR�ᣬ����R��+Z��û����������
				do_update = true;
			}
			else if ( i != raxis
				&& dist > newArchlRef->max_dist)
				do_update = true;
			
			if (do_update)
				newArchlRef->max_dist = dist;				//�ο����˶�����
		}

		newArchlRef->reg_sv_on(i);
	}

	if (newArchlRef->zstartpos == hh)
		newArchlRef->hu = 0;			//�������hu������
	if (newArchlRef->zdstpos == hh)
		newArchlRef->hd = 0;			//�������hd������

	if (1 != newArchlRef->startPlan())
	{//�滮ʧ��
		retValue = -1;
		goto DONE;
	}

	int moments[ARCH_POINTS_CNT] = {0};
	int moment_cnt = 0;
	if (newArchlRef->max_dist > 1E-6)
	{//ˮƽλ�����ֵ��0
		moments[0] = 0;														//��ʼ��
		moments[1] = newArchlRef->ts0;										//ˮƽ��������
		moments[2] = newArchlRef->up_param.cycles;							//Z�ᵽ��hh
		moments[3] = newArchlRef->ts1;										//Z�Ὺʼ�½�
		moments[4] = newArchlRef->ts0 + newArchlRef->line_param.cycles;		//ˮƽ����ֹͣ
		moments[5] = newArchlRef->totalCycles() - 1;						//��ֹ��

		moment_cnt = 6;
	}
	else
	{//ˮƽλ�����ֵ==0
		moments[0] = 0;									//��ʼ��
		moments[1] = newArchlRef->up_param.cycles;		//Z�ᵽ��hh
		moments[2] = newArchlRef->totalCycles() - 1;	//��ֹ��
		moment_cnt = 3;
	}

	//ȥ�������ظ��ĵ㣬������������������
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
		pTime[c] = moments[c] * 1.0 / CYCLES_PER_SEC;		//����ʱ������Ϊ��λ
		for(int i = 0; i < totalAxis; ++i)
		{
			long nextpos = 0;
			if (newArchlRef->lastCycle())//���⸡�����������
				nextpos = endPos[i];
			else
			{
				if (0 == i)
				{//Z��
					nextpos = newArchlRef->getZPosition(i);
				}
				else
				{//ֱ�߲岹
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
	long startpos[] = {5520/*Z����ʼ*/, 			1393, 		2391,3880};				//��ʼ������
	long endpos[]   = {2380/*Z����ֹ*/, 	1868, 	22056, 4678};			//��ֹ������
	//long endpos[]   = {620/*Z����ֹ*/, 	1868, 	22056, 4678};			//��ֹ������
	int cycles = ARCH_POINTS_CNT;
	double time[ARCH_POINTS_CNT];
	long *pResult = NULL;

	int totalAxis = sizeof(startpos)/sizeof(startpos[0]);
	
	if (0 == archl(totalAxis, startpos, endpos, 2, 150000, 150000/*�������*/, 0.1/*Z�����ʱ��*/,
						0.08/*����ʱ��*/, 0.32/*����ʱ��*/,
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
