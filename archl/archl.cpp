#include "AxisPara.h"
#include <fstream>
/*
�� �ܣ�������������Z�Ṱ���˶���
����� ���� TotalAxis�� �岹���� >=1
		*startpos   ��ʼ��λ��, startpos[0]Ϊ��ʼZ���λ
		*endpos		��ֹ��λ��, endpos[0]Ϊ��ֹZ���λ
		MaxVel�� ������ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
		hh:��߾��Ը߶�
		hu:��������߶ȣ��������ʼ�� >= 0
		hd:����½��߶ȣ��������ֹ�� >= 0
���������ppResult �滮���������Ϊ������������Ϊ����
		pCycles:   ������
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
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
		{//ֱ�߲岹��
			double dist = (dstpos > startpos) ? (dstpos - startpos) : (startpos - dstpos);
			if (dist > newArchlRef->max_dist)
				newArchlRef->max_dist = dist;				//�ο����˶�����
		}

		newArchlRef->reg_sv_on(i);
	}

	if (1 != newArchlRef->startPlan())
	{//�滮ʧ��
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
	long startpos[3] = {2000/*Z����ʼ*/, 			464, 		13000};				//��ʼ������
	long endpos[3]   = {3000/*Z����ֹ*/, 	464, 	22509};			//��ֹ������
	int cycles = 0;
	long *pResult = NULL;

	int totalAxis = sizeof(startpos)/sizeof(startpos[0]);
	if (0 == archl(totalAxis, startpos, endpos, 200000, 150000/*�������*/, 0.1, 0.13/*����ʱ��*/, 
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
