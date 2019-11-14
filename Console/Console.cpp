// Console.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include "DMC.h"
#include <Windows.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>

//#define  	TEST_MOVE
//#define 	TEST_HOME
//#define TEST_IO
#define TEST_LINE
//#define TEST_ARCHL
//#define TEST_DEC
//#define TEST_ISTOP
//#define TEST_INC
//#define TEST_PLAN

DWORD WINAPI ThreadFunc(LPVOID p)
{   
	DWORD ms;

	while(true)
    {
		d1000_home_move(2, 50000, 5000, 0.2);
		while(true)
		{
			ms = d1000_check_done(2);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_O_STOP)
		{
			printf("home error.\n");
			throw;
		}

		d1000_start_t_move(2, 50000, 0, 100000, 0.2);			//伺服最大速度3000rpm， 80%上限

		while(true)
		{
			ms = d1000_check_done(2);	

			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
			printf("move error.\n");
			throw;
		}

		Sleep(1000);
	}
	
    return 0;
}


int _tmain(int argc, _TCHAR* argv[])
{
	if (0  != d1000_board_init())
		return -1;
	
	//Sleep(3000);

	HANDLE hThread;
    DWORD  threadId;
	//hThread = CreateThread(NULL, 0, ThreadFunc, 0, 0, &threadId); // 创建线程
	
	DWORD ms;
#ifdef TEST_PLAN
	ArchlRef archRef;
	
#endif


#ifdef TEST_INC

	int i = 0;
	int move = -100;
	while (true)
	{
		int j = 0;
		while(j++ < 5)
		{
			d1000_start_t_move(1, move, 0, 400000, 0.2);			//伺服最大速度3000rpm， 80%上限
		
			while(1)
			{
				ms = d1000_check_done(1);	

				if (MOVESTATE_BUSY != ms)
					break;
			}

			if (ms != MOVESTATE_STOP)
			{
				printf("move error.\n");
				throw;
			}

			Sleep(10000);
		}
		move = -move;
	}

#endif

#ifdef TEST_ISTOP
	d1000_start_t_move(1, 50000, 0, 5000, 0.2);
	Sleep(2000);

	d1000_immediate_stop(1);

	while(1)
	{
		ms = d1000_check_done(1);
		
		if (MOVESTATE_BUSY != ms)
			break;
	}

	d1000_start_t_move(1, -50000, 0, 5000, 0.2);
	Sleep(20000);

#endif

#ifdef TEST_DEC

	srand(0);
	long startpos = d1000_get_command_pos(1);
	
	while (true)
	{
		d1000_start_t_move(1, 150000, 0, 400000, 0.2);


		Sleep(50 + rand() % 100);

		//d1000_decel_stop(1, 0.1);
		d1000_immediate_stop(1);

		while(1)
		{
			ms = d1000_check_done(1);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}
		if (ms != MOVESTATE_CMD_STOP && ms != MOVESTATE_STOP)
		{
			printf("d1000_decel_stop error. ms=%d.\n", (int)ms);
			throw;
		}
		//Sleep(2000);

		d1000_start_ta_move(1, startpos, 0, 400000, 0.2);
		while(1)
		{
			ms = d1000_check_done(1);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}
		if (ms != MOVESTATE_STOP)
		{
			printf("d1000_decel_stop error.\n");
			throw;
		}

		//Sleep(2000);
	}
#endif

#ifdef TEST_MOVE
	int move = 100000;
	int delta = 100;

	int i = 0;
	int axis = 1;
	long startpos = d1000_get_command_pos(axis);
	
	while (move < 150000)
	{
		printf("move = %d, i = %d.\n", move, i);

		switch (i% 4)
		{
		case 0:
			d1000_start_t_move(axis, move, 0, 30000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		case 1:
			d1000_start_ta_move(axis, startpos + move,0, 400000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		case 2:
			d1000_start_s_move(axis, move, 0,400000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		case 3:
			d1000_start_sa_move(axis, startpos + move,0, 400000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		}
		
		while(1)
		{
			ms = d1000_check_done(axis);	

			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
			printf("move error.\n");
			throw;
		}

		Sleep(2000);

		switch (i% 4)
		{
		case 0:
			d1000_start_t_move(axis, -move, 0,30000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		case 1:
			d1000_start_ta_move(axis, startpos,0, 400000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		case 2:
			d1000_start_s_move(axis, -move,  0,400000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		case 3:
			d1000_start_sa_move(axis, startpos, 0, 400000, 0.2);			//伺服最大速度3000rpm， 80%上限
			break;
		}

		while(1)
		{
			ms = d1000_check_done(axis);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
					printf("move error.\n");
			throw;
		}

		//move += delta;
		//++i;
		Sleep(10);
	}
#endif

#ifdef TEST_HOME

	while(true)
	{
		d1000_home_move(1, 50000, 5000, 0.2);
		while(true)
		{
			ms = d1000_check_done(1);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_O_STOP)
		{
			printf("home error.\n");
			throw;
		}

		d1000_start_t_move(1, -100000, 0, 100000, 0.2);			//伺服最大速度3000rpm， 80%上限

		while(true)
		{
			ms = d1000_check_done(1);	

			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
			printf("move error.\n");
			throw;
		}

		Sleep(1000);
	}
	//Sleep(200);
#endif

#ifdef TEST_LINE
	double rate = -1.5;
	while (true)
	{
		printf("rate=%f.", rate);
		int move = 50000;

		short axisArray[] = {1, 2};
		long  distArray[] = {move, rate*move};

		d1000_start_s_line(2, axisArray, distArray, 0, 30000, 0.2);		//

		while(1)
		{
			ms = d1000_check_done(1);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
			printf("move error.\n");
			throw;
		}

		
		short r_axisArray[] = {1,2};
		long  r_distArray[] = {-move, -rate*move};
		d1000_start_s_line(2, r_axisArray, r_distArray,0, 30000, 0.2);	//



		while(1)
		{
			ms = d1000_check_done(1);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
			printf("move error.\n");
			throw;
		}


		//Sleep(2000);
		rate += 0.1;
		if (fabs(rate) < 1E-6)
			rate = 0.1;
		if (fabs(rate - 1.6) < 1E-6)
			rate = -1.5;
	}
#endif
#ifdef TEST_ARCHL

	int zmove = -10000;
	int xmove = 100000;

	int hh = 100000;
	int hu = 10000;
	int hd = 20000;
	short r_axisArray[] = {3,1,2};
	long  r_distArray[] = {0,0,0};

	int zstartpos = d1000_get_command_pos(r_axisArray[0]);	//Z
	int	xstartpos = d1000_get_command_pos(r_axisArray[1]);
	int ystartpos = d1000_get_command_pos(r_axisArray[2]);

	int i = 10000;
	while(i--)
	{
		r_distArray[0] = rand() % 50000;
		r_distArray[1] = 50000 + rand() % 100000;
		r_distArray[2] = 50000 + rand() % 100000;

		printf("zmove = %d, xmove=%d, ymove = %d.\n", r_distArray[0], r_distArray[1], r_distArray[2]);
		
		d1000_start_t_archl(2, r_axisArray, r_distArray,30000, 0.2, hh, hu, hd);	//

		while(1)
		{
			ms = d1000_check_done(1);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
			printf("move error.\n");
			throw;
		}

		r_distArray[0] = zstartpos;
		r_distArray[1] = xstartpos;
		r_distArray[2] = ystartpos;

		d1000_start_ta_archl(2, r_axisArray, r_distArray,30000, 0.2, hh, hd, hu); //
		
		while(1)
		{
			ms = d1000_check_done(1);
			
			if (MOVESTATE_BUSY != ms)
				break;
		}

		if (ms != MOVESTATE_STOP)
		{
			printf("move error.\n");
			throw;
		}
	}
	
#endif

#ifdef TEST_IO
	unsigned int bits;
	ms = d1000_in_bit(2, &bits);
	printf("d1000_in_bit returns %d, bit=0x%x.\n", ms, bits);

	ms = d1000_out_bit(2, 0xFF);
	printf("d1000_out_bit returns %d.\n", ms);
#endif

	//d1000_immediate_stop(1);



	d1000_board_close();
	return 0;
}


