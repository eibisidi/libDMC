#include "DMC.h"

#include <stdio.h>
#include <Windows.h>

#define AXIS_R (1)
#define AXIS_Y (2)
#define AXIS_X (3)
#define AXIS_TON (4)
#define AXIS_MAIN (5)
#define AXIS_RIGHT (6)
#define AXIS_LEFT (7)
#define IO_1   (9)
#define IO_2   (8)

#define WAIT_DONE(axis, ms) 						\
				while(true)							\
				{									\
					ms = d1000_check_done(axis);	\
					if (MOVESTATE_BUSY != ms)		\
						break;						\
				}									\

#define NEG_ARRAY(array) 											\
				{																	\
						for(int i = 0; i < sizeof(array) / sizeof(array[0]); ++i)	\
							array[i] = -array[i];									\
				}


DWORD all_go_home()
{
	DWORD ret;
	DWORD ms;
	do {
		ret = d1000_home_move(AXIS_R, 100, 50, 100/*ms*/);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(AXIS_R, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}


		ret = d1000_home_move(AXIS_Y, 1000, 100, 100/*ms*/);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(AXIS_Y, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}

		ret = d1000_home_move(AXIS_X, 1000, 100, 100/*ms*/);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(AXIS_X, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}

#if 1
		ret = d1000_home_move(AXIS_LEFT, 2000, 1000, 10000);
		if (ERR_NOERR != ret)
			break;

		WAIT_DONE(AXIS_LEFT, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}

		ret = d1000_home_move(AXIS_RIGHT, 2000, 1000, 10000);
		if (ERR_NOERR != ret)
			break;

		WAIT_DONE(AXIS_RIGHT, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}
#endif
	}while(0);

	return ret;
}

DWORD WINAPI IoThreadFunc(LPVOID p)
{
	DWORD ret;
	DWORD in;
	short out;

	printf("Start IO.\n");
	
	while(true)
	{
		in = d1000_in_bit(IO_1, 10);
		printf("current in=%d \n", in);

		out = d1000_get_outbit(IO_1, 15);
		printf("current out=%d \n", out);

		out = out ? 0 : 1;
		ret = d1000_out_bit(IO_1, 15, out);
		if (ERR_NOERR != ret)
			return -1;


		out = d1000_get_outbit(IO_2, 8);
		printf("current out=%d \n", out);

		out = out ? 0 : 1;
		ret = d1000_out_bit(IO_2, 8, out);
		if (ERR_NOERR != ret)
			return -1;

		Sleep(2000);
	}

}

DWORD WINAPI LoaderThreadFunc(LPVOID p)
{   
	DWORD ret, ms;
	printf("Start loading.\n");

	while(true)
    {
    	//R->30000
		ret  = d1000_start_ta_move(AXIS_RIGHT, 30000, 0, 20000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_RIGHT, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

    	//L->20000
		ret  = d1000_start_ta_move(AXIS_LEFT, 20000, 0, 20000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_LEFT, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

    	//L->0
		ret  = d1000_start_ta_move(AXIS_LEFT, 0, 0, 20000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_LEFT, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

		//R->0
		ret  = d1000_start_ta_move(AXIS_RIGHT, 0, 0, 20000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_RIGHT, ms)
		if (ms != MOVESTATE_STOP)
			return -1;
	}
	
    return 0;
}

DWORD WINAPI RotatorThreadFunc(LPVOID p)
{   
	DWORD ret, ms;
	printf("Start rotating.\n");

	int dist = 3000;

	while(true)
    {
		ret  = d1000_start_t_move(AXIS_R, dist, 0, 1000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_R, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

		dist = -dist;

		ret  = d1000_start_t_move(AXIS_R, dist, 0, 1000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_R, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

		dist = -dist;
	}
	
    return 0;
}

DWORD WINAPI MainThreadFunc(LPVOID p)
{   
	DWORD ret, ms;
	printf("Start main.\n");

	int dist = 10000;

	while(true)
    {
		ret  = d1000_start_t_move(AXIS_MAIN, dist, 0, 10000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_MAIN, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

		dist = -dist;

		ret  = d1000_start_t_move(AXIS_MAIN, dist, 0, 10000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_MAIN, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

		dist = -dist;
	}
	
    return 0;
}

DWORD WINAPI TongueThreadFunc(LPVOID p)
{   
	DWORD ret, ms;
	printf("Start main.\n");

	int dist = 10000;

	while(true)
    {
		ret  = d1000_start_t_move(AXIS_TON, dist, 0, 10000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_TON, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

		dist = -dist;

		ret  = d1000_start_t_move(AXIS_TON, dist, 0, 10000, 0.2);
		if (ERR_NOERR != ret)
			return -1;
		WAIT_DONE(AXIS_TON, ms)
		if (ms != MOVESTATE_STOP)
			return -1;

		dist = -dist;
	}
	
    return 0;
}

int main()
{
	if (0  != d1000_board_init())
	{
		printf("d1000_board_init failed.\n");
		return -1;
	}

	if (0 != all_go_home())
	{
		printf("all_go_home failed.\n");
		return -1;
	}

	printf("All Homed.\n");

	HANDLE hThread;
	hThread = CreateThread(NULL, 0, IoThreadFunc, 0, 0, NULL); // 创建线程
	hThread = CreateThread(NULL, 0, LoaderThreadFunc, 0, 0, NULL); // 创建线程
	hThread = CreateThread(NULL, 0, RotatorThreadFunc, 0, 0, NULL); // 创建线程
	hThread = CreateThread(NULL, 0, MainThreadFunc, 0, 0, NULL); // 创建线程
	hThread = CreateThread(NULL, 0, TongueThreadFunc, 0, 0, NULL); // 创建线程

#if 1
	short 	axisArray[] = {AXIS_X, AXIS_Y};
	long	distArray[] = {50000, 50000};
	DWORD   ret, ms;
	while (true)
	{
		ret  = d1000_start_t_line(sizeof(axisArray)/sizeof(axisArray[0]), axisArray, distArray, 0, 100000, 0.2);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(axisArray[0], ms)
		if (ms != MOVESTATE_STOP)
			break;

		NEG_ARRAY(distArray);

		ret  = d1000_start_t_line(sizeof(axisArray)/sizeof(axisArray[0]), axisArray, distArray, 0, 100000, 0.2);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(axisArray[0], ms)
		if (ms != MOVESTATE_STOP)
			break;

		NEG_ARRAY(distArray);
	}
#endif

	d1000_board_close();
	return 0;
}
