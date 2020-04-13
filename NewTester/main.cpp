#include "DMC.h"

#include <stdio.h>
#include <Windows.h>
#include <iostream>

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
					Sleep(1);						\
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

		ret = d1000_home_move(AXIS_R);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(AXIS_R, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}
		//同时回零
		short	AxisArray[]={AXIS_Y, AXIS_X, AXIS_LEFT, AXIS_RIGHT};
		short	TotalAxis = sizeof(AxisArray)/ sizeof(AxisArray[0]);
		ret = d1000_multi_home_move(TotalAxis, AxisArray);

		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(AXIS_Y, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}

		WAIT_DONE(AXIS_X, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}

		WAIT_DONE(AXIS_LEFT, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}
		
		WAIT_DONE(AXIS_RIGHT, ms)
		if (ms != MOVESTATE_O_STOP)
		{
			ret = -1;
			break;
		}

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

	short	axisArray[] = {AXIS_LEFT, AXIS_RIGHT};
	long	distArray[] = {30000, 30000};

	while (true)
	{
		ret  = d1000_start_t_line(sizeof(axisArray)/sizeof(axisArray[0]), axisArray, distArray, 0, 80000, 0.2);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(axisArray[0], ms)
		if (ms != MOVESTATE_STOP)
			break;
		WAIT_DONE(axisArray[1], ms)
		if (ms != MOVESTATE_STOP)
			break;
		d1000_out_bit(9, 3, 1);

		NEG_ARRAY(distArray);

		ret  = d1000_start_t_line(sizeof(axisArray)/sizeof(axisArray[0]), axisArray, distArray, 0, 80000, 0.2);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(axisArray[0], ms)
		if (ms != MOVESTATE_STOP)
			break;
		WAIT_DONE(axisArray[1], ms)
		if (ms != MOVESTATE_STOP)
			break;
		d1000_out_bit(9, 3, 0);

		NEG_ARRAY(distArray);

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
#if 0
	DWORD ret;
	DWORD ms;


	short 	axisArray[] = {1};
	long	VelArray[]	= {10000};

	char 	cmd;

	std::cout << "input cmd." << std::endl;
	while(std::cin >> cmd)
	{
		if (cmd == 'q')
			break;

		if (cmd == 'D')
		{
			ret = d1000_end_running(sizeof(axisArray)/sizeof(axisArray[0]), axisArray, 0.2);
			if (0 != ret)
			{
				printf("d1000_end_running failed. ret = %d\n", ret);
				return -1;
			}
		}
		if (cmd == 'S')
		{
			ret = d1000_start_running(sizeof(axisArray)/sizeof(axisArray[0]), axisArray,VelArray, 0.1);
			if (0 != ret)
			{
				printf("d1000_start_running failed. ret = %d\n", ret);
				return -1;
			}
		}

		if (cmd == 's')
		{
			ret = d1000_start_t_move(1, 100000,0,10000, 0.1);
			if (0 != ret)
			{
				printf("d1000_start_t_move failed. ret = %d\n", ret);
				return -1;
			}
		}

		if (cmd == 'd')
		{
			ret = d1000_decel_stop(1,  0.1);
			if (0 != ret)
			{
				printf("d1000_decel_stop failed. ret = %d\n", ret);
				return -1;
			}
		}

		if (cmd == 'A')
		{		
			ret = d1000_adjust(1,  10, 10000);
			if (0 != ret)
			{
				printf("d1000_adjust failed. ret = %d\n", ret);
				return -1;
			}
		}
		
		std::cout << "input cmd." << std::endl;
	}
	d1000_board_close();


	return 0;
#endif
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
	//LoaderThreadFunc(0);

#if 0
	DWORD	ret, ms;
	while (true)
	{
		ret  = d1000_start_t_move(AXIS_X, 170000, 0, 200000, 0.2);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(AXIS_X, ms)
		if (ms != MOVESTATE_STOP)
			break;

		ret  = d1000_start_t_move(AXIS_X, -170000, 0, 200000, 0.2);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(AXIS_X, ms)
		if (ms != MOVESTATE_STOP)
			break;
	}
#endif

	
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
		WAIT_DONE(axisArray[1], ms)
		if (ms != MOVESTATE_STOP)
			break;

		NEG_ARRAY(distArray);

		ret  = d1000_start_t_line(sizeof(axisArray)/sizeof(axisArray[0]), axisArray, distArray, 0, 100000, 0.2);
		if (ERR_NOERR != ret)
			break;
		WAIT_DONE(axisArray[0], ms)
		if (ms != MOVESTATE_STOP)
			break;
		WAIT_DONE(axisArray[1], ms)
		if (ms != MOVESTATE_STOP)
			break;

		NEG_ARRAY(distArray);
	}
#endif
	d1000_board_close();
	return 0;
}
