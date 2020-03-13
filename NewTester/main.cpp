#include "DMC.h"

#include <stdio.h>

#define AXIS_R (1)
#define AXIS_Y (2)
#define AXIS_X (3)
#define AXIS_TON (4)
#define AXIS_MAIN (5)
#define AXIS_RIGHT (6)
#define AXIS_LEFT (7)

#define WAIT_DONE(axis, ms) 						\
				while(true)							\
				{									\
					ms = d1000_check_done(axis);	\
					if (MOVESTATE_BUSY != ms)		\
						break;						\
				}									\

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
	}while(0);

	return ret;
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

	d1000_board_close();
	return 0;
}
