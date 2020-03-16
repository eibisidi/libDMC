#include "DMC.h"
#include "DmcManager.h"

DWORD WINAPI d1000_board_init(void)
{
	return DmcManager::instance().init();
}

DWORD WINAPI d1000_board_close(void)
{
	DmcManager::instance().close();
	return ERR_NOERR;
}

DWORD WINAPI d1000_start_t_move(short axis,long Dist, long StrVel, long MaxVel,double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_move(axis, Dist, MaxVel, Tacc, false, MOVETYPE_T);
}

DWORD WINAPI d1000_start_ta_move(short axis,long Pos, long StrVel, long MaxVel,double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_move(axis, Pos, MaxVel, Tacc, true, MOVETYPE_T);
}

DWORD WINAPI d1000_start_s_move(short axis,long Dist, long StrVel, long MaxVel,double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_move(axis, Dist, MaxVel, Tacc, false, MOVETYPE_S);
}

DWORD WINAPI d1000_start_sa_move(short axis,long Pos, long StrVel, long MaxVel,double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_move(axis, Pos, MaxVel, Tacc, true, MOVETYPE_S);
}

DWORD WINAPI d1000_home_move(short axis,long highVel,long lowVel,long acc)
{
	return DmcManager::instance().home_move(axis, highVel, lowVel, acc);
}

DWORD WINAPI d1000_check_done(short axis)
{
	return DmcManager::instance().check_done(axis);
}

DWORD WINAPI d1000_decel_stop(short axis, double tDec)
{
	return DmcManager::instance().decel_stop(axis, tDec);
}

DWORD WINAPI d1000_immediate_stop(short axis)
{
	return DmcManager::instance().immediate_stop(axis);
}

DWORD WINAPI d1000_out_bit(short ioslave_idx, short BitNo, short BitData)
{
	return DmcManager::instance().out_bit(ioslave_idx, BitNo, BitData);
}

DWORD WINAPI d1000_get_outbit(short ioslave_idx, short BitNo)
{
	unsigned int output = DmcManager::instance().getIoOutput(ioslave_idx);
	return (output & (1 << BitNo));
}

DWORD WINAPI d1000_in_bit(short ioslave_idx, short BitNo)
{
	unsigned int input = 0;
	DmcManager::instance().in_bit(ioslave_idx, &input);
	return (input & (1 << BitNo));
}

DWORD WINAPI d1000_start_t_line(short TotalAxis,short *AxisArray,long *DistArray,long StrVel, long MaxVel, double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_line(TotalAxis, AxisArray, DistArray, MaxVel, Tacc, false, MOVETYPE_T);
}

DWORD WINAPI d1000_start_ta_line(short TotalAxis,short *AxisArray,long *PosArray, long StrVel, long MaxVel, double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_line(TotalAxis, AxisArray, PosArray, MaxVel, Tacc, true, MOVETYPE_T);
}

DWORD WINAPI d1000_start_s_line(short TotalAxis,short *AxisArray,long *DistArray, long StrVel,long MaxVel, double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_line(TotalAxis, AxisArray, DistArray, MaxVel, Tacc, false, MOVETYPE_S);
}

DWORD WINAPI d1000_start_sa_line(short TotalAxis,short *AxisArray,long *PosArray, long StrVel, long MaxVel, double Tacc)
{
	UNUSED(StrVel);
	return DmcManager::instance().start_line(TotalAxis, AxisArray, PosArray, MaxVel, Tacc, true, MOVETYPE_S);
}

DWORD WINAPI d1000_start_t_archl(short TotalAxis,short *AxisArray,long *DistArray,long MaxVel, double Tacc, long hh, long hu, long hd)
{
	return DmcManager::instance().start_archl(TotalAxis, AxisArray, DistArray, MaxVel, Tacc, false, hh, hu, hd);
}

DWORD WINAPI d1000_start_ta_archl(short TotalAxis,short *AxisArray,long *PosArray,long MaxVel, double Tacc, long hh, long hu, long hd)
{
	return DmcManager::instance().start_archl(TotalAxis, AxisArray, PosArray, MaxVel, Tacc, true,  hh, hu, hd);
}

long WINAPI d1000_get_command_pos(short axis)
{
	return DmcManager::instance().get_command_pos(axis);
}
