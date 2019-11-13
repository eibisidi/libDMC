#ifndef DMC_H
#define DMC_H


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef UNUSED
#define UNUSED(p) ((void)p)
#endif

typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;

typedef unsigned char  INT8U;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  INT8S;                   /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short INT16U;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short INT16S;                  /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   INT32U;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef int			   INT32S;                  /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float		   FP32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double		   FP64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */

#define DMC1000_API __declspec(dllexport)
#define WINAPI __stdcall

enum MoveState
{
	MOVESTATE_NONE = -1,
	MOVESTATE_BUSY = 0,			//正在运行
	MOVESTATE_STOP,				//运动位置到达
	MOVESTATE_TIMEOUT,			//运动超时
	MOVESTATE_ERR,				//运动错误（规划失败，电机使能失败等）
	MOVESTATE_CMD_STOP,			//急停、减速停止
	MOVESTATE_LIMIT_STOP,		//遇限位停止，未使用
	MOVESTATE_O_STOP,			//遇原点停止
};
	
enum ErrorNum{
	ERR_NOERR = 0,					//成功
	ERR_OPENUSB,					//ECM API OpenUsb失败
	ERR_DUP_INIT,					//重复初始化
	ERR_LOAD_XML,					//读取Master.xml配置文件失败
	ERR_ECM_WRITE,					//ECM API Write失败
	ERR_ECM_READ,					//ECM API Read失败
	ERR_ECM_PREOP,					//切换至PreOp模式失败
	ERR_ECM_SAFEOP,					//切换至SafeOp模式失败
	ERR_ECM_OP,						//切换至OP模式失败
	ERR_INVALID_ARG,				//无效的参数

	ERR_MEM,						//内存错误						
	ERR_AXIS_BUSY,					//电机正忙
	ERR_AXIS_NOT_MOVING,			//电机未处于运动状态
	
	ERR_NO_AXIS,					//驱动器不存在
	ERR_IO_NO_SLAVE,				//IO模块不存在
	ERR_IO_BUSY,					//IO模块正忙
	ERR_IO_READ_TIMEOUT,			//读取IO模块输入超时
	ERR_IO_WRITE_TIMEOUT,			//设置IO模块输出超时
	ERR_CLR_ALARM,					//清除告警失败
	ERR_INIT_AXIS,					//初始化驱动器失败
	ERR_SERVO_ON,					//驱动器使能失败
};

#define MAX_PULSE_PER_SEC(rpm, shaftrevo) ((rpm) * (shaftrevo) / 60)		//计算最大速度 (脉冲/s)


#ifdef __cplusplus
extern "C" {
#endif

/*
功 能：进行初始化，在调用其它函数前必须调用此函数进行初始化工作。
参 数：空
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD  WINAPI d1000_board_init(void);

/*
功 能：关闭，释放系统资源
参 数：空
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_board_close(void);

/*
功 能：以梯形速度曲线控制指定轴至运行速度，并以相对坐标运行一段指定距离。
参 数： axis：电机从站索引
		dist: 相对运动距离，单位： pulse，其值的正负表示运动方向；
		StrVel:起始速度，必须为0
		MaxVel：运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_t_move(short axis,long Dist, long StrVel, long MaxVel,double Tacc);

/*
功 能：以梯形速度曲线控制指定轴至运行速度，并以绝对坐标运行一段指定距离。
参 数： axis：电机从站索引
		Pos： 绝对运动位置，单位： pulse；
		StrVel:起始速度，必须为0
		MaxVel：运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_ta_move(short axis,long Pos, long StrVel, long MaxVel,double Tacc);

/*
功 能： 以 S 形速度曲线控制指定轴至运行速度，并以相对坐标运行一段指定距离；
参 数： axis：电机从站索引
		Dist： 相对运动距离，单位： pulse，其值的正负表示运动方向；
		StrVel:起始速度，必须为0
		MaxVel： 运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_s_move(short axis,long Dist, long StrVel, long MaxVel,double Tacc);

/*
功 能：以 S 形速度曲线控制指定轴至运行速度，并以绝对坐标运行一段指定距离。
参 数： axis：电机从站索引
		Pos： 绝对运动位置，单位： pulse；
		StrVel:起始速度，必须为0
		MaxVel：运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_sa_move(short axis,long Pos, long StrVel, long MaxVel,double Tacc);


/*功 能：启动指定轴进行回原点运动。
参 数： axis：电机从站索引
		highVel: 高速搜索减速点，单位： pps；
		lowVel：低速搜索原点，单位： pps；
		Tacc： 减速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_home_move(short axis,long highVel,long lowVel,double Tacc);

/*
功 能：检测指定轴的运动状态。
参 数： axis： 电机从站索引
返回值：正在运行 ：					MOVESTATE_BUSY
		脉冲输出完毕停止： 			MOVESTATE_STOP
		指令停止（如调用了 d1000_decel_stop 函数）MOVESTATE_CMD_STOP
		遇原点停止：				MOVESTATE_O_STOP
		运动错误				MOVESTATE_ERR
		运动超时				MOVESTATE_TIMEOUT
*/
DMC1000_API DWORD WINAPI d1000_check_done(short axis);

/*
功 能：减速停止指定轴脉冲输出。
参 数： axis： 电机从站索引
		tDec:  停止时间
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_decel_stop(short axis, double tDec);


/*
功 能：急停指定轴脉冲输出，电机关闭使能。
参 数： axis： 电机从站索引
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_immediate_stop(short axis);

/*
功 能：输出通用输出信号。
参 数： ioslave_idx：从站索引
		BitData：输出信号
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_out_bit(short ioslave_idx, unsigned int BitData);

/*
功 能：读取通用输入信号状态。
参 数： ioslave_idx：从站索引
输出参数：BitData：输入信号当前值
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_in_bit(short ioslave_idx, unsigned int *BitData);

//////////////////线性插补函数//////////////////
/*
功 能：启动多轴相对坐标的直线插补（T型速度曲线）。
参 数： TotalAxis： 插补轴数
		*AxisArray， AxisArray：电机从站索引列表；
		*DistArray， DistArray：对应轴号列表各轴的相对坐标的距离列表
		MaxVel： 运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_t_line(short TotalAxis,short *AxisArray,long *DistArray, long StrVel, long MaxVel, double Tacc);

/*
功 能：启动多轴绝对坐标的直线插补（T型速度曲线）。
参 数： TotalAxis： 插补轴数
		*AxisArray， AxisArray：电机从站索引列表；
		*PosArray， PosArray：对应轴号列表各轴的绝对坐标的位置列表；
		MaxVel： 运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_ta_line(short TotalAxis,short *AxisArray,long *PosArray, long StrVel,long MaxVel, double Tacc);

/*
功 能：启动多轴相对坐标的直线插补（S型速度曲线）。
参 数： TotalAxis： 插补轴数
		*AxisArray， AxisArray：电机从站索引列表；
		*DistArray， DistArray：对应轴号列表各轴的相对坐标的距离列表
		MaxVel： 运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_s_line(short TotalAxis,short *AxisArray,long *DistArray, long StrVel,long MaxVel, double Tacc);

/*
功 能：启动多轴绝对坐标的直线插补（S型速度曲线）。
参 数： TotalAxis： 插补轴数 >= 1
		*AxisArray， AxisArray：电机从站索引列表；
		*PosArray， PosArray：对应轴号列表各轴的绝对坐标的位置列表；
		MaxVel： 运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_sa_line(short TotalAxis,short *AxisArray,long *PosArray, long StrVel, long MaxVel, double Tacc);

/*
功 能：启动多轴相对坐标的Z轴拱门运动。
参 数： TotalAxis： 插补轴数 >=1 其中[0]轴代表Z轴，其它轴做直线插补
		*AxisArray， AxisArray：电机从站索引列表；
		*DistArray， DistArray：对应轴号列表各轴的相对坐标的距离列表
		MaxVel： 最大行速度，单位： pps；
		Tacc： 加速时间，单位： s。
		hh:最高绝对高度
		hu:相对上升高度，相对于终止点
		hd:相对下降高度，相对于终止点
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_t_archl(short TotalAxis,short *AxisArray,long *DistArray,long MaxVel, double Tacc,
			long hh, long hu, long hd);

/*
功 能：启动多轴绝对坐标的Z轴拱门运动。
参 数： TotalAxis： 插补轴数 >=1 其中[0]轴代表Z轴，其它轴做直线插补
		*AxisArray， AxisArray：电机从站索引列表；
		*PosArray， PosArray：对应轴号列表各轴的绝对坐标的位置列表；
		MaxVel： 运行速度，单位： pps；
		Tacc： 加速时间，单位： s。
		hh:最高绝对高度
		hu:相对上升高度，相对于终止点
		hd:相对下降高度，相对于终止点
返回值：正确：返回 ERR_NoError；
错误：返回相关错误码。
*/
DMC1000_API DWORD WINAPI d1000_start_ta_archl(short TotalAxis,short *AxisArray,long *PosArray,long MaxVel, double Tacc, long hh, long hu, long hd);


//////////////////位置设定和读取函数////////////
/*
功 能：读取指令位置计数器计数值。
参 数： axis：电机从站索引
返回值：指定轴当前指令位置计数器值，单位： pulse。
*/
DMC1000_API long WINAPI d1000_get_command_pos(short axis);


#ifdef __cplusplus
}
#endif

#endif
