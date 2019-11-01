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

typedef unsigned char  INT8U;                   /* defined for unsigned 8-bits integer variable 	�޷���8λ���ͱ���  */
typedef signed   char  INT8S;                   /* defined for signed 8-bits integer variable		�з���8λ���ͱ���  */
typedef unsigned short INT16U;                  /* defined for unsigned 16-bits integer variable 	�޷���16λ���ͱ��� */
typedef signed   short INT16S;                  /* defined for signed 16-bits integer variable 		�з���16λ���ͱ��� */
typedef unsigned int   INT32U;                  /* defined for unsigned 32-bits integer variable 	�޷���32λ���ͱ��� */
typedef int			   INT32S;                  /* defined for signed 32-bits integer variable 		�з���32λ���ͱ��� */
typedef float		   FP32;                    /* single precision floating point variable (32bits) �����ȸ�������32λ���ȣ� */
typedef double		   FP64;                    /* double precision floating point variable (64bits) ˫���ȸ�������64λ���ȣ� */

#define DMC1000_API __declspec(dllexport)
#define WINAPI __stdcall

enum MoveState
{
	MOVESTATE_NONE = -1,
	MOVESTATE_BUSY = 0,			//��������
	MOVESTATE_STOP,				//�˶�λ�õ���
	MOVESTATE_TIMEOUT,			//�˶���ʱ
	MOVESTATE_ERR,				//�˶����󣨹滮ʧ�ܣ����ʹ��ʧ�ܵȣ�
	MOVESTATE_CMD_STOP,			//��ͣ������ֹͣ
	MOVESTATE_LIMIT_STOP,		//����λֹͣ��δʹ��
	MOVESTATE_O_STOP,			//��ԭ��ֹͣ
};
	
enum ErrorNum{
	ERR_NOERR = 0,					//�ɹ�
	ERR_OPENUSB,
	ERR_SET_DRIVER_CURRENT,
	ERR_DUP_INIT,
	ERR_LOAD_XML,
	ERR_ECM_WRITE,
	ERR_ECM_READ,
	ERR_ECM_PREOP,
	ERR_ECM_SAFEOP,
	ERR_ECM_OP,
	ERR_ECM_SETAXIS,
	ERR_MEM,
	ERR_AXIS_BUSY,
	ERR_AXIS_NOT_MOVING,
	ERR_T_PLAN,
	ERR_NO_AXIS,
	ERR_IO_NO_SLAVE,
	ERR_IO_BUSY,
	ERR_IO_READ_TIMEOUT,
	ERR_IO_WRITE_TIMEOUT,
	ERR_CLR_ALARM,
	ERR_CLR_SET_GEARRATIO,
	ERR_SERVO_ON,
};

#define MAX_PULSE_PER_SEC(rpm, shaftrevo) ((rpm) * (shaftrevo) / 60)		//��������ٶ� (����/s)


#ifdef __cplusplus
extern "C" {
#endif

/*
�� �ܣ����г�ʼ�����ڵ�����������ǰ������ô˺������г�ʼ��������
�� ������
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD  WINAPI d1000_board_init(void);

/*
�� �ܣ��رգ��ͷ�ϵͳ��Դ
�� ������
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_board_close(void);

/*
�� �ܣ��������ٶ����߿���ָ�����������ٶȣ����������������һ��ָ�����롣
�� ���� axis�������վ����
		dist: ����˶����룬��λ�� pulse����ֵ��������ʾ�˶�����
		StrVel:��ʼ�ٶȣ�����Ϊ0
		MaxVel�������ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_t_move(short axis,long Dist, long StrVel, long MaxVel,double Tacc);

/*
�� �ܣ��������ٶ����߿���ָ�����������ٶȣ����Ծ�����������һ��ָ�����롣
�� ���� axis�������վ����
		Pos�� �����˶�λ�ã���λ�� pulse��
		StrVel:��ʼ�ٶȣ�����Ϊ0
		MaxVel�������ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_ta_move(short axis,long Pos, long StrVel, long MaxVel,double Tacc);

/*
�� �ܣ� �� S ���ٶ����߿���ָ�����������ٶȣ����������������һ��ָ�����룻
�� ���� axis�������վ����
		Dist�� ����˶����룬��λ�� pulse����ֵ��������ʾ�˶�����
		StrVel:��ʼ�ٶȣ�����Ϊ0
		MaxVel�� �����ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_s_move(short axis,long Dist, long StrVel, long MaxVel,double Tacc);

/*
�� �ܣ��� S ���ٶ����߿���ָ�����������ٶȣ����Ծ�����������һ��ָ�����롣
�� ���� axis�������վ����
		Pos�� �����˶�λ�ã���λ�� pulse��
		StrVel:��ʼ�ٶȣ�����Ϊ0
		MaxVel�������ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_sa_move(short axis,long Pos, long StrVel, long MaxVel,double Tacc);


/*�� �ܣ�����ָ������л�ԭ���˶���
�� ���� axis�������վ����
		highVel: �����������ٵ㣬��λ�� pps��
		lowVel����������ԭ�㣬��λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_home_move(short axis,long highVel,long lowVel,double Tacc);

/*
�� �ܣ����ָ������˶�״̬��
�� ���� axis�� �����վ����
����ֵ���������� ��					MOVESTATE_BUSY
		����������ֹͣ�� 			MOVESTATE_STOP
		ָ��ֹͣ��������� d1000_decel_stop ������MOVESTATE_CMD_STOP
		��ԭ��ֹͣ��				MOVESTATE_O_STOP
		�˶�����				MOVESTATE_ERR
		�˶���ʱ				MOVESTATE_TIMEOUT
*/
DMC1000_API DWORD WINAPI d1000_check_done(short axis);

/*
�� �ܣ�����ָֹͣ�������������
�� ���� axis�� �����վ����
		tDec:  ֹͣʱ��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_decel_stop(short axis, double tDec);


/*
�� �ܣ���ָͣ�����������������ر�ʹ�ܡ�
�� ���� axis�� �����վ����
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_immediate_stop(short axis);

/*
�� �ܣ����ͨ������źš�
�� ���� ioslave_idx����վ����
		BitData������ź�
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_out_bit(short ioslave_idx, unsigned int BitData);

/*
�� �ܣ���ȡͨ�������ź�״̬��
�� ���� ioslave_idx����վ����
���������BitData�������źŵ�ǰֵ
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_in_bit(short ioslave_idx, unsigned int *BitData);

//////////////////���Բ岹����//////////////////
/*
�� �ܣ�����������������ֱ�߲岹��T���ٶ����ߣ���
�� ���� TotalAxis�� �岹����
		*AxisArray�� AxisArray�������վ�����б�
		*DistArray�� DistArray����Ӧ����б������������ľ����б�
		MaxVel�� �����ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_t_line(short TotalAxis,short *AxisArray,long *DistArray, long StrVel, long MaxVel, double Tacc);

/*
�� �ܣ�����������������ֱ�߲岹��T���ٶ����ߣ���
�� ���� TotalAxis�� �岹����
		*AxisArray�� AxisArray�������վ�����б�
		*PosArray�� PosArray����Ӧ����б����ľ��������λ���б�
		MaxVel�� �����ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_ta_line(short TotalAxis,short *AxisArray,long *PosArray, long StrVel,long MaxVel, double Tacc);

/*
�� �ܣ�����������������ֱ�߲岹��S���ٶ����ߣ���
�� ���� TotalAxis�� �岹����
		*AxisArray�� AxisArray�������վ�����б�
		*DistArray�� DistArray����Ӧ����б������������ľ����б�
		MaxVel�� �����ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_s_line(short TotalAxis,short *AxisArray,long *DistArray, long StrVel,long MaxVel, double Tacc);

/*
�� �ܣ�����������������ֱ�߲岹��S���ٶ����ߣ���
�� ���� TotalAxis�� �岹����
		*AxisArray�� AxisArray�������վ�����б�
		*PosArray�� PosArray����Ӧ����б����ľ��������λ���б�
		MaxVel�� �����ٶȣ���λ�� pps��
		Tacc�� ����ʱ�䣬��λ�� s��
����ֵ����ȷ������ ERR_NoError��
���󣺷�����ش����롣
*/
DMC1000_API DWORD WINAPI d1000_start_sa_line(short TotalAxis,short *AxisArray,long *PosArray, long StrVel, long MaxVel, double Tacc);


//////////////////λ���趨�Ͷ�ȡ����////////////
/*
�� �ܣ���ȡָ��λ�ü���������ֵ��
�� ���� axis�������վ����
����ֵ��ָ���ᵱǰָ��λ�ü�����ֵ����λ�� pulse��
*/
DMC1000_API long WINAPI d1000_get_command_pos(short axis);



#ifdef __cplusplus
}
#endif

#endif
