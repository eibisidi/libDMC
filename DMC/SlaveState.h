#ifndef SLAVE_STATE_H
#define SLAVE_STATE_H

#include "DMC.h"

#include "Poco/Mutex.h"

enum OpMode						//控制模式
{
	OPMODE_NONE	  = -1,
	OPMODE_HOMING = HOMING_MODE,
	OPMODE_CSP	  = CSP_MODE,
	OPMODE_CSV	  = CSV_MODE,
	OPMODE_CST	  = CST_MODE,
};

enum IoRequestState
{
	IORS_NONE 	= -1,
	IORS_BUSY 	= 0,
	IORS_SUCCESS,
	IORS_TIMEOUT,
};

class SlaveState
{
private:
	unsigned int	slave_state;
protected:
	Poco::Mutex		mutex;
public:
	SlaveState()
	{
		slave_state = MOVESTATE_NONE;
	}

	inline unsigned int getSlaveState()
	{
		unsigned int ss = MOVESTATE_NONE;
		mutex.lock();
		ss = slave_state;
		mutex.unlock();
		return ss;
	}

	inline void setSlaveState(unsigned int ss)
	{
		mutex.lock();
		slave_state = ss;
		mutex.unlock();
	}

	virtual ~SlaveState()
	{
	}
};

//电机状态
class DriverSlaveState : public SlaveState
{
private:
	long 			curpos;		//当前位置, 驱动器返回实时更新
	unsigned short	status;		//状态字
	OpMode			opmode;		//操作模式
	long			cmdpos;		//绝对位置,第一次电机使能后赋初值,回零后更新
	unsigned short 	alarmcode;	//报警码
public:
	DriverSlaveState()
	{
		curpos = 0;
		status = 0;
		opmode = OPMODE_CSP;//OPMODE_NONE;
		cmdpos = 0;
		alarmcode = 0;
	}

	inline long getCurPos() const {return curpos;}
	inline void setCurPos(long cp) {curpos = cp;}
	inline unsigned short getStatus() const {return status;}
	inline void setStatus(unsigned short s) {status  = s;}
	inline OpMode getOpMode() const {return opmode;}
	inline void setOpMode(OpMode op) {opmode  = op;}
	inline long getCmdPos () const{return cmdpos;}
	inline void setCmdPos(long cp) {cmdpos = cp;}
	inline unsigned short getAlarmCode () const{return alarmcode;}
	inline void setAlarmCode(unsigned short ac) {alarmcode = ac;}
};

class IoSlaveState : public SlaveState
{
private:
	unsigned int    input;			//输入值
	unsigned int    output;			//输出值
	bool			read;
public:
	IoSlaveState()
	{
		input 	= 0;
		output  = 0;
		read	= false;
	}

	inline unsigned int getInput() const {return input;}
	inline void setInput(unsigned int i) {input = i;}

	inline unsigned int getOutput() const {return output;}
	inline void setOutput(unsigned int o) {output = o;}
	inline bool	flip() { read = !read; return read;}
};

#endif
