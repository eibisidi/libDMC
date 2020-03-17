#ifndef SLAVE_STATE_H
#define SLAVE_STATE_H

#include "DMC.h"

#include "Poco/Mutex.h"

enum OpMode						//����ģʽ
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

//���״̬
class DriverSlaveState : public SlaveState
{
private:
	long 			curpos;		//��ǰλ��, ����������ʵʱ����
	unsigned short	status;		//״̬��
	OpMode			opmode;		//����ģʽ
	long			cmdpos;		//����λ��,��һ�ε��ʹ�ܺ󸳳�ֵ,��������
public:
	DriverSlaveState()
	{
		curpos = 0;
		status = 0;
		opmode = OPMODE_CSP;//OPMODE_NONE;
		cmdpos = 0;
	}

	inline long getCurPos() const {return curpos;}
	inline void setCurPos(long cp) {curpos = cp;}
	inline unsigned short getStatus() const {return status;}
	inline void setStatus(unsigned short s) {status  = s;}
	inline OpMode getOpMode() const {return opmode;}
	inline void setOpMode(OpMode op) {opmode  = op;}

	inline long getCmdPos () const
	{
		long cp;
		//mutex.lock();
		cp = cmdpos;
		//mutex.unlock();
		return cp;
	}
	
	inline void setCmdPos(long cp) 
	{
		//mutex.lock();
		cmdpos = cp;
		//mutex.unlock();
	}
};

class IoSlaveState : public SlaveState
{
private:
	unsigned int    input;			//����ֵ
	unsigned int    output;			//���ֵ
public:
	IoSlaveState()
	{
		input 	= 0;
		output  = 0;
	}

	inline unsigned int getInput() const {return input;}
	inline void setInput(unsigned int i) {input = i;}

	inline unsigned int getOutput() const {return output;}
	inline void setOutput(unsigned int o) {output = o;}
};

#endif
