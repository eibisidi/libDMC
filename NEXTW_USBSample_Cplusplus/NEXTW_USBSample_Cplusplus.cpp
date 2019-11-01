// NEXTW_USBSample_Cplusplus.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include <Windows.h>
// STEP 1 - include library (NEXTWUSBLib_12B.h or NEXTWUSBLib_16B.h)
#include "NEXTWUSBLib_12B.h"

void ClearCmdData(transData* data)
{
	for (int i = 0; i < DEF_MA_MAX; i++)
	{
		data[i].CMD = data[i].Parm = 0;
		data[i].Data1 = data[i].Data2 = 0;
	}
}

bool checkResponse(char slaves, char column, unsigned int expectValue, unsigned int retry)
{
	unsigned int realValue=0xFEDCBA98; //inital data
	unsigned int retried;
	transData respData[DEF_MA_MAX];
	for(retried = 0;retried <= retry;retried++)
	{
		ECMUSBRead((unsigned char*)respData,sizeof(respData));

		if (column ==0)
			if (slaves == DEF_MA_MAX-1)
				realValue = respData[slaves].CMD;
			else
				realValue = respData[slaves].CMD & 0xFFFF;
		else if (column==1)
		{
			if (slaves == 0)
				realValue = (respData[slaves].Parm) & 0xFF;
			else
				realValue = respData[slaves].Parm;
		}
		else if (column==2)
			realValue = respData[slaves].Data1;
		else
			realValue = respData[slaves].Data2;

		if (realValue == expectValue)
		{
			// printf("PASS\n");
			break;
		}
		else if (retry == retried)
		{
			printf("Fail,realValue= %d   expectValue= %d, retry = %d \n", realValue, expectValue, retried);
		}
		else
		{	

			ClearCmdData(respData);
			ECMUSBWrite((unsigned char*)respData,sizeof(respData));
		}
	}						
	return (realValue == expectValue);
}

int _tmain(int argc, _TCHAR* argv[])
{
	bool retValue, isOpen = false;
	transData cmdData[DEF_MA_MAX], respData[DEF_MA_MAX];
#pragma region STEP 2 - Open the USB Port
	retValue = OpenECMUSB();
	if (retValue)
		isOpen = true;
	else
	{
		printf("Open USB Fail");
		return 0;
	}
	Sleep(100);
#pragma endregion

#pragma region STEP 3 - SET_STATE to PreOP
	ClearCmdData(cmdData);
	cmdData[0].CMD = SET_STATE;
	cmdData[0].Data1 = STATE_PRE_OP; // set state to pre-OP
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");	
	checkResponse(0,1,2,100);
#pragma endregion

#pragma region STEP 4 - SET_AXIS
	ClearCmdData(cmdData);
	byte slaveType[] = { DRIVE, IO, HSP, DRIVE, DRIVE, None, None, None,
		None, None, None, None, None, None, None, None,
		None, None, None, None, None, None, None, None,
		None, None, None, None, None, None, None, None,
		None, None, None, None, None, None, None, None};
	unsigned int topology;
	int j=0;
	for (int i = 1 ; i<=5 ; i++)
	{		
		topology = 0;
		for (; j < i*8; j++)
		{
			topology |= (slaveType[j] << (j%8)*4);
		}
		cmdData[0].CMD = SET_AXIS;
		cmdData[0].Parm = i-1; //Group
		cmdData[0].Data1 = topology;
		cmdData[0].Data2 = 0;
		if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
			printf("Write Error \n");
		Sleep(2);
	}
#pragma endregion

#pragma region STEP 5 - SET_DC
	cmdData[0].CMD = SET_DC;
	cmdData[0].Parm = 0;
	cmdData[0].Data1 = 2000; // set cycle time to 2000 us
	cmdData[0].Data2 = 0xFFFF; // Auto offet adjustment

	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	Sleep(2);
#pragma endregion

#pragma region STEP 6 - Set DRIVE_MODE
	ClearCmdData(cmdData);
	for (int i = 1; i < DEF_MA_MAX-1; i++)
	{
		if (slaveType[i-1] == DRIVE || slaveType[i-1] == HSP )
		{
			cmdData[i].CMD = DRIVE_MODE;
			cmdData[i].Data1 = CSP_MODE; // set to CSP mode
			cmdData[i].Data2 = DCSYNC; // set using DC sync
		}
	}
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	Sleep(2);
#pragma endregion

#pragma region STEP 7 - SET_STATE to SafeOP (This step could be skipped)
	ClearCmdData(cmdData);
	cmdData[0].CMD = SET_STATE;
	cmdData[0].Data1 = STATE_SAFE_OP;// set state to Safe-OP
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	checkResponse(0,1,4,1000);  // wait for enter to safe op mode
#pragma endregion

#pragma region STEP 8 - SET_STATE to OP
	ClearCmdData(cmdData);
	cmdData[0].CMD = SET_STATE;
	cmdData[0].Data1 = STATE_OPERATIONAL;// set state to OP
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	checkResponse(0,1,8,1000);// wait for enter to op mode
#pragma endregion
	Sleep(3000);
#pragma region STEP 9 - SET Servo On
	ClearCmdData(cmdData);
	for (int i = 1; i < DEF_MA_MAX-1; i++)
	{
		if (slaveType[i-1] == DRIVE || slaveType[i-1] == HSP)  //set drive or HSP slave servo on
			cmdData[i].CMD = SV_ON;
	}
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	Sleep(1000); // wait for servo on 
#pragma endregion

#pragma region STEP 9.1	- Read Current Position
	ECMUSBRead((unsigned char*)respData,sizeof(respData));
	int TargerPosition[DEF_MA_MAX];
	for (int i = 1; i < DEF_MA_MAX; i++)
	{
		TargerPosition[i] = respData[i].Data1;
	}
#pragma endregion

#pragma region STEP 10- SET Cycle Synchronized Position
	Sleep(1000); // wait until motion stop totally
	ClearCmdData(cmdData);
	ECMUSBRead((unsigned char*)respData,sizeof(respData));
	printf("Current Position: Slave 1 = %d, Slave 2 = %d, Slave 3 = %d.\n",respData[1].Data1,respData[2].Data1,respData[3].Data1);
	for (int i = 0; i < 1000; i++) // send 1000 times, let motor move 1000 * 100 pulse
	{		
		for (int j = 1; j < DEF_MA_MAX-1; j++)
		{
			if (slaveType[j - 1] == DRIVE || slaveType[j-1] == HSP) //Set drive or HSP slave Cycle Synchronized Position
			{
				TargerPosition[j] += 100;
				cmdData[j].CMD = CSP;
				cmdData[j].Data1 = TargerPosition[j]; // set to move 100 pulse in one cycle time
			}
		}

		if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
			printf("Write Error \n");

		ECMUSBRead((unsigned char*)respData,sizeof(respData));
		if ((respData[0].Data2 % 65536)<20)  // If FIFO remaining < 20 then sleep 10ms
			Sleep(10);
	}
	Sleep(3000); // wait until motion stop totally
	ECMUSBRead((unsigned char*)respData,sizeof(respData));
	printf("Current Position: Slave 1 = %d, Slave 2 = %d, Slave 3 = %d.\n",respData[1].Data1,respData[2].Data1,respData[3].Data1);
#pragma endregion

#pragma region STEP 11- SET Servo Off
	ClearCmdData(cmdData);
	for (int i = 1; i < DEF_MA_MAX-1; i++)
	{
		if (slaveType[i - 1] == DRIVE || slaveType[i-1] == HSP) //Set drive or HSP slave servo off
			cmdData[i].CMD = SV_OFF;
	}
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	Sleep(100);
#pragma endregion

#pragma region STEP 13- Read Input
	ClearCmdData(cmdData);
	cmdData[0].CMD = LIO_RD; //ECM can read Local Input
	for (int i = 1; i < DEF_MA_MAX-1; i++)
	{		
		if (slaveType[i - 1] == IO) //IO slave can read Input
			cmdData[i].CMD = IO_RD;		
	}
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	Sleep(1);
	ECMUSBRead((unsigned char*)respData,sizeof(respData));

	printf("ECM Local Input Value = %x \n",respData[0].Data1);
	for (int i = 1; i < DEF_MA_MAX-1; i++)
	{
		if (slaveType[i - 1] == IO) //IO slave can read Input
			printf("Slave %d Input Value = %x \n",i,respData[i].Data1);
		if (slaveType[i - 1] == HSP) //HSP has 8 channel Inputs
			printf("Slave %d Input Value = %x \n",i,(respData[i].Data2 & 0xFF));
	}
	Sleep(100);
#pragma endregion

#pragma region STEP 14- Set Output
	ClearCmdData(cmdData);
	cmdData[0].CMD = LIO_WR; //ECM can set Local Output
	cmdData[0].Data1 = 0xF; //Output Value
	for (int i = 1; i < DEF_MA_MAX-1; i++)
	{
		if (slaveType[i - 1] == IO) //IO slave can set Output
		{
			cmdData[i].CMD = IO_WR;
			cmdData[i].Data1 = 0xFFFF; //Output Value
		}
		if (slaveType[i - 1] == HSP) 
		{
			cmdData[i].CMD = CSP;
			cmdData[i].Data2 |= 0xFF; //Output Value
		}
	}
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	Sleep(3000);
	cmdData[0].CMD = LIO_WR; //ECM can set Local Output
	cmdData[0].Data1 = 0x0; //Output Value
	for (int i = 1; i < DEF_MA_MAX-1; i++)
	{
		if (slaveType[i - 1] == IO) //only IO slave can set Output
		{
			cmdData[i].CMD = IO_WR;
			cmdData[i].Data1 = 0x0; //Output Value
		}
		if (slaveType[i - 1] == HSP) 
		{
			cmdData[i].CMD = CSP;
			cmdData[i].Data2 &= 0xFF00; //Output Value
		}
	}
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	Sleep(1000);
#pragma endregion

#pragma region SETP 15- SET_STATE to Init
	ClearCmdData(cmdData);
	cmdData[0].CMD = SET_STATE;
	cmdData[0].Data1 = STATE_INIT;
	if (!isOpen || !ECMUSBWrite((unsigned char*)cmdData,sizeof(cmdData)))
		printf("Write Error \n");
	checkResponse(0,1,1,1000);
#pragma endregion

#pragma region STEP 16- Close USB
	CloseECMUSB();
#pragma endregion
	return 0;
}

