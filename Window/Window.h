
// Window.h : PROJECT_NAME 应用程序的主头文件
//

#pragma once

#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"		// 主符号
#include "Dlg2.h"


// CWindowApp: 
// 有关此类的实现，请参阅 Window.cpp
//

class CWindowApp : public CWinApp
{
public:
	CWindowApp();

// 重写
public:
	virtual BOOL InitInstance();

// 实现

	DECLARE_MESSAGE_MAP()
	CDlg2 m_Dlg2;
	virtual int ExitInstance();
};

extern CWindowApp theApp;