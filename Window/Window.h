
// Window.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������
#include "Dlg2.h"


// CWindowApp: 
// �йش����ʵ�֣������ Window.cpp
//

class CWindowApp : public CWinApp
{
public:
	CWindowApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
	CDlg2 m_Dlg2;
	virtual int ExitInstance();
};

extern CWindowApp theApp;