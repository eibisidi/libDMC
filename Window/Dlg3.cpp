// Dlg3.cpp : 实现文件
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg3.h"
#include "afxdialogex.h"
#include "DMC.h"

// CDlg3 对话框

IMPLEMENT_DYNAMIC(CDlg3, CDialogEx)

CDlg3::CDlg3(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDlg3::IDD, pParent)
	, m_highVel(0)
	, m_lowVel(0)
	, m_tAcc(0)
{
	m_highVel = 50000;
	m_lowVel  = 5000;
	m_tAcc	  = 0.2;
}

CDlg3::~CDlg3()
{
}

void CDlg3::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO2, m_slaveAddrCombo);
	DDX_Text(pDX, IDC_EDIT1, m_highVel);
	DDX_Text(pDX, IDC_EDIT2, m_lowVel);
	DDX_Text(pDX, IDC_EDIT3, m_tAcc);
	DDX_Control(pDX, IDC_EDIT5, m_cmdPosEdit);
}


BEGIN_MESSAGE_MAP(CDlg3, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON4, &CDlg3::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON7, &CDlg3::OnBnClickedButton7)
END_MESSAGE_MAP()


// CDlg3 消息处理程序


BOOL CDlg3::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	CString str;
	for (int i = 1; i < 41; ++i)
	{
		str.Format(_T("%d"), i);
		m_slaveAddrCombo.AddString(str);
	}
	m_slaveAddrCombo.SetCurSel(0);

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常:  OCX 属性页应返回 FALSE
}


void CDlg3::OnBnClickedButton4()
{
	// 刷新当前位置
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	
	CString str;
	long cmdpos = d1000_get_command_pos(axis);
	str.Format(_T("%d"), cmdpos);
	m_cmdPosEdit.SetWindowText(str);

}



void CDlg3::OnBnClickedButton7()
{
	//回原点
	CString str;
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	DWORD ret;
	
	ret = d1000_home_move(axis, m_highVel, m_lowVel, m_tAcc);

	if (ERR_NOERR != ret)
	{
		str.Format(_T("回原点返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}
