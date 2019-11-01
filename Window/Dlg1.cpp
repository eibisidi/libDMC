// Dlg1.cpp : 实现文件
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg1.h"
#include "afxdialogex.h"
#include "DMC.h"

// CDlg1 对话框

IMPLEMENT_DYNAMIC(CDlg1, CDialogEx)

CDlg1::CDlg1(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDlg1::IDD, pParent)
	, m_tAcc(0)
	, m_maxv(0)
	, m_shape(0)
	, m_coordinate(0)
	, m_dist(0)
	, m_tDec(0)
{
	m_maxv = 400000;
	m_tAcc = 0.2;
	m_tDec = 0.2;
}

CDlg1::~CDlg1()
{
}

void CDlg1::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO2, m_slaveAddrCombo);
	DDX_Text(pDX, IDC_EDIT2, m_tAcc);
	DDX_Text(pDX, IDC_EDIT1, m_maxv);
	DDX_Radio(pDX, IDC_RADIO3, m_shape);
	DDX_Radio(pDX, IDC_RADIO1, m_shape);
	DDX_Radio(pDX, IDC_RADIO3, m_coordinate);
	DDX_Control(pDX, IDC_EDIT4, m_coordText);
	DDX_Text(pDX, IDC_EDIT3, m_dist);
	DDX_Control(pDX, IDC_EDIT5, m_cmdPosEdit);
	DDX_Text(pDX, IDC_EDIT6, m_tDec);
}


BEGIN_MESSAGE_MAP(CDlg1, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON1, &CDlg1::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CDlg1::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_RADIO3, &CDlg1::OnBnClickedRadio3)
	ON_BN_CLICKED(IDC_RADIO4, &CDlg1::OnBnClickedRadio4)
	ON_BN_CLICKED(IDC_BUTTON4, &CDlg1::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON3, &CDlg1::OnBnClickedButton3)
END_MESSAGE_MAP()


// CDlg1 消息处理程序


void CDlg1::OnBnClickedButton1()
{	//开始运动
	CString str;
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	// 开始运动
	DWORD ret;
	if (0 == m_coordinate)
	{
		if (0 == m_shape)
			ret = d1000_start_t_move(axis, m_dist, 0, m_maxv, m_tAcc);			
		else
			ret = d1000_start_s_move(axis, m_dist, 0, m_maxv, m_tAcc);
	}
	else
	{
		if (0 == m_shape)
			ret = d1000_start_ta_move(axis, m_dist, 0, m_maxv, m_tAcc);
		else
			ret = d1000_start_sa_move(axis, m_dist, 0, m_maxv, m_tAcc);
	}

	if (ERR_NOERR != ret)
	{
		str.Format(_T("运动返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}


void CDlg1::OnBnClickedButton2()
{
	//减速停止
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	DWORD ret;
	ret = d1000_decel_stop(axis, m_tDec);
	CString str;

	if (ERR_NOERR != ret)
	{
		str.Format(_T("减速停止返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}


BOOL CDlg1::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	CString str;
	for (int i = 1; i < 41; ++i)
	{
		str.Format(_T("%d"), i);
		m_slaveAddrCombo.AddString(str);
	}
	m_slaveAddrCombo.SetCurSel(0);

	m_coordText.SetWindowTextW(_T("位移"));


	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常:  OCX 属性页应返回 FALSE
}


void CDlg1::OnBnClickedRadio3()
{
	m_coordText.SetWindowText(_T("位移"));
}


void CDlg1::OnBnClickedRadio4()
{
	m_coordText.SetWindowText(_T("位置"));
}


void CDlg1::OnBnClickedButton4()
{
	//刷新命令位置
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	
	CString str;
	long cmdpos = d1000_get_command_pos(axis);
	str.Format(_T("%d"), cmdpos);
	m_cmdPosEdit.SetWindowText(str);
}


void CDlg1::OnBnClickedButton3()
{
	//急停
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	DWORD ret;
	ret = d1000_immediate_stop(axis);
	CString str;

	if (ERR_NOERR != ret)
	{
		str.Format(_T("急停停止返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}
