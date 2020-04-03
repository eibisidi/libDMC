// Dlg4.cpp : 实现文件
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg4.h"
#include "afxdialogex.h"
#include "DMC.h"
#include <sstream>

// CDlg4 对话框

IMPLEMENT_DYNAMIC(CDlg4, CDialogEx)

CDlg4::CDlg4(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDlg4::IDD, pParent)
	, m_outputString(_T(""))
{

}

CDlg4::~CDlg4()
{
}

void CDlg4::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO2, m_slaveAddrCombo);
	DDX_Control(pDX, IDC_EDIT1, m_inputEdit);
	DDX_Text(pDX, IDC_EDIT2, m_outputString);
}


BEGIN_MESSAGE_MAP(CDlg4, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON1, &CDlg4::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON8, &CDlg4::OnBnClickedButton8)
END_MESSAGE_MAP()


// CDlg4 消息处理程序


void CDlg4::OnBnClickedButton1()
{
	//读取输入
	CString str;
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	DWORD ret;

	unsigned int bits = 0;

#if 0
	todo 
	ret = d1000_in_bit(axis, &bits);
	if (ERR_NOERR != ret)
	{
		str.Format(_T("读取输入返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
		return;
	}
#endif

	str.Format(_T("0x%04x"), bits);
	m_inputEdit.SetWindowText(str);
}


void CDlg4::OnBnClickedButton8()
{
	//设置输出
	CString str;
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	DWORD ret;

	std::string hexString = CT2A(m_outputString.GetString());

	unsigned int bits = strtoul(hexString.c_str(), NULL, 16);

#if 0
	//todo
	ret = d1000_out_bit(axis, bits);
	if (ERR_NOERR != ret)
	{
		str.Format(_T("设置输出返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
#endif
}


BOOL CDlg4::OnInitDialog()
{
	CDialogEx::OnInitDialog();

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
