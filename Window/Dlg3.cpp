// Dlg3.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg3.h"
#include "afxdialogex.h"
#include "DMC.h"

// CDlg3 �Ի���

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


// CDlg3 ��Ϣ�������


BOOL CDlg3::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  �ڴ���Ӷ���ĳ�ʼ��
	CString str;
	for (int i = 1; i < 41; ++i)
	{
		str.Format(_T("%d"), i);
		m_slaveAddrCombo.AddString(str);
	}
	m_slaveAddrCombo.SetCurSel(0);

	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣:  OCX ����ҳӦ���� FALSE
}


void CDlg3::OnBnClickedButton4()
{
	// ˢ�µ�ǰλ��
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	
	CString str;
	long cmdpos = d1000_get_command_pos(axis);
	str.Format(_T("%d"), cmdpos);
	m_cmdPosEdit.SetWindowText(str);

}



void CDlg3::OnBnClickedButton7()
{
	//��ԭ��
	CString str;
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	DWORD ret;
	
	ret = d1000_home_move(axis, m_highVel, m_lowVel, m_tAcc);

	if (ERR_NOERR != ret)
	{
		str.Format(_T("��ԭ�㷵��ʧ�ܣ�%d"), ret);
		MessageBox(str, _T("ʧ��"));
	}
}
