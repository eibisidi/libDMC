// Dlg1.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg1.h"
#include "afxdialogex.h"
#include "DMC.h"

// CDlg1 �Ի���

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


// CDlg1 ��Ϣ�������


void CDlg1::OnBnClickedButton1()
{	//��ʼ�˶�
	CString str;
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	// ��ʼ�˶�
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
		str.Format(_T("�˶�����ʧ�ܣ�%d"), ret);
		MessageBox(str, _T("ʧ��"));
	}
}


void CDlg1::OnBnClickedButton2()
{
	//����ֹͣ
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	DWORD ret;
	ret = d1000_decel_stop(axis, m_tDec);
	CString str;

	if (ERR_NOERR != ret)
	{
		str.Format(_T("����ֹͣ����ʧ�ܣ�%d"), ret);
		MessageBox(str, _T("ʧ��"));
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

	m_coordText.SetWindowTextW(_T("λ��"));


	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣:  OCX ����ҳӦ���� FALSE
}


void CDlg1::OnBnClickedRadio3()
{
	m_coordText.SetWindowText(_T("λ��"));
}


void CDlg1::OnBnClickedRadio4()
{
	m_coordText.SetWindowText(_T("λ��"));
}


void CDlg1::OnBnClickedButton4()
{
	//ˢ������λ��
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	
	CString str;
	long cmdpos = d1000_get_command_pos(axis);
	str.Format(_T("%d"), cmdpos);
	m_cmdPosEdit.SetWindowText(str);
}


void CDlg1::OnBnClickedButton3()
{
	//��ͣ
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	DWORD ret;
	ret = d1000_immediate_stop(axis);
	CString str;

	if (ERR_NOERR != ret)
	{
		str.Format(_T("��ֹͣͣ����ʧ�ܣ�%d"), ret);
		MessageBox(str, _T("ʧ��"));
	}
}
