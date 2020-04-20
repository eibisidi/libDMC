// Dlg4.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg4.h"
#include "afxdialogex.h"
#include "DMC.h"
#include <sstream>

// CDlg4 �Ի���

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


// CDlg4 ��Ϣ�������


void CDlg4::OnBnClickedButton1()
{
	//��ȡ����
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
		str.Format(_T("��ȡ���뷵��ʧ�ܣ�%d"), ret);
		MessageBox(str, _T("ʧ��"));
		return;
	}
#endif

	str.Format(_T("0x%04x"), bits);
	m_inputEdit.SetWindowText(str);
}


void CDlg4::OnBnClickedButton8()
{
	//�������
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
		str.Format(_T("�����������ʧ�ܣ�%d"), ret);
		MessageBox(str, _T("ʧ��"));
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
	// �쳣:  OCX ����ҳӦ���� FALSE
}
