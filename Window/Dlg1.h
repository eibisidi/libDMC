#pragma once
#include "afxwin.h"


// CDlg1 �Ի���

class CDlg1 : public CDialogEx
{
	DECLARE_DYNAMIC(CDlg1)

public:
	CDlg1(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CDlg1();

// �Ի�������
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	CComboBox m_slaveAddrCombo;
	virtual BOOL OnInitDialog();

	double m_tAcc;
	long m_maxv;
	// //�ٶ��������� 0 T 1 S
	int m_shape;
	// //����ϵ���� 0��� 1����
	int m_coordinate;
	afx_msg void OnBnClickedRadio3();
	CEdit m_coordText;
	afx_msg void OnBnClickedRadio4();
	// //λ�� λ��
	long m_dist;
	CEdit m_cmdPosEdit;
	double m_tDec;
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton3();
};
