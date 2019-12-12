#pragma once
#include "afxwin.h"


// CDlg3 对话框

class CDlg3 : public CDialogEx
{
	DECLARE_DYNAMIC(CDlg3)

public:
	CDlg3(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CDlg3();

// 对话框数据
	enum { IDD = IDD_DIALOG3 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CComboBox m_slaveAddrCombo;
	long m_highVel;
	long m_lowVel;
	long m_acc;
	CEdit m_cmdPosEdit;
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton7();
};
