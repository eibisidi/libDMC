#pragma once
#include "afxwin.h"


// CDlg4 对话框

class CDlg4 : public CDialogEx
{
	DECLARE_DYNAMIC(CDlg4)

public:
	CDlg4(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CDlg4();

// 对话框数据
	enum { IDD = IDD_DIALOG4 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CComboBox m_slaveAddrCombo;
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton8();
	virtual BOOL OnInitDialog();
	CEdit m_inputEdit;
	CString m_outputString;
};
