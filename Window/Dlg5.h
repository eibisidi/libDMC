#pragma once
#include "afxwin.h"


// CDlg5 对话框

class CDlg5 : public CDialogEx
{
	DECLARE_DYNAMIC(CDlg5)

public:
	CDlg5(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CDlg5();

// 对话框数据
	enum { IDD = IDD_DIALOG5 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	long m_maxv;
	double m_tAcc;
	int m_coordinate;
	CComboBox m_slaveAddrCombo;
	CEdit m_coordText;
	CComboBox m_ZAddrCombo;
	int m_hu;
	int m_hd;
	int m_hh;
	CEdit m_cmdPosEdit;
	CListBox m_slaveDistList;
	virtual BOOL OnInitDialog();
	double m_tDec;
	int m_dist;
	std::map<int, int> m_mapSlaveDists;

	void UpdateZComboBox();


	afx_msg void OnBnClickedRadio3();
	afx_msg void OnBnClickedRadio4();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton6();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
};
