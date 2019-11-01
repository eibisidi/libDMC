#pragma once
#include "afxwin.h"
#include <map>


// CDlg2 对话框

class CDlg2 : public CDialogEx
{
	DECLARE_DYNAMIC(CDlg2)

public:
	CDlg2(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CDlg2();

// 对话框数据
	enum { IDD = IDD_DIALOG2 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton5();
	CComboBox m_slaveAddrCombo;

	virtual BOOL OnInitDialog();
	CEdit m_coordText;

	std::map<int, int> m_mapSlaveDists;
	CListBox m_slaveDistList;
	int m_dist;
	afx_msg void OnBnClickedButton6();


	long m_maxv;

	double m_tAcc;
	int m_shape;
	int m_coordinate;
	afx_msg void OnBnClickedButton4();
	double m_tDec;
	CEdit m_cmdPosEdit;
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedRadio3();
	afx_msg void OnBnClickedRadio4();
};
