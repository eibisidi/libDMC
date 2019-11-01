#pragma once
#include "afxwin.h"


// CDlg1 对话框

class CDlg1 : public CDialogEx
{
	DECLARE_DYNAMIC(CDlg1)

public:
	CDlg1(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CDlg1();

// 对话框数据
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	CComboBox m_slaveAddrCombo;
	virtual BOOL OnInitDialog();

	double m_tAcc;
	long m_maxv;
	// //速度曲线类型 0 T 1 S
	int m_shape;
	// //坐标系类型 0相对 1绝对
	int m_coordinate;
	afx_msg void OnBnClickedRadio3();
	CEdit m_coordText;
	afx_msg void OnBnClickedRadio4();
	// //位移 位置
	long m_dist;
	CEdit m_cmdPosEdit;
	double m_tDec;
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton3();
};
