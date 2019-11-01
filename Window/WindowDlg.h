
// WindowDlg.h : 头文件
//

#pragma once
#include "afxcmn.h"
#include "Dlg1.h"
#include "Dlg2.h"
#include "Dlg3.h"
#include "Dlg4.h"


// CWindowDlg 对话框
class CWindowDlg : public CDialogEx
{
// 构造
public:
	CWindowDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_WINDOW_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnTcnSelchangeTab3(NMHDR *pNMHDR, LRESULT *pResult);
	CTabCtrl m_tabCtrl;
	CDlg1 m_Dlg1;
	CDlg2 m_Dlg2;
	afx_msg void OnDestroy();
	CDlg3 m_Dlg3;
	CDlg4 m_Dlg4;
};
