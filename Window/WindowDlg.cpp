
// WindowDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "Window.h"
#include "WindowDlg.h"
#include "afxdialogex.h"
#include "DMC.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CWindowDlg 对话框



CWindowDlg::CWindowDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CWindowDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CWindowDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TAB3, m_tabCtrl);
}

BEGIN_MESSAGE_MAP(CWindowDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB3, &CWindowDlg::OnTcnSelchangeTab3)
	ON_WM_DESTROY()
END_MESSAGE_MAP()


// CWindowDlg 消息处理程序

BOOL CWindowDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO:  在此添加额外的初始化代码
	m_tabCtrl.InsertItem(0, _T("单轴点位"));
	m_tabCtrl.InsertItem(1, _T("直线插补"));
	m_tabCtrl.InsertItem(2, _T("回原点"));
	m_tabCtrl.InsertItem(3, _T("IO模块"));
	m_tabCtrl.InsertItem(4, _T("Z轴拱门"));

	m_Dlg1.Create(IDD_DIALOG1, &m_tabCtrl);
	m_Dlg2.Create(IDD_DIALOG2, &m_tabCtrl);
	m_Dlg3.Create(IDD_DIALOG3, &m_tabCtrl);
	m_Dlg4.Create(IDD_DIALOG4, &m_tabCtrl);
	m_Dlg5.Create(IDD_DIALOG5, &m_tabCtrl);

	// 调整子对话框大小及位置
	CRect rc;
	m_tabCtrl.GetClientRect(&rc);
	CRect rcTabItem;
	m_tabCtrl.GetItemRect(0, rcTabItem);
	rc.top += rcTabItem.Height() + 4;
	rc.left += 4;
	rc.bottom -= 4;
	rc.right -= 4;
	m_Dlg1.MoveWindow(&rc);
	m_Dlg2.MoveWindow(&rc);
	m_Dlg3.MoveWindow(&rc);
	m_Dlg4.MoveWindow(&rc);
	m_Dlg5.MoveWindow(&rc);

	// 默认标签选中
	m_Dlg1.ShowWindow(SW_SHOW);
	m_tabCtrl.SetCurFocus(0);

	if (ERR_NOERR != d1000_board_init())
	{
		::AfxMessageBox(_T("d1000_board_init failed"), MB_OK);
		::exit(-1);
	}

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CWindowDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CWindowDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CWindowDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CWindowDlg::OnTcnSelchangeTab3(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO:  在此添加控件通知处理程序代码
	switch (m_tabCtrl.GetCurSel())
	{
	case 0:
		m_Dlg1.ShowWindow(SW_SHOW);
		m_Dlg2.ShowWindow(SW_HIDE);
		m_Dlg3.ShowWindow(SW_HIDE);
		m_Dlg4.ShowWindow(SW_HIDE);
		m_Dlg5.ShowWindow(SW_HIDE);
		m_Dlg1.SetFocus();
		break;
	case 1:
		m_Dlg2.ShowWindow(SW_SHOW);
		m_Dlg1.ShowWindow(SW_HIDE);
		m_Dlg3.ShowWindow(SW_HIDE);
		m_Dlg4.ShowWindow(SW_HIDE);
		m_Dlg5.ShowWindow(SW_HIDE);
		m_Dlg2.SetFocus();
		break;
	case 2:
		m_Dlg3.ShowWindow(SW_SHOW);
		m_Dlg4.ShowWindow(SW_HIDE);
		m_Dlg2.ShowWindow(SW_HIDE);
		m_Dlg1.ShowWindow(SW_HIDE);
		m_Dlg5.ShowWindow(SW_HIDE);
		m_Dlg3.SetFocus();
		break;
	case 3:
		m_Dlg4.ShowWindow(SW_SHOW);
		m_Dlg1.ShowWindow(SW_HIDE);
		m_Dlg2.ShowWindow(SW_HIDE);
		m_Dlg3.ShowWindow(SW_HIDE);
		m_Dlg5.ShowWindow(SW_HIDE);
		m_Dlg4.SetFocus();
		break;
	case 4:
		m_Dlg5.ShowWindow(SW_SHOW);
		m_Dlg1.ShowWindow(SW_HIDE);
		m_Dlg2.ShowWindow(SW_HIDE);
		m_Dlg3.ShowWindow(SW_HIDE);
		m_Dlg4.ShowWindow(SW_HIDE);
		m_Dlg5.SetFocus();
		break;
	default:
		break;
	}


	*pResult = 0;
}



void CWindowDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	d1000_board_close();
}
