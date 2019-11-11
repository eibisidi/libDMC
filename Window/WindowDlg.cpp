
// WindowDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "Window.h"
#include "WindowDlg.h"
#include "afxdialogex.h"
#include "DMC.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
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


// CWindowDlg �Ի���



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


// CWindowDlg ��Ϣ�������

BOOL CWindowDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
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

	// ���ô˶Ի����ͼ�ꡣ  ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO:  �ڴ���Ӷ���ĳ�ʼ������
	m_tabCtrl.InsertItem(0, _T("�����λ"));
	m_tabCtrl.InsertItem(1, _T("ֱ�߲岹"));
	m_tabCtrl.InsertItem(2, _T("��ԭ��"));
	m_tabCtrl.InsertItem(3, _T("IOģ��"));
	m_tabCtrl.InsertItem(4, _T("Z�Ṱ��"));

	m_Dlg1.Create(IDD_DIALOG1, &m_tabCtrl);
	m_Dlg2.Create(IDD_DIALOG2, &m_tabCtrl);
	m_Dlg3.Create(IDD_DIALOG3, &m_tabCtrl);
	m_Dlg4.Create(IDD_DIALOG4, &m_tabCtrl);
	m_Dlg5.Create(IDD_DIALOG5, &m_tabCtrl);

	// �����ӶԻ����С��λ��
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

	// Ĭ�ϱ�ǩѡ��
	m_Dlg1.ShowWindow(SW_SHOW);
	m_tabCtrl.SetCurFocus(0);

	if (ERR_NOERR != d1000_board_init())
	{
		::AfxMessageBox(_T("d1000_board_init failed"), MB_OK);
		::exit(-1);
	}

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ  ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CWindowDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR CWindowDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CWindowDlg::OnTcnSelchangeTab3(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO:  �ڴ���ӿؼ�֪ͨ����������
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
