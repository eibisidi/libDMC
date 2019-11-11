// Dlg5.cpp : 实现文件
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg5.h"
#include "afxdialogex.h"
#include "DMC.h"


// CDlg5 对话框

IMPLEMENT_DYNAMIC(CDlg5, CDialogEx)

CDlg5::CDlg5(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDlg5::IDD, pParent)
	, m_maxv(0)
	, m_tAcc(0)
	, m_coordinate(0)
	, m_hu(0)
	, m_hd(0)
	, m_hh(0)
	, m_tDec(0)
	, m_dist(0)
{
	m_maxv = 100000;
	m_tAcc = 0.2;
	m_tDec = 0.2;
	m_hu   = 10000;
	m_hh   = 100000;
	m_hd   = 10000;
}

CDlg5::~CDlg5()
{
}

void CDlg5::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, m_maxv);
	DDX_Text(pDX, IDC_EDIT2, m_tAcc);
	DDX_Radio(pDX, IDC_RADIO3, m_coordinate);
	DDX_Control(pDX, IDC_COMBO2, m_slaveAddrCombo);
	DDX_Control(pDX, IDC_EDIT8, m_coordText);
	DDX_Control(pDX, IDC_COMBO3, m_ZAddrCombo);
	DDX_Text(pDX, IDC_EDIT3, m_hu);
	DDX_Text(pDX, IDC_EDIT7, m_hd);
	DDX_Text(pDX, IDC_EDIT9, m_hh);
	DDX_Control(pDX, IDC_EDIT5, m_cmdPosEdit);
	DDX_Control(pDX, IDC_LIST1, m_slaveDistList);
	DDX_Text(pDX, IDC_EDIT6, m_tDec);
	DDX_Text(pDX, IDC_DLG2_DIST, m_dist);
	DDX_Text(pDX, IDC_DLG2_DIST, m_dist);
}


BEGIN_MESSAGE_MAP(CDlg5, CDialogEx)
	ON_BN_CLICKED(IDC_RADIO3, &CDlg5::OnBnClickedRadio3)
	ON_BN_CLICKED(IDC_RADIO4, &CDlg5::OnBnClickedRadio4)
	ON_BN_CLICKED(IDC_BUTTON5, &CDlg5::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &CDlg5::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON1, &CDlg5::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON4, &CDlg5::OnBnClickedButton4)
END_MESSAGE_MAP()


// CDlg5 消息处理程序


BOOL CDlg5::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	CString str;
	for (int i = 1; i < 41; ++i)
	{
		str.Format(_T("%d"), i);
		m_slaveAddrCombo.AddString(str);
	}
	m_slaveAddrCombo.SetCurSel(0);

	m_coordText.SetWindowTextW(_T("位移"));

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常:	OCX 属性页应返回 FALSE
}



void CDlg5::OnBnClickedRadio3()
{
	m_coordText.SetWindowText(_T("位移"));
}


void CDlg5::OnBnClickedRadio4()
{
	m_coordText.SetWindowText(_T("位置"));
}

void CDlg5::UpdateZComboBox()
{
	m_ZAddrCombo.ResetContent();
	for (std::map<int, int>::const_iterator iter = m_mapSlaveDists.begin();
				iter != m_mapSlaveDists.end();
				++iter)
	{
		CString cstr;
		cstr.Format(_T("%d"), iter->first);
		m_ZAddrCombo.AddString(cstr);
	}
	m_ZAddrCombo.SetCurSel(0);
}

void CDlg5::OnBnClickedButton5()
{
	//增加从站轨迹
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	m_mapSlaveDists[axis] = m_dist;
	
	m_slaveDistList.ResetContent();
	for (std::map<int, int>::const_iterator iter = m_mapSlaveDists.begin();
				iter != m_mapSlaveDists.end();
				++iter)
	{
		CString cstr;
		cstr.Format(_T("%d	  %d"), iter->first, iter->second);
		m_slaveDistList.AddString(cstr);
	}

	UpdateZComboBox();
}

void CDlg5::OnBnClickedButton6()
{
	//删除选中的轨迹
	CString cstr;
	int cursel = m_slaveDistList.GetCurSel();
	m_slaveDistList.GetText(cursel, cstr);
	cstr = cstr.Left(cstr.Find(_T(' ')));

	int axis = _wtoi(cstr);
	m_mapSlaveDists.erase(axis);

	m_slaveDistList.DeleteString(cursel);

	UpdateZComboBox();
}

void CDlg5::OnBnClickedButton1()
{	//开始运动
	CString str;
	UpdateData(TRUE);

	m_ZAddrCombo.GetWindowTextW(str);
	int zAxis = _wtoi(str);

	short axisCount = m_mapSlaveDists.size();
	short axisArray[40];
	long distArray[40];

	int i = 1;
	for (std::map<int, int>::const_iterator iter = m_mapSlaveDists.begin();
					iter != m_mapSlaveDists.end();
					++iter)
	{
		if (zAxis == iter->first)
		{
			axisArray[0] = zAxis;
			distArray[0] = iter->second;
		}
		else
		{
			axisArray[i] = iter->first;
			distArray[i] = iter->second;
			++i;
		}
	}

	// 开始运动
	DWORD ret;
	if (0 == m_coordinate)
	{
		ret = d1000_start_t_archl(axisCount, axisArray, distArray, m_maxv, m_tAcc, m_hh, m_hu, m_hd);			
	}
	else
	{
		ret = d1000_start_t_archl(axisCount, axisArray, distArray, m_maxv, m_tAcc, m_hh, m_hu, m_hd);
	}

	if (ERR_NOERR != ret)
	{
		str.Format(_T("运动返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}



void CDlg5::OnBnClickedButton4()
{
	// 刷新当前位置
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	
	CString str;
	long cmdpos = d1000_get_command_pos(axis);
	str.Format(_T("%d"), cmdpos);
	m_cmdPosEdit.SetWindowText(str);
}

