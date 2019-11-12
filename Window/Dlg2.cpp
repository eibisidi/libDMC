// Dlg2.cpp : 实现文件
//

#include "stdafx.h"
#include "Window.h"
#include "Dlg2.h"
#include "afxdialogex.h"
#include "DMC.h"

// CDlg2 对话框

IMPLEMENT_DYNAMIC(CDlg2, CDialogEx)

CDlg2::CDlg2(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDlg2::IDD, pParent)
	, m_dist(0)
	, m_maxv(0)
	, m_tAcc(0)
	, m_shape(0)
	, m_coordinate(0)
	, m_tDec(0)
{
	m_maxv = 10000;
	m_tAcc = 0.2;
	m_tDec = 0.2;

}

CDlg2::~CDlg2()
{
}

void CDlg2::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO2, m_slaveAddrCombo);
	DDX_Control(pDX, IDC_EDIT8, m_coordText);
	DDX_Control(pDX, IDC_LIST1, m_slaveDistList);
	DDX_Text(pDX, IDC_DLG2_DIST, m_dist);
	DDX_Text(pDX, IDC_EDIT1, m_maxv);
	DDX_Text(pDX, IDC_EDIT2, m_tAcc);
	DDX_Radio(pDX, IDC_RADIO1, m_shape);
	DDX_Radio(pDX, IDC_RADIO3, m_coordinate);
	DDX_Text(pDX, IDC_EDIT6, m_tDec);
	DDX_Control(pDX, IDC_EDIT5, m_cmdPosEdit);
}


BEGIN_MESSAGE_MAP(CDlg2, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON5, &CDlg2::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &CDlg2::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON4, &CDlg2::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON1, &CDlg2::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CDlg2::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CDlg2::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_RADIO3, &CDlg2::OnBnClickedRadio3)
	ON_BN_CLICKED(IDC_RADIO4, &CDlg2::OnBnClickedRadio4)
END_MESSAGE_MAP()


// CDlg2 消息处理程序


void CDlg2::OnBnClickedButton5()
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
		cstr.Format(_T("%d    %d"), iter->first, iter->second);
		m_slaveDistList.AddString(cstr);
	}
}

BOOL CDlg2::OnInitDialog()
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
	// 异常:  OCX 属性页应返回 FALSE
}


void CDlg2::OnBnClickedButton6()
{
	//删除选中的轨迹
	CString cstr;
	int cursel = m_slaveDistList.GetCurSel();
	m_slaveDistList.GetText(cursel, cstr);
	cstr = cstr.Left(cstr.Find(_T(' ')));

	int axis = _wtoi(cstr);
	m_mapSlaveDists.erase(axis);

	m_slaveDistList.DeleteString(cursel);
}


void CDlg2::OnBnClickedButton4()
{
	// 刷新当前位置
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;
	
	CString str;
	long cmdpos = d1000_get_command_pos(axis);
	str.Format(_T("%d"), cmdpos);
	m_cmdPosEdit.SetWindowText(str);

#if 0
	AllocConsole();
	freopen( "CONOUT$","w",stdout);
	printf("i的值为%d\n", cmdpos);
	FreeConsole(); 
#endif
}


void CDlg2::OnBnClickedButton1()
{	//开始运动
	CString str;
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	short axisCount = m_mapSlaveDists.size();
	short axisArray[40];
	long distArray[40];

	int i = 0;
	for (std::map<int, int>::const_iterator iter = m_mapSlaveDists.begin();
					iter != m_mapSlaveDists.end();
					++iter, ++i)
	{
		axisArray[i] = iter->first;
		distArray[i] = iter->second;
	}

	// 开始运动
	DWORD ret;
	if (0 == m_coordinate)
	{
		if (0 == m_shape)
			ret = d1000_start_t_line(axisCount, axisArray, distArray,0, m_maxv, m_tAcc);			
		else
			ret = d1000_start_s_line(axisCount, axisArray, distArray,0, m_maxv, m_tAcc);
	}
	else
	{
		if (0 == m_shape)
			ret = d1000_start_ta_line(axisCount, axisArray, distArray,0, m_maxv, m_tAcc);
		else
			ret = d1000_start_sa_line(axisCount, axisArray, distArray,0, m_maxv, m_tAcc);
	}

	if (ERR_NOERR != ret)
	{
		str.Format(_T("运动返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}



void CDlg2::OnBnClickedButton2()
{
	//减速停止
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	DWORD ret;
	ret = d1000_decel_stop(axis, m_tDec);
	CString str;

	if (ERR_NOERR != ret)
	{
		str.Format(_T("减速停止返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}



void CDlg2::OnBnClickedButton3()
{
	//急停
	UpdateData(TRUE);
	int axis = m_slaveAddrCombo.GetCurSel() + 1;

	DWORD ret;
	ret = d1000_immediate_stop(axis);
	CString str;

	if (ERR_NOERR != ret)
	{
		str.Format(_T("急停停止返回失败，%d"), ret);
		MessageBox(str, _T("失败"));
	}
}

void CDlg2::OnBnClickedRadio3()
{
	m_coordText.SetWindowText(_T("位移"));
}


void CDlg2::OnBnClickedRadio4()
{
	m_coordText.SetWindowText(_T("位置"));
}
