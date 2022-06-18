// openvsdDlg.cpp: 实现文件
//
#include "pch.h"
#include "framework.h"
#include "openvsd.h"
#include "openvsdDlg.h"
#include "afxdialogex.h"
#include <stdio.h>
#include <iostream>
#include <Windows.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CopenvsdDlg 对话框



CopenvsdDlg::CopenvsdDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_OPENVSD_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CopenvsdDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CopenvsdDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CopenvsdDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CopenvsdDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CopenvsdDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CopenvsdDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CopenvsdDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &CopenvsdDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &CopenvsdDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &CopenvsdDlg::OnBnClickedButton8)
END_MESSAGE_MAP()


// CopenvsdDlg 消息处理程序

BOOL CopenvsdDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
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

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CopenvsdDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CopenvsdDlg::OnPaint()
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
HCURSOR CopenvsdDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CopenvsdDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
		// TODO: Add your control notification handler code here
	Vehicle dof1;
	//body 1
	Vector3x rotiner(1, 1, 1);
	Vector3x r01(0, 0, 2);
	Vector3x rd0(0, 0, 0);
	Vector3x rdd0(0, 0, 0);
	EulerAngle t0(0, 0, 0);
	EulerAngle td0(0, 0, 0);
	EulerAngle tdd0(0, 0, 0);
	RigidBody body1(1, "dof1 body", 0, 1, 1.0, rotiner, r01, rd0, rdd0, t0, td0, tdd0);
	ResObserver obs1(1, "body1", 1);
	body1.CreateObserverHdr(obs1);
	dof1.CreateInertiaElemHdr(body1);
	//body 2
	Vector3x r02(0, 0, 1);
	RigidBody body2(2, "dof2 body", 0, 1, 1.0, rotiner, r02, rd0, rdd0, t0, td0, tdd0);
	ResObserver obs2(1, "body2", 1);
	body2.CreateObserverHdr(obs2);
	dof1.CreateInertiaElemHdr(body2);

	LinearTSDA ts(0, "linear spring", dof1.Inertia_array[1].getPointer(), 0, dof1.Inertia_array[2].getPointer(), 0, 50, 4, 1);
	LinearTSDA ts2(1, "linear spring2", dof1.Inertia_array[2].getPointer(), 0, dof1.Inertia_array[0].getPointer(), 0, 200, 20, 1);

	dof1.CreateForceElemHdr(ts);
	dof1.CreateForceElemHdr(ts2);

	dof1.RK4integration(0.001, 5);
	//dof1.Newmark(0.01, 30, 0.25, 0.5);
}


void CopenvsdDlg::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码
	WRcontForce wf(2.1e11, 2.1e11, 0.24, 0.24, 0.4);

	Matrix mx0; mx0.LoadAscii();
	Matrix mx1; mx1.LoadAscii();
	Matrix mx2; mx2.LoadAscii();
	mx2.InverseSort();
	Vector3x o(0, 0, 0);
	EulerAngle a(0, 0, 0);
	Track tk(mx0, mx1, mx2, o, a, 1.506, 0.025, 10.0, 3, 3, 3);

	Matrix mxlwheel;
	mxlwheel.LoadAscii();
	Matrix mxrwheel;
	mxrwheel.LoadAscii();
	mxrwheel.InverseSort();

	Vector3x rotiner(950, 950, 120);
	Vector3x r0(0, 0, 0.46);
	double v = 0;
	Vector3x rd0(v, 0, 0);
	Vector3x rdd0(0, 0, 0);
	EulerAngle t0(0, 0, 0);
	EulerAngle td0(0, 0, v / 0.46);
	EulerAngle tdd0(0, 0, 0);
	RWheelSet ws1(1, "wheelset1", 1500, rotiner, r0, rd0, rdd0, t0, td0, tdd0, 0.46, 0.46, 1.500, mxlwheel, mxrwheel, 0, 0, tk, 3, 3, 3, wf);
	//
	ws1.InitWheelRailContactGeometry(-0.0071, 0.0071, 0.0001, tk);
	ws1.getRollingRadiusDiff().SaveAscii();
	ws1.getConsEquContactPnts().SaveAscii();
}


void CopenvsdDlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	Matrix mx;
	mx.LoadAscii();
	WheelGeometry wg(0.4575, mx, 3, 5, 5, 3, 5, 5);
	wg.get_Profile().getPoints().SaveAscii();
	wg.get_Profile_1st().getPoints().SaveAscii();
	wg.get_Profile_2nd().getPoints().SaveAscii();
	//wg.getCurvatureMat().SaveAscii();
}


void CopenvsdDlg::OnBnClickedButton4()
{
	// TODO: 在此添加控件通知处理程序代码
	WRcontForce wf(2.1e11, 2.1e11, 0.24, 0.24, 0.3);
	Matrix mx0; mx0.LoadAscii();
	Matrix mx1; mx1.LoadAscii();
	Matrix mx2; mx2.LoadAscii();
	mx2.InverseSort();
	Vector3x o(0, 0, 0);
	EulerAngle a(0, 0, 0);
	Track tk(mx0, mx1, mx2, o, a, 1.506, 0.025, 10.0, 3, 3, 3);

	Matrix mxlwheel; mxlwheel.LoadAscii();
	Matrix mxrwheel; mxrwheel.LoadAscii();
	mxrwheel.InverseSort();
	Vector3x rotiner(950, 120, 950);
	Vector3x r0(0, 0.000, 0.46);
	Vector3x rd0(0, 0, 0);
	Vector3x rdd0(0, 0, 0);
	EulerAngle t0(0, 0, 0);
	EulerAngle td0(0, 0, 0);
	EulerAngle tdd0(0, 0, 0);
	//20210406
	RWheelSet ws1(1, "wheelset1", 1000, rotiner, r0, rd0, rdd0, t0, td0, tdd0, 0.46, 0.46, 1.5, mxlwheel, mxrwheel, 0, 0, tk, 2, 2, 2, wf);
	ResObserver obs1(1, "wsres1", 1);
	ws1.CreateObserverHdr(obs1);
	Vehicle singlewheel;
	singlewheel.CreateInertiaElemHdr(ws1);
	//
	singlewheel.InitBalancedPosition(0.0002, 5.0);
	singlewheel.UpdateStaticConfiguration();
	double V = 5;
	singlewheel.SetRunSpeed(V);
	singlewheel.UpdateVelocityConfiguration();
	singlewheel.RK4(0.0002, 1.0);
}


void CopenvsdDlg::OnBnClickedButton5()
{
	// TODO: 在此添加控件通知处理程序代码
	WRcontForce wf(2.1e11, 2.1e11, 0.24, 0.24, 0.4);
	Matrix mx0; mx0.LoadAscii();
	Matrix mx1; mx1.LoadAscii();
	Matrix mx2; mx2.LoadAscii();
	mx2.InverseSort();
	Vector3x o(0, 0, 0);
	EulerAngle a(0, 0, 0);
	Track tk(mx0, mx1, mx2, o, a, 1.506, 0.025, 10.0, 3, 3, 3);
	Matrix mxlwheel;
	mxlwheel.LoadAscii();
	Matrix mxrwheel;
	mxrwheel.LoadAscii();
	mxrwheel.InverseSort();

	Vector3x rotiner(950, 950, 120);
	Vector3x r0(0, 0, 0.46);
	double v = 0;
	Vector3x rd0(v, 0, 0);
	Vector3x rdd0(0, 0, 0);
	EulerAngle t0(0, 0, 0);
	EulerAngle td0(0, 0, v / 0.46);
	EulerAngle tdd0(0, 0, 0);
	//RWheelSet ws1(1, "wheelset1", 1500, rotiner, r0, rd0, rdd0, t0, td0, tdd0, 0.46, 0.46, 1.500, mxlwheel, mxrwheel, 0, tk, 10, 20, 40, wf);
	RWheelSet ws1(1, "wheelset1", 1500, rotiner, r0, rd0, rdd0, t0, td0, tdd0, 0.46, 0.46, 1.500, mxlwheel, mxrwheel, 0, 0, tk, 2, 2, 2, wf);
	//
	ws1.InitWheelRailContactGeometry(-0.0071, 0.0071, 0.0001, tk);
	ws1.getRollingRadiusDiff().SaveAscii();
	ws1.getConsEquContactPnts().SaveAscii();
}


void CopenvsdDlg::OnBnClickedButton6()
{
	// TODO: 在此添加控件通知处理程序代码
	Vehicle vh1;
	//
	double l_radius = 0.46; double r_radius = 0.46;
	double axle_base = 2.5; double bogie_base = 19.0;
	double profile_base = 1.5; double suspension_base = 2.0; double ss_suspension_base = 2.0;
	double ps_high = 1.0; double ps_low = 0.46;
	double ss_high = 1.150; double ss_low = 0.525;
	//
	double wheelset_mass = 1813; double ws_cg_height = 0.46;
	Vector3x wheelset_rotiner(1120, 112, 1120);
	//
	double bg_mass = 2615; double bg_cg_height = 0.6;
	Vector3x bg_rotiner(1722, 1476, 3067);
	//
	double cb_mass = 32000; double cb_cg_height = 1.8;
	Vector3x cb_rotiner(56800, 1970000, 1970000);

	//
	//0 WR force
	WRcontForce wf(2.1e11, 2.1e11, 0.24, 0.24, 0.4);
	//1 Track
	Matrix mx0; mx0.LoadAscii();
	Matrix mx1; mx1.LoadAscii();
	Matrix mx2; mx2.LoadAscii();
	mx2.InverseSort();
	Vector3x o(0, 0, 0);
	EulerAngle a(0, 0, 0);
	Track tk(mx0, mx1, mx2, o, a, 1.506, 0.025, 10.0, 3, 3, 3);
	//2 Wheelset
	Matrix mxlwheel; mxlwheel.LoadAscii();
	Matrix mxrwheel; mxrwheel.LoadAscii();
	mxrwheel.InverseSort();

	////2-1 ws1_
	Vector3x ws1_r0(0, 0, ws_cg_height); 
	Vector3x ws1_rd0(0, 0, 0);
	Vector3x ws1_rdd0(0, 0, 0);
	EulerAngle ws1_t0(0, 0, 0); 
	EulerAngle ws1_td0(0, 0, 0);
	EulerAngle ws1_tdd0(0, 0, 0);
	RWheelSet ws1(1, "ws1", wheelset_mass, wheelset_rotiner, ws1_r0, ws1_rd0, ws1_rdd0, ws1_t0, ws1_td0, ws1_tdd0, l_radius, r_radius, profile_base, mxlwheel, mxrwheel, 0, 0, tk, 2, 2, 2, wf);
	//
	Vector3x ws1_ps_r1(0.0, suspension_base / 2.0, 0.0);
	EulerAngle ws1_ps_ea1(0.0, 0.0, 0.0);
	Marker ws1_ps_L(3, ws1_ps_r1, ws1_ps_ea1); // marker index 3
	Vector3x ws1_ps_r2(0.0, (-1.0) * suspension_base / 2.0, 0.0);
	EulerAngle ws1_ps_ea2(0.0, 0.0, 0.0);
	Marker ws1_ps_R(4, ws1_ps_r2, ws1_ps_ea2); // marker index 4
	ws1.addMarker(ws1_ps_L); // marker index 3
	ws1.addMarker(ws1_ps_R); // marker index 4
	//
	ResObserver wsobs1(1, "ws1", 1);
	ws1.CreateObserverHdr(wsobs1);
	vh1.CreateInertiaElemHdr(ws1);

	////2-2 ws2_
	Vector3x ws2_r0(axle_base, 0.000, ws_cg_height); 
	Vector3x ws2_rd0(0, 0, 0);
	Vector3x ws2_rdd0(0, 0, 0);
	EulerAngle ws2_t0(0, 0, 0); 
	EulerAngle ws2_td0(0, 0, 0); 
	EulerAngle ws2_tdd0(0, 0, 0);
	RWheelSet ws2(2, "ws2", wheelset_mass, wheelset_rotiner, ws2_r0, ws2_rd0, ws2_rdd0, ws2_t0, ws2_td0, ws2_tdd0, l_radius, r_radius, profile_base, mxlwheel, mxrwheel, 0, 0, tk, 2, 2, 2, wf);
	//
	Vector3x ws2_ps_r1(0.0, suspension_base / 2.0, 0.0); 
	EulerAngle ws2_ps_ea1(0.0, 0.0, 0.0);
	Marker ws2_ps_L(3, ws2_ps_r1, ws2_ps_ea1);
	Vector3x ws2_ps_r2(0.0, (-1.0) * suspension_base / 2.0, 0.0);
	EulerAngle ws2_ps_ea2(0.0, 0.0, 0.0);
	Marker ws2_ps_R(4, ws2_ps_r2, ws2_ps_ea2);
	ws2.addMarker(ws2_ps_L); // marker index 3
	ws2.addMarker(ws2_ps_R); // marker index 4
	//
	ResObserver wsobs2(1, "ws2", 1);
	ws2.CreateObserverHdr(wsobs2);
	vh1.CreateInertiaElemHdr(ws2);

	////2-3 ws3_
	Vector3x ws3_r0(bogie_base, 0.000, ws_cg_height); 
	Vector3x ws3_rd0(0, 0, 0); 
	Vector3x ws3_rdd0(0, 0, 0);
	EulerAngle ws3_t0(0, 0, 0); 
	EulerAngle ws3_td0(0, 0, 0); 
	EulerAngle ws3_tdd0(0, 0, 0);
	RWheelSet ws3(3, "ws3", wheelset_mass, wheelset_rotiner, ws3_r0, ws3_rd0, ws3_rdd0, ws3_t0, ws3_td0, ws3_tdd0, l_radius, r_radius, profile_base, mxlwheel, mxrwheel, 0, 0, tk, 2, 2, 2, wf);
	//
	Vector3x ws3_ps_r1(0.0, suspension_base / 2.0, 0.0); 
	EulerAngle ws3_ps_ea1(0.0, 0.0, 0.0);
	Marker ws3_ps_L(3, ws3_ps_r1, ws3_ps_ea1);
	Vector3x ws3_ps_r2(0.0, (-1.0) * suspension_base / 2.0, 0.0); 
	EulerAngle ws3_ps_ea2(0.0, 0.0, 0.0);
	Marker ws3_ps_R(4, ws3_ps_r2, ws3_ps_ea2);
	ws3.addMarker(ws3_ps_L); // marker index 3
	ws3.addMarker(ws3_ps_R); // marker index 4
	//
	ResObserver wsobs3(1, "ws3", 1);
	ws3.CreateObserverHdr(wsobs3);
	vh1.CreateInertiaElemHdr(ws3);

	////2-4 ws4_
	Vector3x ws4_r0(axle_base + bogie_base, 0.000, ws_cg_height); 
	Vector3x ws4_rd0(0, 0, 0); 
	Vector3x ws4_rdd0(0, 0, 0);
	EulerAngle ws4_t0(0, 0, 0); 
	EulerAngle ws4_td0(0, 0, 0);
	EulerAngle ws4_tdd0(0, 0, 0);
	RWheelSet ws4(4, "ws4", wheelset_mass, wheelset_rotiner, ws4_r0, ws4_rd0, ws4_rdd0, ws4_t0, ws4_td0, ws4_tdd0, l_radius, r_radius, profile_base, mxlwheel, mxrwheel, 0, 0, tk, 2, 2, 2, wf);
	//
	Vector3x ws4_ps_r1(0.0, suspension_base / 2.0, 0.0);
	EulerAngle ws4_ps_ea1(0.0, 0.0, 0.0);
	Marker ws4_ps_L(3, ws4_ps_r1, ws4_ps_ea1);
	Vector3x ws4_ps_r2(0.0, (-1.0) * suspension_base / 2.0, 0.0); 
	EulerAngle ws4_ps_ea2(0.0, 0.0, 0.0);
	Marker ws4_ps_R(4, ws4_ps_r2, ws4_ps_ea2);
	ws4.addMarker(ws4_ps_L); // marker index 3
	ws4.addMarker(ws4_ps_R); // marker index 4
	//
	ResObserver wsobs4(1, "ws4", 1);
	ws4.CreateObserverHdr(wsobs4);
	vh1.CreateInertiaElemHdr(ws4);

	//3 bogie
	////3-1 bg1
	Vector3x bg1_r0(axle_base / 2.0, 0.000, bg_cg_height); 
	Vector3x bg1_rd0(0, 0, 0); 
	Vector3x bg1_rdd0(0, 0, 0);
	EulerAngle bg1_t0(0, 0, 0);
	EulerAngle bg1_td0(0, 0, 0);
	EulerAngle bg1_tdd0(0, 0, 0);
	RigidBody bg1(5, "bg1", 0, 1, bg_mass, bg_rotiner, bg1_r0, bg1_rd0, bg1_rdd0, bg1_t0, bg1_td0, bg1_tdd0);
	// primary suspension markers-high
	Vector3x bg1_ps_r1(-1 * axle_base / 2.0, suspension_base / 2.0, ps_high - bg_cg_height);
	EulerAngle bg1_ps_ea1(0.0, 0.0, 0.0);
	Vector3x bg1_ps_r2(-1 * axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_high - bg_cg_height);
	EulerAngle bg1_ps_ea2(0.0, 0.0, 0.0);
	Vector3x bg1_ps_r3(axle_base / 2.0, suspension_base / 2.0, ps_high - bg_cg_height); 
	EulerAngle bg1_ps_ea3(0.0, 0.0, 0.0);
	Vector3x bg1_ps_r4(axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_high - bg_cg_height); 
	EulerAngle bg1_ps_ea4(0.0, 0.0, 0.0);
	Marker bg1_ps_h1(1, bg1_ps_r1, bg1_ps_ea1); //marker number 1
	Marker bg1_ps_h2(2, bg1_ps_r2, bg1_ps_ea2); //marker number 2
	Marker bg1_ps_h3(3, bg1_ps_r3, bg1_ps_ea3); //marker number 3
	Marker bg1_ps_h4(4, bg1_ps_r4, bg1_ps_ea4); //marker number 4
	bg1.addMarker(bg1_ps_h1); 
	bg1.addMarker(bg1_ps_h2);
	bg1.addMarker(bg1_ps_h3); 
	bg1.addMarker(bg1_ps_h4);
	// primary suspension markers-low
	Vector3x bg1_ps_r11(-1 * axle_base / 2.0, suspension_base / 2.0, ps_low - bg_cg_height); 
	EulerAngle bg1_ps_ea11(0.0, 0.0, 0.0);
	Vector3x bg1_ps_r22(-1 * axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_low - bg_cg_height);
	EulerAngle bg1_ps_ea22(0.0, 0.0, 0.0);
	Vector3x bg1_ps_r33(axle_base / 2.0, suspension_base / 2.0, ps_low - bg_cg_height); 
	EulerAngle bg1_ps_ea33(0.0, 0.0, 0.0);
	Vector3x bg1_ps_r44(axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_low - bg_cg_height);
	EulerAngle bg1_ps_ea44(0.0, 0.0, 0.0);
	Marker bg1_ps_l1(5, bg1_ps_r11, bg1_ps_ea11); //marker number 5
	Marker bg1_ps_l2(6, bg1_ps_r22, bg1_ps_ea22); //marker number 6
	Marker bg1_ps_l3(7, bg1_ps_r33, bg1_ps_ea33); //marker number 7
	Marker bg1_ps_l4(8, bg1_ps_r44, bg1_ps_ea44); //marker number 8
	bg1.addMarker(bg1_ps_l1); bg1.addMarker(bg1_ps_l2); bg1.addMarker(bg1_ps_l3); bg1.addMarker(bg1_ps_l4);
	// second suspension markers-low
	Vector3x bg1_ss_r1(0.0, ss_suspension_base / 2.0, ss_low - bg_cg_height); 
	EulerAngle bg1_ss_ea1(0.0, 0.0, 0.0);
	Vector3x bg1_ss_r2(0.0, (-1.0) * ss_suspension_base / 2.0, ss_low - bg_cg_height); 
	EulerAngle bg1_ss_ea2(0.0, 0.0, 0.0);
	Marker bg1_ss_L(9, bg1_ss_r1, bg1_ss_ea1); //marker number 9
	Marker bg1_ss_R(10, bg1_ss_r2, bg1_ss_ea2); //marker number 10
	bg1.addMarker(bg1_ss_L);
	bg1.addMarker(bg1_ss_R);

	ResObserver bgobs1(1, "bg1", 1);
	bg1.CreateObserverHdr(bgobs1);
	vh1.CreateInertiaElemHdr(bg1);

	////3-2 bg2
	Vector3x bg2_r0(axle_base / 2.0 + bogie_base, 0.000, bg_cg_height); Vector3x bg2_rd0(0, 0, 0); Vector3x bg2_rdd0(0, 0, 0);
	EulerAngle bg2_t0(0, 0, 0); EulerAngle bg2_td0(0, 0, 0); EulerAngle bg2_tdd0(0, 0, 0);
	RigidBody bg2(6, "bg2", 0, 1, bg_mass, bg_rotiner, bg2_r0, bg2_rd0, bg2_rdd0, bg2_t0, bg2_td0, bg2_tdd0);
	// primary suspension markers-high
	Vector3x bg2_ps_r1(-1 * axle_base / 2.0, suspension_base / 2.0, ps_high - bg_cg_height);
	EulerAngle bg2_ps_ea1(0.0, 0.0, 0.0);
	Vector3x bg2_ps_r2(-1 * axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_high - bg_cg_height); 
	EulerAngle bg2_ps_ea2(0.0, 0.0, 0.0);
	Vector3x bg2_ps_r3(axle_base / 2.0, suspension_base / 2.0, ps_high - bg_cg_height);
	EulerAngle bg2_ps_ea3(0.0, 0.0, 0.0);
	Vector3x bg2_ps_r4(axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_high - bg_cg_height); 
	EulerAngle bg2_ps_ea4(0.0, 0.0, 0.0);
	Marker bg2_ps_h1(1, bg2_ps_r1, bg2_ps_ea1); //marker number 1
	Marker bg2_ps_h2(2, bg2_ps_r2, bg2_ps_ea2); //marker number 2
	Marker bg2_ps_h3(3, bg2_ps_r3, bg2_ps_ea3); //marker number 3
	Marker bg2_ps_h4(4, bg2_ps_r4, bg2_ps_ea4); //marker number 4
	bg2.addMarker(bg2_ps_h1); bg2.addMarker(bg2_ps_h2); bg2.addMarker(bg2_ps_h3); bg2.addMarker(bg2_ps_h4);
	//primary suspension markers-low
	Vector3x bg2_ps_r11(-1 * axle_base / 2.0, suspension_base / 2.0, ps_low - bg_cg_height); 
	EulerAngle bg2_ps_ea11(0.0, 0.0, 0.0);
	Vector3x bg2_ps_r22(-1 * axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_low - bg_cg_height);
	EulerAngle bg2_ps_ea22(0.0, 0.0, 0.0);
	Vector3x bg2_ps_r33(axle_base / 2.0, suspension_base / 2.0, ps_low - bg_cg_height); 
	EulerAngle bg2_ps_ea33(0.0, 0.0, 0.0);
	Vector3x bg2_ps_r44(axle_base / 2.0, (-1.0) * suspension_base / 2.0, ps_low - bg_cg_height);
	EulerAngle bg2_ps_ea44(0.0, 0.0, 0.0);
	Marker bg2_ps_l1(5, bg2_ps_r11, bg2_ps_ea11); //marker number 5
	Marker bg2_ps_l2(6, bg2_ps_r22, bg2_ps_ea22); //marker number 6
	Marker bg2_ps_l3(7, bg2_ps_r33, bg2_ps_ea33); //marker number 7
	Marker bg2_ps_l4(8, bg2_ps_r44, bg2_ps_ea44); //marker number 8
	bg2.addMarker(bg2_ps_l1); bg2.addMarker(bg2_ps_l2); bg2.addMarker(bg2_ps_l3); bg2.addMarker(bg2_ps_l4);
	// second suspension markers
	Vector3x bg2_ss_r1(0.0, ss_suspension_base / 2.0, ss_low - bg_cg_height); 
	EulerAngle bg2_ss_ea1(0.0, 0.0, 0.0);
	Vector3x bg2_ss_r2(0.0, (-1.0) * ss_suspension_base / 2.0, ss_low - bg_cg_height); 
	EulerAngle bg2_ss_ea2(0.0, 0.0, 0.0);
	Marker bg2_ss_L(9, bg2_ss_r1, bg2_ss_ea1); //marker number 9
	Marker bg2_ss_R(10, bg2_ss_r2, bg2_ss_ea2); //marker number 10
	bg2.addMarker(bg2_ss_L);
	bg2.addMarker(bg2_ss_R);
	//
	ResObserver bgobs2(1, "bg2", 1);
	bg2.CreateObserverHdr(bgobs2);
	vh1.CreateInertiaElemHdr(bg2);

	//4 carbody cb cb1_
	Vector3x cb1_r0(axle_base / 2.0 + bogie_base / 2.0, 0.000, cb_cg_height);
	Vector3x cb1_rd0(0, 0, 0); Vector3x cb1_rdd0(0, 0, 0);
	EulerAngle cb1_t0(0, 0, 0); EulerAngle cb1_td0(0, 0, 0); 
	EulerAngle cb1_tdd0(0, 0, 0);
	RigidBody cb1(7, "cb1", 0, 1, cb_mass, cb_rotiner, cb1_r0, cb1_rd0, cb1_rdd0, cb1_t0, cb1_td0, cb1_tdd0);

	// carbody second suspension markers
	Vector3x cb_ss_r1(-1 * bogie_base / 2.0, ss_suspension_base / 2.0, ss_high - cb_cg_height); 
	EulerAngle cb_ss_ea1(0.0, 0.0, 0.0);
	Vector3x cb_ss_r2(-1 * bogie_base / 2.0, (-1.0) * ss_suspension_base / 2.0, ss_high - cb_cg_height); 
	EulerAngle cb_ss_ea2(0.0, 0.0, 0.0);
	Vector3x cb_ss_r3(bogie_base / 2.0, ss_suspension_base / 2.0, ss_high - cb_cg_height);
	EulerAngle cb_ss_ea3(0.0, 0.0, 0.0);
	Vector3x cb_ss_r4(bogie_base / 2.0, (-1.0) * ss_suspension_base / 2.0, ss_high - cb_cg_height); 
	EulerAngle cb_ss_ea4(0.0, 0.0, 0.0);

	Vector3x cb_ss_r11(-1 * bogie_base / 2.0, ss_suspension_base / 2.0, ss_low - cb_cg_height); 
	EulerAngle cb_ss_ea11(0.0, 0.0, 0.0);
	Vector3x cb_ss_r22(-1 * bogie_base / 2.0, (-1.0) * ss_suspension_base / 2.0, ss_low - cb_cg_height); 
	EulerAngle cb_ss_ea22(0.0, 0.0, 0.0);
	Vector3x cb_ss_r33(bogie_base / 2.0, ss_suspension_base / 2.0, ss_low - cb_cg_height);
	EulerAngle cb_ss_ea33(0.0, 0.0, 0.0);
	Vector3x cb_ss_r44(bogie_base / 2.0, (-1.0) * ss_suspension_base / 2.0, ss_low - cb_cg_height); 
	EulerAngle cb_ss_ea44(0.0, 0.0, 0.0);

	Marker cb_ss_h1(1, cb_ss_r1, cb_ss_ea1); //marker number 1
	Marker cb_ss_h2(2, cb_ss_r2, cb_ss_ea2); //marker number 2
	Marker cb_ss_h3(3, cb_ss_r3, cb_ss_ea3); //marker number 3
	Marker cb_ss_h4(4, cb_ss_r4, cb_ss_ea4); //marker number 4

	Marker cb_ss_l1(5, cb_ss_r11, cb_ss_ea11); //marker number 5
	Marker cb_ss_l2(6, cb_ss_r22, cb_ss_ea22); //marker number 6
	Marker cb_ss_l3(7, cb_ss_r33, cb_ss_ea33); //marker number 7
	Marker cb_ss_l4(8, cb_ss_r44, cb_ss_ea44); //marker number 8
	//
	cb1.addMarker(cb_ss_h1); cb1.addMarker(cb_ss_h2); cb1.addMarker(cb_ss_h3); cb1.addMarker(cb_ss_h4);
	cb1.addMarker(cb_ss_l1); cb1.addMarker(cb_ss_l2); cb1.addMarker(cb_ss_l3); cb1.addMarker(cb_ss_l4);
	//
	ResObserver cbobs1(1, "cb1", 1);
	cb1.CreateObserverHdr(cbobs1);
	vh1.CreateInertiaElemHdr(cb1);
	//

	//5 primary suspensions
	double ps_kz = 1.22e6; 
	double ps_cz = 4e3;
	Vector3x ps_kxy; 
	Vector3x ps_cxy;
	ps_kxy[0] = 31.391e6; ps_kxy[1] = 3.884e6; ps_kxy[2] = 0;
	ps_cxy[0] = 15e3; ps_cxy[1] = 2e3; ps_cxy[2] = 0;
	//
	LinearTSDA ps_z1(0, "psz1", vh1.Inertia_array[1].getPointer(), 3, vh1.Inertia_array[5].getPointer(), 1, ps_kz, ps_cz, ps_high - ps_low);
	LinearTSDA ps_z2(1, "psz2", vh1.Inertia_array[1].getPointer(), 4, vh1.Inertia_array[5].getPointer(), 2, ps_kz, ps_cz, ps_high - ps_low);
	LinearTSDA ps_z3(2, "psz3", vh1.Inertia_array[2].getPointer(), 3, vh1.Inertia_array[5].getPointer(), 3, ps_kz, ps_cz, ps_high - ps_low);
	LinearTSDA ps_z4(3, "psz4", vh1.Inertia_array[2].getPointer(), 4, vh1.Inertia_array[5].getPointer(), 4, ps_kz, ps_cz, ps_high - ps_low);
	PointTSDA ps_xy1(4, "psxy1", vh1.Inertia_array[1].getPointer(), 3, vh1.Inertia_array[5].getPointer(), 5, ps_kxy, ps_cxy);
	PointTSDA ps_xy2(5, "psxy2", vh1.Inertia_array[1].getPointer(), 4, vh1.Inertia_array[5].getPointer(), 6, ps_kxy, ps_cxy);
	PointTSDA ps_xy3(6, "psxy3", vh1.Inertia_array[2].getPointer(), 3, vh1.Inertia_array[5].getPointer(), 7, ps_kxy, ps_cxy);
	PointTSDA ps_xy4(7, "psxy4", vh1.Inertia_array[2].getPointer(), 4, vh1.Inertia_array[5].getPointer(), 8, ps_kxy, ps_cxy);
	vh1.CreateForceElemHdr(ps_z1);
	vh1.CreateForceElemHdr(ps_z2);
	vh1.CreateForceElemHdr(ps_z3);
	vh1.CreateForceElemHdr(ps_z4);
	vh1.CreateForceElemHdr(ps_xy1);
	vh1.CreateForceElemHdr(ps_xy2);
	vh1.CreateForceElemHdr(ps_xy3);
	vh1.CreateForceElemHdr(ps_xy4);
	//
	LinearTSDA ps_z5(8, "psz5", vh1.Inertia_array[3].getPointer(), 3, vh1.Inertia_array[6].getPointer(), 1, ps_kz, ps_cz, ps_high - ps_low);
	LinearTSDA ps_z6(9, "psz6", vh1.Inertia_array[3].getPointer(), 4, vh1.Inertia_array[6].getPointer(), 2, ps_kz, ps_cz, ps_high - ps_low);
	LinearTSDA ps_z7(10, "psz7", vh1.Inertia_array[4].getPointer(), 3, vh1.Inertia_array[6].getPointer(), 3, ps_kz, ps_cz, ps_high - ps_low);
	LinearTSDA ps_z8(11, "psz8", vh1.Inertia_array[4].getPointer(), 4, vh1.Inertia_array[6].getPointer(), 4, ps_kz, ps_cz, ps_high - ps_low);
	PointTSDA ps_xy5(12, "psxy5", vh1.Inertia_array[3].getPointer(), 3, vh1.Inertia_array[6].getPointer(), 5, ps_kxy, ps_cxy);
	PointTSDA ps_xy6(13, "psxy6", vh1.Inertia_array[3].getPointer(), 4, vh1.Inertia_array[6].getPointer(), 6, ps_kxy, ps_cxy);
	PointTSDA ps_xy7(14, "psxy7", vh1.Inertia_array[4].getPointer(), 3, vh1.Inertia_array[6].getPointer(), 7, ps_kxy, ps_cxy);
	PointTSDA ps_xy8(15, "psxy8", vh1.Inertia_array[4].getPointer(), 4, vh1.Inertia_array[6].getPointer(), 8, ps_kxy, ps_cxy);
	vh1.CreateForceElemHdr(ps_z5);
	vh1.CreateForceElemHdr(ps_z6);
	vh1.CreateForceElemHdr(ps_z7);
	vh1.CreateForceElemHdr(ps_z8);
	vh1.CreateForceElemHdr(ps_xy5);
	vh1.CreateForceElemHdr(ps_xy6);
	vh1.CreateForceElemHdr(ps_xy7);
	vh1.CreateForceElemHdr(ps_xy8);
	//
	//6 second suspensions
	double ss_kz = 0.43e6; double ss_cz = 20e3;
	Vector3x ss_kxy; 
	Vector3x ss_cxy;
	ss_kxy[0] = 0.16e6; ss_kxy[1] = 0.16e6; ss_kxy[2] = 0;
	ss_cxy[0] = 25e3; ss_cxy[1] = 32e3; ss_cxy[2] = 0;
	//
	LinearTSDA ss_z1(16, "ssz1", vh1.Inertia_array[7].getPointer(), 1, vh1.Inertia_array[5].getPointer(), 9, ss_kz, ss_cz, ss_high - ss_low);
	LinearTSDA ss_z2(17, "ssz2", vh1.Inertia_array[7].getPointer(), 2, vh1.Inertia_array[5].getPointer(), 10, ss_kz, ss_cz, ss_high - ss_low);
	LinearTSDA ss_z3(18, "ssz3", vh1.Inertia_array[7].getPointer(), 3, vh1.Inertia_array[6].getPointer(), 9, ss_kz, ss_cz, ss_high - ss_low);
	LinearTSDA ss_z4(19, "ssz4", vh1.Inertia_array[7].getPointer(), 4, vh1.Inertia_array[6].getPointer(), 10, ss_kz, ss_cz, ss_high - ss_low);
	PointTSDA ss_xy1(20, "ssxy1", vh1.Inertia_array[7].getPointer(), 5, vh1.Inertia_array[5].getPointer(), 9, ss_kxy, ss_cxy);
	PointTSDA ss_xy2(21, "ssxy2", vh1.Inertia_array[7].getPointer(), 6, vh1.Inertia_array[5].getPointer(), 10, ss_kxy, ss_cxy);
	PointTSDA ss_xy3(22, "ssxy3", vh1.Inertia_array[7].getPointer(), 7, vh1.Inertia_array[6].getPointer(), 9, ss_kxy, ss_cxy);
	PointTSDA ss_xy4(23, "ssxy4", vh1.Inertia_array[7].getPointer(), 8, vh1.Inertia_array[6].getPointer(), 10, ss_kxy, ss_cxy);
	vh1.CreateForceElemHdr(ss_z1);
	vh1.CreateForceElemHdr(ss_z2);
	vh1.CreateForceElemHdr(ss_z3);
	vh1.CreateForceElemHdr(ss_z4);
	vh1.CreateForceElemHdr(ss_xy1);
	vh1.CreateForceElemHdr(ss_xy2);
	vh1.CreateForceElemHdr(ss_xy3);
	vh1.CreateForceElemHdr(ss_xy4);

	// computation module
	vh1.InitBalancedPosition(0.0001, 2.0);
	vh1.UpdateStaticConfiguration();
	double V = 10;
	vh1.SetRunSpeed(V);
	vh1.UpdateVelocityConfiguration();
	vh1.RK4(0.0002, 1.0);

}


void CopenvsdDlg::OnBnClickedButton7()
{
	// TODO: 在此添加控件通知处理程序代码
	Vehicle dof1;
	//1 create body
	Vector3x rotiner(1, 1, 1);
	Vector3x r0(0, 0, 1);
	//Vector3x r0(0, 0, 0);
	Vector3x rd0(0, 0, 0);
	Vector3x rdd0(0, 0, 0);
	EulerAngle t0(0, 0, 0);
	EulerAngle td0(0, 0, 0);
	EulerAngle tdd0(0, 0, 0);
	RigidBody body1(1, "dof1 body", 0, 1, 1.0, rotiner, r0, rd0, rdd0, t0, td0, tdd0);
	ResObserver obs1(1, "body1res1", 1);
	body1.CreateObserverHdr(obs1);
	dof1.CreateInertiaElemHdr(body1);
	//2 create tsda
	LinearTSDA ts1(0, "linear spring", dof1.Inertia_array[1].getPointer(), 0, dof1.Inertia_array[0].getPointer(), 0, 50, 4, 1.0);

	dof1.CreateForceElemHdr(ts1);
	//dof1.InitBalancedPosition(0.01, 10);
	//dof1.UpdateStaticConfiguration();
	dof1.RK4(0.001, 5);
	//dof1.ABAM(0.001, 10);
	//dof1.Newmark(0.01, 10, 0.25, 0.5);
	//FreeConsole();
}


void CopenvsdDlg::OnBnClickedButton8()
{
	// TODO: 在此添加控件通知处理程序代码
	
}
