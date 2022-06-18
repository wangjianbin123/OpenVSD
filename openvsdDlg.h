
// openvsdDlg.h: 头文件
//

#ifndef _OBSERVER_H_
#define _OBSERVER_H_
#include "Observer.h"
#endif

#ifndef _OBSERVERHDR_H_
#define _OBSERVERHDR_H_
#include "ObserverHdr.h"
#endif

#ifndef _RESOBSERVER_H_
#define _RESOBSERVER_H_
#include "ResObserver.h"
#endif

#ifndef _VEHICLE_H_
#define _VEHICLE_H_
#include "Vehicle.h"
#endif

#pragma once


// CopenvsdDlg 对话框
class CopenvsdDlg : public CDialogEx
{
// 构造
public:
	CopenvsdDlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_OPENVSD_DIALOG };
#endif

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
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton6();
	afx_msg void OnBnClickedButton7();
	afx_msg void OnBnClickedButton8();
};
