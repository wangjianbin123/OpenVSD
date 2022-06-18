
// Track.cpp : 实现文件
#include "pch.h"
#include "Track.h"

#ifndef _MATH_H_
#define _MATH_H_
#include <math.h>
#endif

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

#ifndef _VECTOR3X_H_
#define _VECTOR3X_H_
#include "Vector3x.h"
#endif

#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_
#include "CubicSpline.h"
#endif

#ifndef _CSPLINE_H_
#define _CSPLINE_H_
#include "Spline.h"
#endif

#ifndef _EULERANGLE_H_
#define _EULERANGLE_H_
#include "EulerAngle.h"
#endif

#ifndef _EULERPARA_H_
#define _EULERPARA_H_
#include "EulerPara.h"
#endif


#ifndef _GAUSS_INTE_LIMIT_
#define GAUSS_INTE 1.0e-8
#endif

#include <vector>

// Track
//IMPLEMENT_SERIAL(Track,CObject,VERSIONABLE_SCHEMA|2)

Track::Track():m_num(0),m_name("DEFAULT TRACK")
{
	m_nDiscription.Empty();
	m_nConPnt.Empty();
	m_nCPxyz.Empty();
	m_nOrigin.reSet(0,0,0);
	m_nOrientation.reSet(0,0,0);
}
Track::Track(Matrix &mx,Matrix &pmxl,Matrix &pmxr)           
{
	m_num = 1;
	m_name = "Track0";
	gauge_width = 1.500;
	dp_interval = 10.000;
	cant = 0.025;
	m_nDiscription = mx;
	EulerAngle leftcant(atan(1.0*cant),0.0,  0.0);
	EulerAngle rightcant(atan(-1.0*cant),0.0,  0.0);
	Matrix left(pmxl.Row(), 3);
	Matrix right(pmxr.Row(), 3);
	for (int i = 0; i < pmxl.Row(); i++)
	{
		left[i][0] = pmxl[i][0];
		left[i][1] = pmxl[i][1];
	}
	for (int j = 0; j < pmxr.Row(); j++)
	{
		right[j][0] = pmxr[j][0];
		right[j][1] = pmxr[j][1];
	}
	Matrix leftc = ~(leftcant.getRotMat()*(~left));
	Matrix rightc = ~(rightcant.getRotMat()*(~right));
	LProfile = leftc.getDoubleCols(0, 1);
	RProfile = rightc.getDoubleCols(0, 1);
	this->InitTrackDiffGeo();
	this->InitCurvature();
}
Track::Track(Matrix &mxdescrip,Matrix &leftmx,Matrix &rightmx,Vector3x ori,EulerAngle ang,double gauge,double c,double interval,int sm,int dersm1st,int dersm2nd)
{
	m_num=1;
	m_name="Track0";
	gauge_width=gauge;
	m_nOrigin=ori;
	m_nOrientation=ang;
	cant=c;
	dp_interval=interval;
	m_nDiscription=mxdescrip;
	EulerAngle leftcant(atan(1.0*c),0.0,  0.0);
	EulerAngle rightcant(atan(-1.0*c),0.0, 0.0);
	Matrix left(leftmx.Row(), 3);
	Matrix right(rightmx.Row(), 3);
	for (int i = 0; i < leftmx.Row(); i++)
	{
		left[i][0] = leftmx[i][0];
		left[i][1] = leftmx[i][1];
	}
	for (int j = 0; j < rightmx.Row(); j++)
	{
		right[j][0] = rightmx[j][0];
		right[j][1] = rightmx[j][1];
	}
	Matrix leftc = ~(leftcant.getRotMat()*(~left));
	Matrix rightc = ~(rightcant.getRotMat()*(~right));
	LProfile = leftc.getDoubleCols(0, 1);
	RProfile = rightc.getDoubleCols(0, 1);
	this->InitTrackDiffGeo(sm,dersm1st,dersm2nd);
	this->InitCurvature();
}
Track::Track(Matrix &mxdescrip, Matrix &leftmx, Matrix &rightmx, Vector3x ori, EulerAngle ang, double gauge, double c, double interval, int sm, int dersm1st, int dersm2nd, int csm1, int csm2, int csm3)
{
	m_num = 1;
	m_name = "Track0";
	gauge_width = gauge;
	m_nOrigin = ori;
	m_nOrientation = ang;
	cant = c;
	dp_interval = interval;
	m_nDiscription = mxdescrip;
	EulerAngle leftcant(atan(1.0*c), 0.0, 0.0);
	EulerAngle rightcant(atan(-1.0*c), 0.0, 0.0);
	Matrix left(leftmx.Row(), 3);
	Matrix right(rightmx.Row(), 3);
	for (int i = 0; i < leftmx.Row(); i++)
	{
		left[i][0] = leftmx[i][0];
		left[i][1] = leftmx[i][1];
	}
	for (int j = 0; j < rightmx.Row(); j++)
	{
		right[j][0] = rightmx[j][0];
		right[j][1] = rightmx[j][1];
	}
	Matrix leftc = ~(leftcant.getRotMat()*(~left));
	Matrix rightc = ~(rightcant.getRotMat()*(~right));
	LProfile = leftc.getDoubleCols(0, 1);
	RProfile = rightc.getDoubleCols(0, 1);
	this->InitTrackDiffGeo(sm, dersm1st, dersm2nd);
	/*this->InitCurvature();*/
	this->InitCurvature(csm1, csm2, csm3);
}
Track::Track(const Track &tk)
{
	m_num=tk.m_num;
	m_name=tk.m_name;
	m_nDiscription=tk.getDiscription();
	LProfile=tk.getLProfile();
	RProfile=tk.getRProfile();
	m_nOrigin=tk.getOrigin();
	m_nOrientation=tk.getOrientation();
	gauge_width=tk.getWidth();
	cant=tk.getcant();
	dp_interval=tk.getdpinterval();
	m_nConPnt=tk.getConPntMat();
	m_nCPxyz=tk.getCPxyzMat();
	m_nArcLenDiffLeft=tk.getArcLenDiffMatLeft();
	m_nArcLenDiffRight=tk.getArcLenDiffMatRight();
	LeftRP_Data=tk.getLeftRPDATA();
	RightRP_Data=tk.getRightRPDADA();
	LeftRailCS=tk.getLeftRailCS();
	RightRailCS=tk.getRightRailCS();
	LRProfDeri_1st=tk.getLRP_D1st();
	LRProfDeri_2nd=tk.getLRP_D2nd();
	LRProfDeri_3rd = tk.getLRP_D3rd();//20191025 added
	RRProfDeri_1st=tk.getRRP_D1st();
	RRProfDeri_2nd=tk.getRRP_D2nd();
	RRProfDeri_3rd = tk.getRRP_D3rd();//20101025 added
	LProfileCS=tk.getLProfileCS();
	RProfileCS=tk.getRProfileCS();
	LProfile_D1st=tk.getLProfile_D1st();
	LProfile_D2nd=tk.getLProfile_D2nd();
	RProfile_D1st=tk.getRProfile_D1st();
	RProfile_D2nd=tk.getRProfile_D2nd();
	LPcurMat=tk.getLPcurMat();
	RPcurMat=tk.getRPcurMat();
	lrp=tk.getlrp();
	rrp=tk.getrrp();
	lpdeltas=tk.getlpdeltas();
	lps=tk.getlps();
	rpdeltas=tk.getrpdeltas();
	rps=tk.getrps();
	lcur=tk.getlcur();
	rcur=tk.getrcur();
	xindic=tk.getxindic();
}
Track& Track::operator =(const Track &tk)
{
	if (this==&tk)
	{
		return *this;
	}
	else
	{
		m_num=tk.m_num;
		m_name=tk.m_name;
		m_nDiscription=tk.getDiscription();
		LProfile=tk.getLProfile();
		RProfile=tk.getRProfile();
		m_nOrigin=tk.getOrigin();
		m_nOrientation=tk.getOrientation();
		gauge_width=tk.getWidth();
		cant=tk.getcant();
		dp_interval=tk.getdpinterval();
		m_nConPnt=tk.getConPntMat();
		m_nCPxyz=tk.getCPxyzMat();
		m_nArcLenDiffLeft=tk.getArcLenDiffMatLeft();
		m_nArcLenDiffRight=tk.getArcLenDiffMatRight();
		LeftRP_Data=tk.getLeftRPDATA();
		RightRP_Data=tk.getRightRPDADA();
		LeftRailCS=tk.getLeftRailCS();
		RightRailCS=tk.getRightRailCS();
		LRProfDeri_1st=tk.getLRP_D1st();
		LRProfDeri_2nd=tk.getLRP_D2nd();
		LRProfDeri_3rd = tk.getLRP_D3rd();//20191025 added
		RRProfDeri_1st=tk.getRRP_D1st();
		RRProfDeri_2nd=tk.getRRP_D2nd();
		RRProfDeri_3rd = tk.getRRP_D3rd();//20191025 added
		LProfileCS=tk.getLProfileCS();
		RProfileCS=tk.getRProfileCS();
		LProfile_D1st=tk.getLProfile_D1st();
		LProfile_D2nd=tk.getLProfile_D2nd();
		RProfile_D1st=tk.getRProfile_D1st();
		RProfile_D2nd=tk.getRProfile_D2nd();
		LPcurMat=tk.getLPcurMat();
		RPcurMat=tk.getRPcurMat();
		lrp=tk.getlrp();
		rrp=tk.getrrp();
		lpdeltas=tk.getlpdeltas();
		lps=tk.getlps();
		rpdeltas=tk.getrpdeltas();
		rps=tk.getrps();
		lcur=tk.getlcur();
		rcur=tk.getrcur();
		xindic=tk.getxindic();
		return *this;
	}
}
void Track::setParameters(int num,CString name,Matrix description,Matrix lp,Matrix rp,Vector3x ori,EulerAngle ang,double gauge,double c,double interval)
{
	m_num=num;
	m_name=name;
	m_nDiscription=description;
	m_nOrigin=ori;
	m_nOrientation=ang;
	gauge_width=gauge;
	cant=c;
	dp_interval=interval;
	EulerAngle leftcant(atan(1.0*c),0.0, 0.0);
	EulerAngle rightcant(atan(-1.0*c),0.0, 0.0);
	Matrix left(lp.Row(), 3);
	Matrix right(rp.Row(), 3);
	for (int i = 0; i < lp.Row(); i++)
	{
		left[i][0] = lp[i][0];
		left[i][1] = lp[i][1];
	}
	for (int j = 0; j < rp.Row(); j++)
	{
		right[j][0] = rp[j][0];
		right[j][1] = rp[j][1];
	}
	Matrix leftc = ~(leftcant.getRotMat()*(~left));
	Matrix rightc = ~(rightcant.getRotMat()*(~right));
	LProfile = leftc.getDoubleCols(0, 1);
	RProfile = rightc.getDoubleCols(0, 1);
	this->InitTrackDiffGeo();
	this->InitCurvature();
}
Track::~Track()
{
}
//初始化函数//////////////////////////////////////////////////////////////////////////////////////////
void Track::setConPntMat(void) //初始化控制点信息矩阵 0映射弧长 1摇头角 2侧滚角 3竖高 4实际弧长
{
	if (!m_nDiscription.IsEmpty())
	{
		m_nConPnt.SetMatrix(m_nDiscription.Row(),5);
		for (int i=0;i<m_nDiscription.Row();i++)
		{
			m_nConPnt[i][0]=m_nDiscription[i][1]; // Initialize projection S
			m_nConPnt[i][2]=m_nDiscription[i][3]; // Initialize Roll angle phi
			m_nConPnt[i][3]=m_nDiscription[i][4]; // Initialize hight z coordinate
		}
		for (int i=1;i<m_nDiscription.Row();i++) // Initialize Yaw angle psi
		{
			m_nConPnt[i][1]=m_nConPnt[i-1][1]+(1/(m_nDiscription[i][1]-m_nDiscription[i-1][1]))*((m_nDiscription[i][2]/2)*
				pow((m_nDiscription[i][1]-m_nDiscription[i-1][1]),2))+(m_nDiscription[i][1]-m_nDiscription[i-1][1])*m_nDiscription[i-1][2]/2;
		}
		for (int i=1;i<m_nDiscription.Row();i++) // Initialize actual arc length s
		{
			double slope=(m_nConPnt[i][3]-m_nConPnt[i-1][3])/(m_nConPnt[i][0]-m_nConPnt[i-1][0]);
			m_nConPnt[i][4]=m_nConPnt[i-1][4]+(m_nConPnt[i][0]-m_nConPnt[i-1][0])/(cos(atan(slope))); //version1.0.2 修改s初始化函数
		}
	}
}
void Track::setCPxyzMat(void)
{
	if (!m_nDiscription.IsEmpty())
	{
		m_nCPxyz.SetMatrix(m_nDiscription.Row(),3);
		//设定Z坐标
		for (int i=1;i<m_nDiscription.Row();i++)
		{
			if (m_nConPnt[i][3]==m_nConPnt[i-1][3])
			{
				m_nCPxyz[i][2]=m_nCPxyz[i-1][2];
			}
			else
			{
				double theta=atan((m_nConPnt[i][3]-m_nConPnt[i-1][3])/(m_nConPnt[i][0]-m_nConPnt[i-1][0]));
				m_nCPxyz[i][2]=m_nCPxyz[i-1][2]+(m_nConPnt[i][4]-m_nConPnt[i-1][4])*sin(theta); //version1.0.2 修改z坐标初始化函数
			}
		}
		//设定X坐标 20130610 modified for just gauss integration used only
		for (int i=1;i<m_nDiscription.Row();i++)
		{
			//曲线段高斯5点积分
			double conven_delta = GAUSS_INTE;
			double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
			double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
			double inte_interval=m_nConPnt[i][0]-m_nConPnt[i-1][0];
			double inte1=0.0;
			double inte2=0.0;
			double low,high,tmp,coff,angle;

			//初始进行积分域分两段
			int sub=2;
			for (int k=0;k<sub;k++)
			{
				low=m_nConPnt[i-1][0]+k*inte_interval/sub;
				high=m_nConPnt[i-1][0]+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					angle=this->getRPYaw(coff);
					tmp+=weight[j]*((high-low)/2)*cos(angle);
				}
				inte2+=tmp;
			}
			while (abs(inte2-inte1)>conven_delta)
			{
				inte1=inte2;
				sub*=2;
				inte2=0.0;
				for (int k=0;k<sub;k++)
				{
					low=m_nConPnt[i-1][0]+k*inte_interval/sub;
					high=m_nConPnt[i-1][0]+(k+1)*inte_interval/sub;
					tmp=0.0;
					for (int j=0;j<5;j++)
					{
						coff=((high-low)/2)*intpnt[j]+((high+low)/2);
						angle=this->getRPYaw(coff);
						tmp+=weight[j]*((high-low)/2)*cos(angle);
					}
					inte2+=tmp;
				}
			}
			m_nCPxyz[i][0]=m_nCPxyz[i-1][0]+inte2;
		}
		//设定Y坐标 20130610 modified for just gauss integration used only
		for (int i=1;i<m_nDiscription.Row();i++)
		{		
			//曲线段高斯5点积分
			double conven_delta = GAUSS_INTE;
			double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
			double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
			double inte_interval=m_nConPnt[i][0]-m_nConPnt[i-1][0];
			double inte1=0.0;
			double inte2=0.0;
			double low,high,tmp,coff,angle;

			//初始进行积分域分两段
			int sub=2;
			for (int k=0;k<sub;k++)
			{
				low=m_nConPnt[i-1][0]+k*inte_interval/sub;
				high=m_nConPnt[i-1][0]+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					angle=this->getRPYaw(coff);
					tmp+=weight[j]*((high-low)/2)*sin(angle);
				}
				inte2+=tmp;
			}
			while (abs(inte2-inte1)>conven_delta)
			{
				inte1=inte2;
				sub*=2;
				inte2=0.0;
				for (int k=0;k<sub;k++)
				{
					low=m_nConPnt[i-1][0]+k*inte_interval/sub;
					high=m_nConPnt[i-1][0]+(k+1)*inte_interval/sub;
					tmp=0.0;
					for (int j=0;j<5;j++)
					{
						coff=((high-low)/2)*intpnt[j]+((high+low)/2);
						angle=this->getRPYaw(coff);
						tmp+=weight[j]*((high-low)/2)*sin(angle);
					}
					inte2+=tmp;
				}
			}
			m_nCPxyz[i][1]=m_nCPxyz[i-1][1]+inte2;
		}
	}
}
void Track::setArcLenDiffMatLeft(void)
{
	if (!m_nDiscription.IsEmpty())
	{
		m_nArcLenDiffLeft.SetMatrix(m_nDiscription.Row(),2);
		for (int ii=1;ii<m_nDiscription.Row();ii++)//第一列 轨道中心线映射弧长
		{
			m_nArcLenDiffLeft[ii][0]=m_nConPnt[ii][0];
		}
		for (int jj=1;jj<m_nDiscription.Row();jj++)
		{
			double conven_delta = GAUSS_INTE;
			double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
			double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
			double inte_interval=m_nConPnt[jj][0]-m_nConPnt[jj-1][0];//两控制点间的积分区间
			double inte1=0.0;
			double inte2=0.0;
			double low,high,tmp,coff,CH,phi,theta;
			//初始进行积分域分两段
			int sub=2;
			for (int k=0;k<sub;k++)
			{
				low=m_nConPnt[jj-1][0]+k*inte_interval/sub;
				high=m_nConPnt[jj-1][0]+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					CH=this->getCH(coff);
					phi=this->getRPRoll(coff);
					Vector3x vec3=this->LeftRailAngle(this->TranS2s(coff));
					theta=vec3[1];
					tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
				}
				inte2+=tmp;
			}
			while (abs(inte2-inte1)>conven_delta)
			{
				inte1=inte2;
				sub*=2;
				inte2=0.0;
				for (int k=0;k<sub;k++)
				{
					low=m_nConPnt[jj-1][0]+k*inte_interval/sub;
					high=m_nConPnt[jj-1][0]+(k+1)*inte_interval/sub;
					tmp=0.0;
					for (int j=0;j<5;j++)
					{
						coff=((high-low)/2)*intpnt[j]+((high+low)/2);
						CH=this->getCH(coff);
						phi=this->getRPRoll(coff);
						Vector3x vec3=this->LeftRailAngle(this->TranS2s(coff));
						theta=vec3[1];
						tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
					}
					inte2+=tmp;
				}
			}
			m_nArcLenDiffLeft[jj][1]=m_nArcLenDiffLeft[jj-1][1]+0.5*inte2;
		}
	}
}
void Track::setArcLenDiffMatRight(void)// 20130610 modified for just gauss integration used only
{
	if (!m_nDiscription.IsEmpty())
	{
		m_nArcLenDiffRight.SetMatrix(m_nDiscription.Row(),2);
		for (int ii=1;ii<m_nDiscription.Row();ii++)
		{
			m_nArcLenDiffRight[ii][0]=m_nConPnt[ii][0];
		}
		for (int jj=1;jj<m_nDiscription.Row();jj++)
		{
			double conven_delta = GAUSS_INTE;
			double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
			double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
			double inte_interval=m_nConPnt[jj][0]-m_nConPnt[jj-1][0];//两控制点间的积分区间
			double inte1=0.0;
			double inte2=0.0;
			double low,high,tmp,coff,CH,phi,theta;
			//初始进行积分域分两段
			int sub=2;
			for (int k=0;k<sub;k++)
			{
				low=m_nConPnt[jj-1][0]+k*inte_interval/sub;
				high=m_nConPnt[jj-1][0]+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					CH=this->getCH(coff);
					phi=this->getRPRoll(coff);
					Vector3x vec3=this->RightRailAngle(this->TranS2s(coff));
					theta=vec3[1];
					tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
				}
				inte2+=tmp;
			}
			while (abs(inte2-inte1)>conven_delta)
			{
				inte1=inte2;
				sub*=2;
				inte2=0.0;
				for (int k=0;k<sub;k++)
				{
					low=m_nConPnt[jj-1][0]+k*inte_interval/sub;
					high=m_nConPnt[jj-1][0]+(k+1)*inte_interval/sub;
					tmp=0.0;
					for (int j=0;j<5;j++)
					{
						coff=((high-low)/2)*intpnt[j]+((high+low)/2);
						CH=this->getCH(coff);
						phi=this->getRPRoll(coff);
						Vector3x vec3=this->RightRailAngle(this->TranS2s(coff));
						theta=vec3[1];
						tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
					}
					inte2+=tmp;
				}
			}
			m_nArcLenDiffRight[jj][1]=m_nArcLenDiffRight[jj-1][1]+0.5*inte2;
		}
	}
}

void Track::setLeftRPData(void)//（0 映射弧长；1 实际弧长；2 型面坐标X；3 型面坐标Y；4 型面坐标Z；5 型面摇头；6 型面点头；7 型面侧滚）
{
	int Row_DisMat=m_nDiscription.Row();
	int Data_Row=static_cast<int>((m_nDiscription[Row_DisMat-1][1]-m_nDiscription[0][1])/dp_interval+1);
	LeftRP_Data.SetMatrix(Data_Row,8);
	for (int i=0;i<Data_Row;i++)
	{
		LeftRP_Data[i][0]=i*dp_interval; //映射弧长 S
		LeftRP_Data[i][1]=this->TranS2s(i*dp_interval)-this->getDiffArcLenLeft(this->TranS2s(i*dp_interval));//左轨，负号，得到实际弧长
		Vector3x vec3=this->LeftRailRP(this->TranS2s(i*dp_interval));
		LeftRP_Data[i][2]=vec3[0];
		LeftRP_Data[i][3]=vec3[1];
		LeftRP_Data[i][4]=vec3[2];
		Vector3x vec3_angle=this->LeftRailAngle(this->TranS2s(i*dp_interval));
		LeftRP_Data[i][5]=vec3_angle[0];
		LeftRP_Data[i][6]=vec3_angle[1];
		LeftRP_Data[i][7]=vec3_angle[2];
	}
}
void Track::setRightRPData(void)//（0 映射弧长；1 实际弧长；2 型面坐标X；3 型面坐标Y；4 型面坐标Z；5 型面摇头；6 型面点头；7 型面侧滚）
{
	int Row_DisMat=m_nDiscription.Row();
	int Data_Row=static_cast<int>((m_nDiscription[Row_DisMat-1][1]-m_nDiscription[0][1])/dp_interval+1);
	RightRP_Data.SetMatrix(Data_Row,8);
	for (int i=0;i<Data_Row;i++)
	{
		RightRP_Data[i][0]=i*dp_interval; //映射弧长 S
		RightRP_Data[i][1]=this->TranS2s(i*dp_interval)+this->getDiffArcLenRight(this->TranS2s(i*dp_interval));//右轨，正号
		Vector3x vec3=this->RightRailRP(this->TranS2s(i*dp_interval));
		RightRP_Data[i][2]=vec3[0];
		RightRP_Data[i][3]=vec3[1];
		RightRP_Data[i][4]=vec3[2];
		Vector3x vec3_angle=this->RightRailAngle(this->TranS2s(i*dp_interval));
		RightRP_Data[i][5]=vec3_angle[0];
		RightRP_Data[i][6]=vec3_angle[1];
		RightRP_Data[i][7]=vec3_angle[2];
	}
}
void Track::setLeftCSs(void)//设置左轨型面坐标系样条曲线
{
	for (int i=1;i<=6;i++)
	{
		// as functons of the actural arc length
		Matrix mx=this->getLeftRPDATA().getDoubleCols(1,(i+1));
		CubicSpline cs(mx);
		LeftRailCS.push_back(cs);
	}
}

void Track::setRightCSs(void)//设置右轨型面坐标系样条曲线
{
	for(int i=1;i<=6;i++)
	{
		//as functions of the actural arc length
		Matrix mx=this->getRightRPDADA().getDoubleCols(1,(i+1));
		CubicSpline cs(mx);
		RightRailCS.push_back(cs);
	}
}
void Track::setLeftProfile_Derivate(void)
{
	for (int i=0;i<6;i++)
	{
		//CubicSpline used
		CubicSpline cs=LeftRailCS[i].DerivativeC();
		LRProfDeri_1st.push_back(cs);
		CubicSpline cs2=LRProfDeri_1st[i].DerivativeC();
		LRProfDeri_2nd.push_back(cs2);
		//20191025 added
		CubicSpline cs3 = LRProfDeri_2nd[i].DerivativeC();
		LRProfDeri_3rd.push_back(cs3);
	}
}
void Track::setRightProfile_Derivate(void)
{
	for (int i=0;i<6;i++)
	{
		//CubicSpline used
		CubicSpline cs=RightRailCS[i].DerivativeC();
		RRProfDeri_1st.push_back(cs);
		CubicSpline cs2=RRProfDeri_1st[i].DerivativeC();
		RRProfDeri_2nd.push_back(cs2);
		//20191025 added
		CubicSpline cs3 = RRProfDeri_2nd[i].DerivativeC();
		RRProfDeri_3rd.push_back(cs3);
	}
}
void Track::setProfileCS(void)
{
	LProfileCS.setParameters(LProfile);
	RProfileCS.setParameters(RProfile);

	LProfile_D1st = LProfileCS.Derivative();
	//20200916
	/*LProfile_D2nd = LProfile_D1st.Derivative();*/
	LProfile_D2nd = LProfileCS.Derivative2nd();
	RProfile_D1st = RProfileCS.Derivative();
	//20200916
	/*RProfile_D2nd = RProfile_D1st.Derivative();*/
	RProfile_D2nd = RProfileCS.Derivative2nd();
}
void Track::setProfileCS(int sm0,int sm1,int sm2)
{
	LProfileCS.setParameters(LProfile);
	RProfileCS.setParameters(RProfile);

	LProfileCS.gaussian_smothing_original(sm0,0.99);
	RProfileCS.gaussian_smothing_original(sm0, 0.99);
	LProfileCS.gaussian_smothing_derivative(sm1, 0.99);
	RProfileCS.gaussian_smothing_derivative(sm1, 0.99);
	LProfileCS.gaussian_smothing_derivative2nd(sm2, 0.99);
	RProfileCS.gaussian_smothing_derivative2nd(sm2, 0.99);

	LProfile_D1st = LProfileCS.Derivative();
	LProfile_D2nd = LProfileCS.Derivative2nd();
	RProfile_D1st = RProfileCS.Derivative();
	RProfile_D2nd = RProfileCS.Derivative2nd();
}
void Track::setPcurMat(void)
{
	LPcurMat.SetMatrix(LProfile.Row(),3);
	RPcurMat.SetMatrix(RProfile.Row(),3);
	for (int i=0;i<LProfile.Row();i++)
	{
		LPcurMat[i][0]=i;
		LPcurMat[i][1]=LProfile[i][0];
		LPcurMat[i][2]=LProfile[i][1];
	}
	for (int i=0;i<RProfile.Row();i++)
	{
		RPcurMat[i][0]=i;
		RPcurMat[i][1]=RProfile[i][0];
		RPcurMat[i][2]=RProfile[i][1];
	}
}
void Track::setSPcur(void)
{
	CubicSpline lpx(LPcurMat.getDoubleCols(0,1));
	lrp.push_back(lpx);
	CubicSpline lpy(LPcurMat.getDoubleCols(0,2));
	lrp.push_back(lpy);

	CubicSpline lpxd=lpx.Derivative();
	lrp.push_back(lpxd);
	CubicSpline lpyd=lpy.Derivative();
	lrp.push_back(lpyd);

	CubicSpline rpx(RPcurMat.getDoubleCols(0,1));
	rrp.push_back(rpx);
	CubicSpline rpy(RPcurMat.getDoubleCols(0,2));
	rrp.push_back(rpy);

	CubicSpline rpxd=rpx.Derivative();
	rrp.push_back(rpxd);
	CubicSpline rpyd=rpy.Derivative();
	rrp.push_back(rpyd);
}

void Track::setcurDeltaS(void)
{
	lpdeltas.SetMatrix(LPcurMat.Row()-1,2);
	rpdeltas.SetMatrix(RPcurMat.Row()-1,2);
	//高斯积分
	double conven_delta = GAUSS_INTE; //设定gauss积分误差限
	double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
	double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
	///left rail geometry for curvature computation
	for (int i=1;i<LPcurMat.Row();i++)
	{
		double inte_interval=LPcurMat[i][0]-LPcurMat[i-1][0];
		double inte1=0.0;
		double inte2=0.0;
		double low,high,tmp,coff;
		int sub=2;
		for (int k=0;k<sub;k++)
		{
			low=LPcurMat[i-1][0]+k*inte_interval/sub;
			high=LPcurMat[i-1][0]+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				tmp+=weight[j]*((high-low)/2)*sqrt(pow(lrp[2].ValueAt(coff),2.0)+pow(lrp[3].ValueAt(coff),2.0));
			}
			inte2+=tmp;
		}
		while (abs(inte2-inte1)>conven_delta)
		{
			inte1=inte2;
			sub*=2;
			inte2=0.0;
			for (int k=0;k<sub;k++)
			{
				low=LPcurMat[i-1][0]+k*inte_interval/sub;
				high=LPcurMat[i-1][0]+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					tmp+=weight[j]*((high-low)/2)*sqrt(pow(lrp[2].ValueAt(coff),2.0)+pow(lrp[3].ValueAt(coff),2.0));
				}
				inte2+=tmp;
			}
		}
		lpdeltas[i-1][0]=i-1;
		lpdeltas[i-1][1]=inte2;
	}
	//right rail geometry for curvature computation
	for (int i = 1; i<RPcurMat.Row(); i++)
	{
		double inte_interval=RPcurMat[i][0]-RPcurMat[i-1][0];
		double inte1=0.0;
		double inte2=0.0;
		double low,high,tmp,coff;
		int sub=2;
		for (int k=0;k<sub;k++)
		{
			low=RPcurMat[i-1][0]+k*inte_interval/sub;
			high=RPcurMat[i-1][0]+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				tmp+=weight[j]*((high-low)/2)*sqrt(pow(rrp[2].ValueAt(coff),2.0)+pow(rrp[3].ValueAt(coff),2.0));
			}
			inte2+=tmp;
		}
		while (abs(inte2-inte1)>conven_delta)
		{
			inte1=inte2;
			sub*=2;
			inte2=0.0;
			for (int k=0;k<sub;k++)
			{
				low=RPcurMat[i-1][0]+k*inte_interval/sub;
				high=RPcurMat[i-1][0]+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					tmp+=weight[j]*((high-low)/2)*sqrt(pow(rrp[2].ValueAt(coff),2.0)+pow(rrp[3].ValueAt(coff),2.0));
				}
				inte2+=tmp;
			}
		}
		rpdeltas[i-1][0]=i-1;
		rpdeltas[i-1][1]=inte2;
	}
	lps.SetMatrix(LPcurMat.Row(),4); 
	rps.SetMatrix(RPcurMat.Row(),4);
	for (int i=0;i<LPcurMat.Row();i++)
	{
		lps[i][0]=i;
		if (i>0)
		{
			lps[i][1]=lpdeltas[i-1][1]+lps[i-1][1];
		}
		lps[i][2]=LPcurMat[i][1];//x
		lps[i][3]=LPcurMat[i][2];//y
	}
	for (int i=0;i<RPcurMat.Row();i++)
	{
		rps[i][0]=i;
		if (i>0)
		{
			rps[i][1]=rpdeltas[i-1][1]+rps[i-1][1];
		}
		rps[i][2]=RPcurMat[i][1];//x
		rps[i][3]=RPcurMat[i][2];//y
	}
}
void Track::setcurSp(void)
{
	//left rail profile curvature
	CubicSpline cs1(lps.getDoubleCols(1,2));
	CubicSpline cs2(lps.getDoubleCols(1,3));
	CubicSpline cs3=cs1.Derivative();
	CubicSpline cs4=cs2.Derivative();
	CubicSpline cs5=cs3.Derivative();
	CubicSpline cs6=cs4.Derivative();
	lcur.push_back(cs1);
	lcur.push_back(cs2);
	lcur.push_back(cs3);
	lcur.push_back(cs4);
	lcur.push_back(cs5);
	lcur.push_back(cs6);
	//right rail profile curvature
	CubicSpline cs7(rps.getDoubleCols(1,2));
	CubicSpline cs8(rps.getDoubleCols(1,3));
	CubicSpline cs9=cs7.Derivative();
	CubicSpline cs10=cs8.Derivative();
	CubicSpline cs11=cs9.Derivative();
	CubicSpline cs12=cs10.Derivative();
	rcur.push_back(cs7);
	rcur.push_back(cs8);
	rcur.push_back(cs9);
	rcur.push_back(cs10);
	rcur.push_back(cs11);
	rcur.push_back(cs12);
	CubicSpline lxindic(lps.getDoubleCols(2,1));
	CubicSpline rxindic(rps.getDoubleCols(2,1));
	xindic.push_back(lxindic); xindic.push_back(rxindic);
}

void Track::setcurSp(int csm1, int csm2, int csm3)
{
	//left rail profile curvature
	CubicSpline cs1(lps.getDoubleCols(1, 2));
	CubicSpline cs2(lps.getDoubleCols(1, 3));
	//
	cs1.gaussian_smothing_original(csm1, 0.99);
	cs2.gaussian_smothing_original(csm1, 0.99);
	cs1.gaussian_smothing_derivative(csm2, 0.99);
	cs2.gaussian_smothing_derivative(csm2, 0.99);
	cs1.gaussian_smothing_derivative2nd(csm3, 0.99);
	cs2.gaussian_smothing_derivative2nd(csm3, 0.99);

	CubicSpline cs3 = cs1.Derivative();
	CubicSpline cs4 = cs2.Derivative();
	CubicSpline cs5 = cs1.Derivative2nd();
	CubicSpline cs6 = cs2.Derivative2nd();

	lcur.push_back(cs1);
	lcur.push_back(cs2);
	lcur.push_back(cs3);
	lcur.push_back(cs4);
	lcur.push_back(cs5);
	lcur.push_back(cs6);
	//right rail profile curvature
	CubicSpline cs7(rps.getDoubleCols(1, 2));
	CubicSpline cs8(rps.getDoubleCols(1, 3));

	cs7.gaussian_smothing_original(csm1, 0.99);
	cs8.gaussian_smothing_original(csm1, 0.99);
	cs7.gaussian_smothing_derivative(csm2, 0.99);
	cs8.gaussian_smothing_derivative(csm2, 0.99);
	cs7.gaussian_smothing_derivative2nd(csm3, 0.99);
	cs8.gaussian_smothing_derivative2nd(csm3, 0.99);

	CubicSpline cs9 = cs7.Derivative();
	CubicSpline cs10 = cs8.Derivative();
	CubicSpline cs11 = cs7.Derivative2nd();
	CubicSpline cs12 = cs8.Derivative2nd();

	rcur.push_back(cs7);
	rcur.push_back(cs8);
	rcur.push_back(cs9);
	rcur.push_back(cs10);
	rcur.push_back(cs11);
	rcur.push_back(cs12);
	//
	CubicSpline lxindic(lps.getDoubleCols(2, 1));
	CubicSpline rxindic(rps.getDoubleCols(2, 1));
	//
	xindic.push_back(lxindic); 
	xindic.push_back(rxindic);
}
void Track::InitTrackDiffGeo(void)
{
	this->setConPntMat();
	this->setCPxyzMat();
	this->setArcLenDiffMatLeft();
	this->setArcLenDiffMatRight();// left rail coordinates s x y z psi theta phi
	this->setLeftRPData(); // right rail coordinates s x y z psi theta phi
	this->setRightRPData(); // left rail coordinates cubic splines respect to s
	this->setLeftCSs(); // right rail coordinates cubic splines respect to s
	this->setRightCSs(); // rail profile coordinates cubic splines group
	this->setLeftProfile_Derivate();
	this->setRightProfile_Derivate();
	this->setProfileCS();
}

void Track::InitTrackDiffGeo(int sm0,int sm1,int sm2)
{
	this->setConPntMat();
	this->setCPxyzMat();
	this->setArcLenDiffMatLeft();
	this->setArcLenDiffMatRight();
	this->setLeftRPData();
	this->setRightRPData();
	this->setLeftCSs();
	this->setRightCSs();
	this->setLeftProfile_Derivate();
	this->setRightProfile_Derivate();
	this->setProfileCS(sm0,sm1,sm2);
}

void Track::InitCurvature(void)
{
	this->setPcurMat();
	this->setSPcur();
	this->setcurDeltaS();
	this->setcurSp();
}
void Track::InitCurvature(int csm1, int csm2, int csm3)
{
	this->setPcurMat();
	this->setSPcur();
	this->setcurDeltaS();
	/*this->setcurSp();*/
	this->setcurSp(csm1, csm2, csm3);
}
//辅助函数
int Track::LocSegmentS(double S)
{
	if (S==0.0)
	{
		return 0;
	}
	int low=0;
	int high=m_nDiscription.Row()-2;
	int mid=0;
	while (low<=high)
	{
		mid=(low+high)/2;
		if ((S>m_nDiscription[mid][1])&&(S<=m_nDiscription[mid+1][1]))
		{
			return mid;
		}
		if (m_nDiscription[mid+1][1]<S)
		{
			low=mid+1;
		}
		if (m_nDiscription[mid][1]>=S)
		{
			high=mid-1;
		}
	}
	return -1;
}
int Track::LocSegments(double s)
{
	if (s==0.0)
	{
		return 0;
	}
	int low=0;
	int high=m_nDiscription.Row()-2;
	int mid=0;
	while(low<=high)
	{
		mid = (low + high) / 2;
		if ((s>m_nConPnt[mid][4])&&(s<=m_nConPnt[mid+1][4]))
		{
			return mid;
		}
		if (m_nConPnt[mid+1][4]<s)
		{
			low=mid+1;
		}
		if (m_nConPnt[mid][4]>=s)
		{
			high=mid-1;
		}
	}
	return -1;
}
double Track::TranS2s(double S)
{
	int loc=this->LocSegmentS(S);
	double tmp=0.0;
	if (loc!=-1)
	{
		double slope=(m_nConPnt[loc+1][3]-m_nConPnt[loc][3])/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]);
		tmp=m_nConPnt[loc][4]+(S-m_nConPnt[loc][0])/(cos(atan(slope)));
		return tmp;
	}
	else
	{
		return -1;
	}
}
double Track::Trans2S(double s)
{
	int loc=this->LocSegments(s);
	double tmp=0.0;
	if (loc!=-1)
	{
		double slope=(m_nConPnt[loc+1][3]-m_nConPnt[loc][3])/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]);
		tmp=m_nConPnt[loc][0]+(s-m_nConPnt[loc][4])*(cos(atan(slope)));
		return tmp;
	}
	else
	{
		return -1;
	}
}
double Track::getCH(double S)
{
	int loc=this->LocSegmentS(S);
	double C1=m_nDiscription[loc+1][2];
	double C0=m_nDiscription[loc][2];
	double S1=m_nDiscription[loc+1][1];
	double S0=m_nDiscription[loc][1];
	return (C1*(S-S0)-C0*(S-S1))/(S1-S0);
}
//轨道中心线函数//
double Track::getRPx(double S)
{
	int loc=this->LocSegmentS(S);
	//曲线段高斯5点积分
	double conven_delta=GAUSS_INTE; //设定gauss积分误差限
	double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
	double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
	double inte_interval=S-m_nConPnt[loc][0];
	double inte1=0.0;
	double inte2=0.0;
	double low,high,tmp,coff,angle;
	//初始进行积分域分两段
	int sub=2;
	for (int k=0;k<sub;k++)
	{
		low=m_nConPnt[loc][0]+k*inte_interval/sub;
		high=m_nConPnt[loc][0]+(k+1)*inte_interval/sub;
		tmp=0.0;
		for (int j=0;j<5;j++)
		{
			coff=((high-low)/2)*intpnt[j]+((high+low)/2);
			angle=this->getRPYaw(coff);
			tmp+=weight[j]*((high-low)/2)*cos(angle);
		}
		inte2+=tmp;
	}
	while (abs(inte2-inte1)>conven_delta)
	{
		inte1=inte2;
		sub*=2;
		inte2=0.0;
		for (int k=0;k<sub;k++)
		{
			low=m_nConPnt[loc][0]+k*inte_interval/sub;
			high=m_nConPnt[loc][0]+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				angle=this->getRPYaw(coff);
				tmp+=weight[j]*((high-low)/2)*cos(angle);
			}
			inte2+=tmp;
		}
	}
		return m_nCPxyz[loc][0]+inte2;
}
double Track::getRPy(double S)
{
	int loc=this->LocSegmentS(S);
	//曲线段高斯5点积分
	double conven_delta = GAUSS_INTE; //设定gauss积分误差限
	double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
	double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
	double inte_interval=S-m_nConPnt[loc][0];
	double inte1=0.0;
	double inte2=0.0;

	double low,high,tmp,coff,angle;
	//初始进行积分域分两段
	int sub=2;
	for (int k=0;k<sub;k++)
	{
		low=m_nConPnt[loc][0]+k*inte_interval/sub;
		high=m_nConPnt[loc][0]+(k+1)*inte_interval/sub;
		tmp=0.0;
		for (int j=0;j<5;j++)
		{
			coff=((high-low)/2)*intpnt[j]+((high+low)/2);
			angle=this->getRPYaw(coff);
			tmp+=weight[j]*((high-low)/2)*sin(angle);
		}
		inte2+=tmp;
	}
	while (abs(inte2-inte1)>conven_delta)
	{
		inte1=inte2;
		sub*=2;
		inte2=0.0;
		for (int k=0;k<sub;k++)
		{
			low=m_nConPnt[loc][0]+k*inte_interval/sub;
			high=m_nConPnt[loc][0]+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				angle=this->getRPYaw(coff);
				tmp+=weight[j]*((high-low)/2)*sin(angle);
			}
			inte2+=tmp;
		}
	}
	return m_nCPxyz[loc][1]+inte2;
}
double Track::getRPz(double S)
{
	int loc=this->LocSegmentS(S);
	if (m_nConPnt[loc][3]==m_nConPnt[loc+1][3])
	{
		return m_nCPxyz[loc][2];
	}
	else
	{
		double theta=atan((m_nConPnt[loc+1][3]-m_nConPnt[loc][3])/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]));
		return m_nCPxyz[loc][2]+(S-m_nConPnt[loc][0])*sin(theta);
	}
}

//返回映射弧长坐标系下轨道中心线坐标系摇头角
double Track::getRPYaw(double S)
{
	int loc=this->LocSegmentS(S);
	double tmp=0;
	tmp=m_nConPnt[loc][1]+(1/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]))*((m_nDiscription[loc+1][2]/2)*pow((S-m_nConPnt[loc][0]),2)-
		(m_nDiscription[loc][2]/2)*pow((S-m_nConPnt[loc+1][0]),2))+(m_nDiscription[loc][2]/2)*(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]);
	return tmp;
}

//返回映射弧长坐标系下轨道中心线点头角
double Track::getRPPitch(double S)
{
	int loc=this->LocSegmentS(S);
	double tmp=atan((m_nConPnt[loc+1][3]-m_nConPnt[loc][3])/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]));
	return tmp;
}

//返回映射弧长坐标系下轨道中心线侧滚角
double Track::getRPRoll(double S)
{
	int loc=this->LocSegmentS(S);
	double tmp=0;
	tmp=((m_nConPnt[loc+1][2]*(S-m_nConPnt[loc][0]))-(m_nConPnt[loc][2]*(S-m_nConPnt[loc+1][0])))/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]);
	return tmp;
}

//轨道中心线转动矩阵
Matrix Track::getTrackRotMat(double s)
{
	double ProjS=this->Trans2S(s);
	double psi=this->getRPYaw(ProjS);
	double theta=this->getRPPitch(ProjS);
	double phi=this->getRPRoll(ProjS);
	Matrix tmpMx(3,3);
	tmpMx[0][0]=cos(psi)*cos(theta);
	tmpMx[0][1]=(-1)*sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
	tmpMx[0][2]=(-1)*sin(psi)*sin(phi)-cos(psi)*sin(theta)*cos(phi);
	tmpMx[1][0]=sin(psi)*cos(theta);
	tmpMx[1][1]=cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
	tmpMx[1][2]=cos(psi)*sin(phi)-sin(psi)*sin(theta)*cos(phi);
	tmpMx[2][0]=sin(theta);
	tmpMx[2][1]=(-1)*cos(theta)*sin(phi);
	tmpMx[2][2]=cos(theta)*cos(phi);
	return tmpMx;
}
//轨道中心线切线矢量
Vector3x Track::getTangentVec(double s)
{
	double ProjS=this->Trans2S(s);
	double psi=this->getRPYaw(ProjS);
	double theta=this->getRPPitch(ProjS);
	double phi=this->getRPRoll(ProjS);
	Vector3x vec3;
	vec3[0]=cos(psi)*cos(theta);
	vec3[1]=sin(psi)*cos(theta);
	vec3[2]=sin(theta);
	return vec3;
}
//左轨函数//
//<1>左轨型面坐标系空间矢量
Vector3x Track::LeftRailRP(double s)
{
	Vector3x locvec3(0,gauge_width/2,0);
	double ProjS=this->Trans2S(s);
	Vector3x vec3Ref(this->getRPx(ProjS),this->getRPy(ProjS),this->getRPz(ProjS));
	Vector3x global = this->getTrackRotMat(s)*locvec3;
	Vector3x vec=vec3Ref+global;
	return vec;
}
//<2>左轨型面转动欧拉角 摇头->点头->侧滚 3->2->1
Vector3x Track::LeftRailAngle(double s)
{
	double ProjS=this->Trans2S(s);
	double psi=this->getRPYaw(ProjS);
	double phi=this->getRPRoll(ProjS);
	int loc=this->LocSegmentS(ProjS);
	double psi_slope=(m_nConPnt[loc+1][2]-m_nConPnt[loc][2])/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]);//轨道中心线侧滚角变化量与映射弧长的比值
	double delta_theta=atan((gauge_width/2*cos(phi))*psi_slope);//dphi(S)/dS
	double theta=this->getRPPitch(ProjS)-delta_theta;//dw/dS=cos(phi(S))*dphi(S)/dS 非线性曲线的切线矢量对应的角度值。
	Vector3x vec3(psi,theta,phi);
	return vec3;
}

//<3>左轨型面转动矩阵
Matrix Track::LeftRailRotMat(double s)
{
	Vector3x vec=this->LeftRailAngle(s);
	double psi=vec[0];
	double theta=vec[1];
	double phi=vec[2];
	Matrix tmpMx(3,3);
	tmpMx[0][0]=cos(psi)*cos(theta);
	tmpMx[0][1]=(-1)*sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
	tmpMx[0][2]=(-1)*sin(psi)*sin(phi)-cos(psi)*sin(theta)*cos(phi);
	tmpMx[1][0]=sin(psi)*cos(theta);
	tmpMx[1][1]=cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
	tmpMx[1][2]=cos(psi)*sin(phi)-sin(psi)*sin(theta)*cos(phi);
	tmpMx[2][0]=sin(theta);
	tmpMx[2][1]=(-1)*cos(theta)*sin(phi);
	tmpMx[2][2]=cos(theta)*cos(phi);
	return tmpMx;
}
//<4>左轨型面切线矢量
Vector3x Track::LeftRailTanVec(double s)
{
	Matrix tmpmx=this->LeftRailRotMat(s);
	Vector3x vec3(tmpmx[0][0],tmpmx[1][0],tmpmx[2][0]);
	return vec3;
}
//<5>左轨弧长差
double Track::getDiffArcLenLeft(double s)
{
	double S=this->Trans2S(s);
	int jj=this->LocSegmentS(S);
	
	double conven_delta = GAUSS_INTE; //设定gauss积分误差限
	double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
	double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
				
	double inte_interval=S-m_nConPnt[jj][0];//两控制点间的积分区间
	double inte1=0.0;
	double inte2=0.0;
	double low,high,tmp,coff,CH,phi,theta;
	//初始进行积分域分两段
	int sub=2;
	for (int k=0;k<sub;k++)
	{
		low=m_nConPnt[jj][0]+k*inte_interval/sub;
		high=m_nConPnt[jj][0]+(k+1)*inte_interval/sub;
		tmp=0.0;
		for (int j=0;j<5;j++)
		{
			coff=((high-low)/2)*intpnt[j]+((high+low)/2);
			CH=this->getCH(coff);
			phi=this->getRPRoll(coff);
			Vector3x vec3=this->LeftRailAngle(this->TranS2s(coff));
			theta=vec3[1];
			tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
		}
		inte2+=tmp;
	}
	while (abs(inte2-inte1)>conven_delta)
	{
		inte1=inte2;
		sub*=2;
		inte2=0.0;
		for (int k=0;k<sub;k++)
		{
			low=m_nConPnt[jj][0]+k*inte_interval/sub;
			high=m_nConPnt[jj][0]+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				CH=this->getCH(coff);
				phi=this->getRPRoll(coff);
				Vector3x vec3=this->LeftRailAngle(this->TranS2s(coff));
				theta=vec3[1];
				tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
			}
			inte2+=tmp;
		}
	}
	return (m_nArcLenDiffLeft[jj][1]+0.5*inte2);
}
Matrix Track::LeftRotationMat(double sl1)
{
	double psi=LeftRailCS[3].ValueAtC(sl1);
	double theta=LeftRailCS[4].ValueAtC(sl1);
	double phi=LeftRailCS[5].ValueAtC(sl1);
	Matrix tmpMx(3,3);
	tmpMx[0][0]=cos(psi)*cos(theta);
	tmpMx[0][1]=(-1)*sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
	tmpMx[0][2]=(-1)*sin(psi)*sin(phi)-cos(psi)*sin(theta)*cos(phi);
	tmpMx[1][0]=sin(psi)*cos(theta);
	tmpMx[1][1]=cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
	tmpMx[1][2]=cos(psi)*sin(phi)-sin(psi)*sin(theta)*cos(phi);
	tmpMx[2][0]=sin(theta);
	tmpMx[2][1]=(-1)*cos(theta)*sin(phi);
	tmpMx[2][2]=cos(theta)*cos(phi);
	return tmpMx;
}
Vector3x Track::LeftProfile_Local(double sl2)
{
	Vector3x Local(0,sl2,LProfileCS.ValueAt(sl2));
	return Local;
}
Vector3x Track::LeftRP_global(double sl1)
{
	Vector3x vec3(LeftRailCS[0].ValueAtC(sl1),LeftRailCS[1].ValueAtC(sl1),LeftRailCS[2].ValueAtC(sl1));
	return vec3;
}
Vector3x Track::LeftPositon(double sl1, double sl2)
{
	Matrix RotMat=this->LeftRotationMat(sl1);
	Vector3x Local=this->LeftProfile_Local(sl2);
	Vector3x local_tmp=RotMat*Local;
	Vector3x global_tmp=this->LeftRP_global(sl1);
	Vector3x LP=global_tmp+local_tmp;
	return LP;
}
Vector3x Track::LeftRail_T1(double sl1, double sl2)
{
	Vector3x Rd(LRProfDeri_1st[0].ValueAtC(sl1),LRProfDeri_1st[1].ValueAtC(sl1),LRProfDeri_1st[2].ValueAtC(sl1));
	Vector3x leftt1 = Rd + this->getLeftAd(sl1,sl2)*this->LeftProfile_Local(sl2);
	return leftt1;
}
Vector3x Track::LeftRail_T2(double sl1, double sl2)
{
	Vector3x vec3(0,1, LProfile_D1st.ValueAt(sl2));//B spline used
	Vector3x vec = this->LeftRotationMat(sl1)*vec3;
	return vec;
}
Vector3x Track::LeftRail_N(double sl1, double sl2)
{
	Vector3x T1=this->LeftRail_T1(sl1,sl2);
	Vector3x T2=this->LeftRail_T2(sl1,sl2);
	Vector3x vec=T1.crossProduct(T2);
	return vec;
}
Vector3x Track::LeftRail_T1DS1(double sl1, double sl2)
{
	Vector3x Rdd(LRProfDeri_2nd[0].ValueAtC(sl1),LRProfDeri_2nd[1].ValueAtC(sl1),LRProfDeri_2nd[2].ValueAtC(sl1));//cubicspline
	Vector3x vec=Rdd+this->getLeftAdd(sl1,sl2)*this->LeftProfile_Local(sl2);
	return vec;
}
Vector3x Track::LeftRail_T1DS2(double sl1,double sl2)
{
	Vector3x vec3(0,1, LProfile_D1st.ValueAt(sl2));//b spline
	return this->getLeftAd(sl1,sl2)*vec3;
}
Vector3x Track::LeftRail_T1DS1DS1(double s11, double s12)
{
	Vector3x Rddd(LRProfDeri_3rd[0].ValueAtC(s11), LRProfDeri_3rd[1].ValueAtC(s11), LRProfDeri_3rd[2].ValueAtC(s11));//cubicspline

	double psi = LeftRailCS[3].ValueAtC(s11);
	double psid = LRProfDeri_1st[3].ValueAtC(s11);
	double psidd = LRProfDeri_2nd[3].ValueAtC(s11);//add psidd

	double theta = LeftRailCS[4].ValueAtC(s11);
	double thetad = LRProfDeri_1st[4].ValueAtC(s11);
	double thetadd = LRProfDeri_2nd[4].ValueAtC(s11);

	double phi = LeftRailCS[5].ValueAtC(s11);
	double phid = LRProfDeri_1st[5].ValueAtC(s11);
	double phidd = LRProfDeri_2nd[5].ValueAtC(s11);

	Matrix Adpsi(3, 3);
	Adpsi[0][0] = (-1)*sin(psi)*cos(theta);
	Adpsi[0][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsi[0][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsi[1][0] = cos(psi)*cos(theta);
	Adpsi[1][1] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsi[1][2] = (-1)*sin(psi)*sin(phi) - cos(psi)*sin(theta)*cos(phi);
	Adpsi[2][0] = 0;
	Adpsi[2][1] = 0;
	Adpsi[2][2] = 0;

	Matrix Adtheta(3, 3);
	Adtheta[0][0] = (-1)*cos(psi)*sin(theta);
	Adtheta[0][1] = cos(psi)*cos(theta)*sin(phi);
	Adtheta[0][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adtheta[1][0] = (-1)*sin(psi)*sin(theta);
	Adtheta[1][1] = sin(psi)*cos(theta)*sin(phi);
	Adtheta[1][2] = (-1)*sin(psi)*cos(theta)*cos(phi);
	Adtheta[2][0] = cos(theta);
	Adtheta[2][1] = sin(theta)*sin(phi);
	Adtheta[2][2] = (-1)*sin(theta)*cos(phi);

	Matrix Adphi(3, 3);
	Adphi[0][0] = 0;
	Adphi[0][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphi[0][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adphi[1][0] = 0;
	Adphi[1][1] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphi[1][2] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	Adphi[2][0] = 0;
	Adphi[2][1] = (-1)*cos(theta)*cos(phi);
	Adphi[2][2] = (-1)*cos(theta)*sin(phi);

	Matrix Adpsipsi(3, 3);
	Adpsipsi[0][0] = (-1)*cos(psi)*cos(theta);
	Adpsipsi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adpsipsi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsipsi[1][0] = (-1)*sin(psi)*cos(theta);
	Adpsipsi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsipsi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsipsi[2][0] = 0;
	Adpsipsi[2][1] = 0;
	Adpsipsi[2][2] = 0;

	Matrix Adthetatheta(3, 3);
	Adthetatheta[0][0] = (-1)*cos(psi)*cos(theta);
	Adthetatheta[0][1] = (-1)*cos(psi)*sin(theta)*sin(phi);
	Adthetatheta[0][2] = cos(psi)*sin(theta)*cos(phi);
	Adthetatheta[1][0] = (-1)*sin(psi)*cos(theta);
	Adthetatheta[1][1] = (-1)*sin(psi)*sin(theta)*sin(phi);
	Adthetatheta[1][2] = sin(psi)*sin(theta)*cos(phi);
	Adthetatheta[2][0] = (-1)*sin(theta);
	Adthetatheta[2][1] = cos(theta)*sin(phi);
	Adthetatheta[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adphiphi(3, 3);
	Adphiphi[0][0] = 0;
	Adphiphi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adphiphi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphiphi[1][0] = 0;
	Adphiphi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adphiphi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphiphi[2][0] = 0;
	Adphiphi[2][1] = cos(theta)*sin(phi);
	Adphiphi[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adpsitheta(3, 3);
	Adpsitheta[0][0] = sin(psi)*sin(theta);
	Adpsitheta[0][1] = (-1)*sin(psi)*cos(theta)*sin(phi);
	Adpsitheta[0][2] = sin(psi)*cos(theta)*cos(phi);
	Adpsitheta[1][0] = (-1)*cos(psi)*sin(theta);
	Adpsitheta[1][1] = cos(psi)*cos(theta)*sin(phi);
	Adpsitheta[1][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adpsitheta[2][0] = 0;
	Adpsitheta[2][1] = 0;
	Adpsitheta[2][2] = 0;

	Matrix Adpsiphi(3, 3);
	Adpsiphi[0][0] = 0;
	Adpsiphi[0][1] = cos(psi)*sin(phi) - sin(psi)*sin(theta)*cos(phi);
	Adpsiphi[0][2] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsiphi[1][0] = 0;
	Adpsiphi[1][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsiphi[1][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsiphi[2][0] = 0;
	Adpsiphi[2][1] = 0;
	Adpsiphi[2][2] = 0;

	Matrix Adthetaphi(3, 3);
	Adthetaphi[0][0] = 0;
	Adthetaphi[0][1] = cos(psi)*cos(theta)*cos(phi);
	Adthetaphi[0][2] = cos(psi)*cos(theta)*sin(phi);
	Adthetaphi[1][0] = 0;
	Adthetaphi[1][1] = sin(psi)*cos(theta)*cos(phi);
	Adthetaphi[1][2] = sin(psi)*cos(theta)*sin(phi);
	Adthetaphi[2][0] = 0;
	Adthetaphi[2][1] = sin(theta)*cos(phi);
	Adthetaphi[2][2] = sin(theta)*sin(phi);

	Matrix Addd = 3.0 * (Adpsipsi*psid + Adpsitheta*thetad + Adpsiphi*phid)*psidd; // simplified equations from PP116
	Vector3x vec = Rddd + Addd*this->LeftProfile_Local(s12);
	return vec;
}
Vector3x Track::LeftRail_T1DS1DS2(double sl1, double sl2)
{
	Vector3x vec3(0, 1, LProfile_D1st.ValueAt(sl2));//b spline
	return this->getLeftAdd(sl1,sl2)*vec3;
}
Vector3x Track::LeftRail_T1DS2DS2(double sl1, double sl2)
{
	Vector3x vec3(0, 0, LProfile_D2nd.ValueAt(sl2));//b spline
	return this->getLeftAd(sl1,sl2)*vec3;
}
Vector3x Track::LeftRail_T2DS1(double sl1, double sl2)
{
	Vector3x vec3(0,1, LProfile_D1st.ValueAt(sl2));//b spline
	return this->getLeftAd(sl1,sl2)*vec3;
}
Vector3x Track::LeftRail_T2DS2(double sl1, double sl2)
{
	Matrix RotMat=this->LeftRotationMat(sl1);
	Vector3x vec3(0,0, LProfile_D2nd.ValueAt(sl2));//b spline
	Vector3x vec = RotMat*vec3;
	return vec;
}
Vector3x Track::LeftRail_T2DS1DS1(double sl1, double sl2)
{
	Vector3x vec3(0, 1, LProfile_D1st.ValueAt(sl2));
	return this->getLeftAdd(sl1,sl2)*vec3;
}
Vector3x Track::LeftRail_T2DS1DS2(double sl1, double sl2)
{
	Vector3x vec3(0, 0, LProfile_D2nd.ValueAt(sl2));//20191025 from pp121 [0 s12 f(s12)] [0 1 D[f(s12)]] [0 0 D2[f(s12)]]
	return this->getLeftAd(sl1,sl2)*vec3;
}
Vector3x Track::LeftRail_T2DS2DS2(double s11, double s12)
{
	Matrix RotMat = this->LeftRotationMat(s11);
	Vector3x vec3(0, 0, LProfile_D2nd.SlopeAt(s12));//b spline
	Vector3x vec = RotMat*vec3;
	return vec;
}
Vector3x Track::LeftRail_NDS1(double sl1, double sl2)
{
	Vector3x vec1=this->LeftRail_T1DS1(sl1,sl2).crossProduct(this->LeftRail_T2(sl1,sl2));
	Vector3x vec2=this->LeftRail_T1(sl1,sl2).crossProduct(this->LeftRail_T2DS1(sl1,sl2));
	return vec1+vec2;
}
Vector3x Track::LeftRail_NDS2(double sl1, double sl2)
{
	Vector3x vec1=this->LeftRail_T1DS2(sl1,sl2).crossProduct(this->LeftRail_T2(sl1,sl2));
	Vector3x vec2=this->LeftRail_T1(sl1,sl2).crossProduct(this->LeftRail_T2DS2(sl1,sl2));
	return vec1+vec2;
}
Vector3x Track::LeftRail_NDS1DS1(double sl1, double sl2)
{
	Vector3x vec1 = this->LeftRail_T1DS1DS1(sl1, sl2).crossProduct(this->LeftRail_T2(sl1, sl2));
	Vector3x vec2 = this->LeftRail_T1DS1(sl1, sl2).crossProduct(this->LeftRail_T2DS1(sl1, sl2));
	Vector3x vec3 = this->LeftRail_T1(sl1, sl2).crossProduct(this->LeftRail_T2DS1DS1(sl1, sl2));
	return vec1 + 2 * vec2 + vec3;
}
Vector3x Track::LeftRail_NDS1DS2(double sl1, double sl2)
{
	Vector3x vec1 = this->LeftRail_T1DS1DS2(sl1, sl2).crossProduct(this->LeftRail_T2(sl1, sl2));
	Vector3x vec2 = this->LeftRail_T1DS1(sl1, sl2).crossProduct(this->LeftRail_T2DS2(sl1, sl2));
	Vector3x vec3 = this->LeftRail_T1DS2(sl1, sl2).crossProduct(this->LeftRail_T2DS1(sl1, sl2));
	Vector3x vec4 = this->LeftRail_T1(sl1, sl2).crossProduct(this->LeftRail_T2DS1DS2(sl1, sl2));
	return vec1 + vec2 + vec3 + vec4;
}
Vector3x Track::LeftRail_NDS2DS2(double sl1, double sl2)
{
	Vector3x vec1 = this->LeftRail_T1DS2DS2(sl1, sl2).crossProduct(this->LeftRail_T2(sl1, sl2));
	Vector3x vec2 = this->LeftRail_T1DS2(sl1, sl2).crossProduct(this->LeftRail_T2DS2(sl1, sl2));
	Vector3x vec3 = this->LeftRail_T1(sl1, sl2).crossProduct(this->LeftRail_T2DS2DS2(sl1, sl2));
	return vec1 + 2 * vec2 + vec3;
}
double Track::LeftRail_curvature(double sl2)
{
	double s=xindic[0].ValueAt(sl2);
	Vector3x vec(lcur[4].ValueAt(s),lcur[5].ValueAt(s),0);
	return vec.getNorm();
}

Matrix Track::getLeftCurvatureMat(void)
{
	int nrow = LProfile.Row();
	Matrix mx(nrow, 2);
	for (int i = 0; i < nrow; i++)
	{
		mx[i][0] = LProfile[i][0];
		mx[i][1] = this->LeftRail_curvature(mx[i][0]);
	}
	return mx;
}

//右轨函数////////////////////////////////////////////
//<1>右轨型面坐标系空间矢量
Vector3x Track::RightRailRP(double s)
{
	Vector3x locvec3(0,(-1)*gauge_width/2,0);
	double ProjS=this->Trans2S(s);
	Vector3x vec3Ref(this->getRPx(ProjS),this->getRPy(ProjS),this->getRPz(ProjS));
	Vector3x global=this->getTrackRotMat(s)*locvec3;
	Vector3x vec=vec3Ref+global;
	return vec;
}

//<2>右轨型面转动欧拉角 摇头->点头->侧滚
Vector3x Track::RightRailAngle(double s)
{
	double ProjS=this->Trans2S(s);
	double psi=this->getRPYaw(ProjS);
	double phi=this->getRPRoll(ProjS);
	int loc=this->LocSegmentS(ProjS);
	double psi_slope=(m_nConPnt[loc+1][2]-m_nConPnt[loc][2])/(m_nConPnt[loc+1][0]-m_nConPnt[loc][0]);
	double delta_theta=atan((gauge_width/2*cos(phi))*psi_slope);
	double theta=this->getRPPitch(ProjS)+delta_theta;
	Vector3x vec3(psi,theta,phi);
	return vec3;
}
//<3>右轨型面转动矩阵
Matrix Track::RightRailRotMat(double s)
{
	Vector3x vec=this->LeftRailAngle(s);
	double psi=vec[0];
	double theta=vec[1];
	double phi=vec[2];
	Matrix tmpMx(3,3);
	tmpMx[0][0]=cos(psi)*cos(theta);
	tmpMx[0][1]=(-1)*sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
	tmpMx[0][2]=(-1)*sin(psi)*sin(phi)-cos(psi)*sin(theta)*cos(phi);
	tmpMx[1][0]=sin(psi)*cos(theta);
	tmpMx[1][1]=cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
	tmpMx[1][2]=cos(psi)*sin(phi)-sin(psi)*sin(theta)*cos(phi);
	tmpMx[2][0]=sin(theta);
	tmpMx[2][1]=(-1)*cos(theta)*sin(phi);
	tmpMx[2][2]=cos(theta)*cos(phi);
	return tmpMx;
}

//<4>右轨型面切线矢量
Vector3x Track::RightRailTanVec(double s)
{
	Matrix tmpmx=this->LeftRailRotMat(s);
	Vector3x vec3(tmpmx[0][0],tmpmx[1][0],tmpmx[2][0]);
	return vec3;
}
//<5>右轨弧长差
double Track::getDiffArcLenRight(double s)
{
	double S=this->Trans2S(s);
	int jj=this->LocSegmentS(S);
	
	double conven_delta=GAUSS_INTE; //设定gauss积分误差限
	double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};//设定权重系数
	double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
				
	double inte_interval=S-m_nConPnt[jj][0];//两控制点间的积分区间
	double inte1=0.0;
	double inte2=0.0;
	double low,high,tmp,coff,CH,phi,theta;
	//初始进行积分域分两段
	int sub=2;
	for (int k=0;k<sub;k++)
	{
		low=m_nConPnt[jj][0]+k*inte_interval/sub;
		high=m_nConPnt[jj][0]+(k+1)*inte_interval/sub;
		tmp=0.0;
		for (int j=0;j<5;j++)
		{
			coff=((high-low)/2)*intpnt[j]+((high+low)/2);
			CH=this->getCH(coff);
			phi=this->getRPRoll(coff);
			Vector3x vec3=this->RightRailAngle(this->TranS2s(coff));
			theta=vec3[1];
			tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
		}
		inte2+=tmp;
	}
	while (abs(inte2-inte1)>conven_delta)
	{
		inte1=inte2;
		sub*=2;
		inte2=0.0;
		for (int k=0;k<sub;k++)
		{
			low=m_nConPnt[jj][0]+k*inte_interval/sub;
			high=m_nConPnt[jj][0]+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				CH=this->getCH(coff);
				phi=this->getRPRoll(coff);
				Vector3x vec3=this->RightRailAngle(this->TranS2s(coff));
				theta=vec3[1];
				tmp+=weight[j]*((high-low)/2)*CH*gauge_width*cos(phi)/cos(theta);
			}
			inte2+=tmp;
		}
	}
	return (m_nArcLenDiffRight[jj][1]+0.5*inte2);
}
Matrix Track::RightRotationMat(double sr1)
{
	double psi=RightRailCS[3].ValueAtC(sr1);
	double theta=RightRailCS[4].ValueAtC(sr1);
	double phi=RightRailCS[5].ValueAtC(sr1);
	Matrix tmpMx(3,3);
	tmpMx[0][0]=cos(psi)*cos(theta);
	tmpMx[0][1]=(-1)*sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
	tmpMx[0][2]=(-1)*sin(psi)*sin(phi)-cos(psi)*sin(theta)*cos(phi);
	tmpMx[1][0]=sin(psi)*cos(theta);
	tmpMx[1][1]=cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
	tmpMx[1][2]=cos(psi)*sin(phi)-sin(psi)*sin(theta)*cos(phi);
	tmpMx[2][0]=sin(theta);
	tmpMx[2][1]=(-1)*cos(theta)*sin(phi);
	tmpMx[2][2]=cos(theta)*cos(phi);
	return tmpMx;
}
Vector3x Track::RightProfile_Local(double sr2)
{
	Vector3x Local(0,sr2,RProfileCS.ValueAt(sr2));//b spline
	return Local;
}
Vector3x Track::RightRP_global(double sr1)
{
	Vector3x vec3(RightRailCS[0].ValueAtC(sr1),RightRailCS[1].ValueAtC(sr1),RightRailCS[2].ValueAtC(sr1));
	return vec3;
}
Vector3x Track::RightPosition(double sr1, double sr2)
{
	Matrix RotMat=this->RightRotationMat(sr1);
	Vector3x Local=this->RightProfile_Local(sr2);
	Vector3x local_tmp=RotMat*Local;
	Vector3x global_tmp=this->RightRP_global(sr1);
	Vector3x LP=global_tmp+local_tmp;
	return LP;
}

Vector3x Track::RightRail_T1(double sr1, double sr2)
{
	Vector3x Rd(RRProfDeri_1st[0].ValueAtC(sr1),RRProfDeri_1st[1].ValueAtC(sr1),RRProfDeri_1st[2].ValueAtC(sr1));
	Vector3x vec3=Rd+this->getRightAd(sr1,sr2)*this->RightProfile_Local(sr2);
	return vec3;
}

Vector3x Track::RightRail_T2(double sr1, double sr2)
{
	Vector3x localvec3(0,1, RProfile_D1st.ValueAt(sr2));//b spline
	Vector3x vec3 = this->RightRotationMat(sr1)*localvec3;
	return vec3;
}

Vector3x Track::RightRail_N(double sr1, double sr2)
{
	Vector3x T1=this->RightRail_T1(sr1,sr2);
	Vector3x T2=this->RightRail_T2(sr1,sr2);
	Vector3x vec=T1.crossProduct(T2);
	return vec;
}
Vector3x Track::RightRail_T1DS1(double sr1, double sr2)
{
	Vector3x Rdd(RRProfDeri_2nd[0].ValueAtC(sr1),RRProfDeri_2nd[1].ValueAtC(sr1),RRProfDeri_2nd[2].ValueAtC(sr1));
	Vector3x vec=Rdd+this->getRightAdd(sr1,sr2)*this->RightProfile_Local(sr2);
	return vec;
}
Vector3x Track::RightRail_T1DS2(double sr1, double sr2)
{
	Vector3x vec3(0,1, RProfile_D1st.ValueAt(sr2));//b spline
	return this->getRightAd(sr1,sr2)*vec3;
}
Vector3x Track::RightRail_T1DS1DS1(double sr1, double sr2)
{
	Vector3x Rddd(RRProfDeri_3rd[0].ValueAtC(sr1), RRProfDeri_3rd[1].ValueAtC(sr1), RRProfDeri_3rd[2].ValueAtC(sr1));

	double psi = RightRailCS[3].ValueAtC(sr1);
	double psid = RRProfDeri_1st[3].ValueAtC(sr1);
	double psidd = RRProfDeri_2nd[3].ValueAtC(sr1);

	double theta = RightRailCS[4].ValueAtC(sr1);
	double thetad = RRProfDeri_1st[4].ValueAtC(sr1);
	double thetadd = RRProfDeri_2nd[4].ValueAtC(sr1);

	double phi = RightRailCS[5].ValueAtC(sr1);
	double phid = RRProfDeri_1st[5].ValueAtC(sr1);
	double phidd = RRProfDeri_2nd[5].ValueAtC(sr1);

	Matrix Adpsi(3, 3);
	Adpsi[0][0] = (-1)*sin(psi)*cos(theta);
	Adpsi[0][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsi[0][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsi[1][0] = cos(psi)*cos(theta);
	Adpsi[1][1] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsi[1][2] = (-1)*sin(psi)*sin(phi) - cos(psi)*sin(theta)*cos(phi);
	Adpsi[2][0] = 0;
	Adpsi[2][1] = 0;
	Adpsi[2][2] = 0;

	Matrix Adtheta(3, 3);
	Adtheta[0][0] = (-1)*cos(psi)*sin(theta);
	Adtheta[0][1] = cos(psi)*cos(theta)*sin(phi);
	Adtheta[0][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adtheta[1][0] = (-1)*sin(psi)*sin(theta);
	Adtheta[1][1] = sin(psi)*cos(theta)*sin(phi);
	Adtheta[1][2] = (-1)*sin(psi)*cos(theta)*cos(phi);
	Adtheta[2][0] = cos(theta);
	Adtheta[2][1] = sin(theta)*sin(phi);
	Adtheta[2][2] = (-1)*sin(theta)*cos(phi);

	Matrix Adphi(3, 3);
	Adphi[0][0] = 0;
	Adphi[0][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphi[0][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adphi[1][0] = 0;
	Adphi[1][1] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphi[1][2] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	Adphi[2][0] = 0;
	Adphi[2][1] = (-1)*cos(theta)*cos(phi);
	Adphi[2][2] = (-1)*cos(theta)*sin(phi);

	Matrix Adpsipsi(3, 3);
	Adpsipsi[0][0] = (-1)*cos(psi)*cos(theta);
	Adpsipsi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adpsipsi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsipsi[1][0] = (-1)*sin(psi)*cos(theta);
	Adpsipsi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsipsi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsipsi[2][0] = 0;
	Adpsipsi[2][1] = 0;
	Adpsipsi[2][2] = 0;

	Matrix Adthetatheta(3, 3);
	Adthetatheta[0][0] = (-1)*cos(psi)*cos(theta);
	Adthetatheta[0][1] = (-1)*cos(psi)*sin(theta)*sin(phi);
	Adthetatheta[0][2] = cos(psi)*sin(theta)*cos(phi);
	Adthetatheta[1][0] = (-1)*sin(psi)*cos(theta);
	Adthetatheta[1][1] = (-1)*sin(psi)*sin(theta)*sin(phi);
	Adthetatheta[1][2] = sin(psi)*sin(theta)*cos(phi);
	Adthetatheta[2][0] = (-1)*sin(theta);
	Adthetatheta[2][1] = cos(theta)*sin(phi);
	Adthetatheta[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adphiphi(3, 3);
	Adphiphi[0][0] = 0;
	Adphiphi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adphiphi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphiphi[1][0] = 0;
	Adphiphi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adphiphi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphiphi[2][0] = 0;
	Adphiphi[2][1] = cos(theta)*sin(phi);
	Adphiphi[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adpsitheta(3, 3);
	Adpsitheta[0][0] = sin(psi)*sin(theta);
	Adpsitheta[0][1] = (-1)*sin(psi)*cos(theta)*sin(phi);
	Adpsitheta[0][2] = sin(psi)*cos(theta)*cos(phi);
	Adpsitheta[1][0] = (-1)*cos(psi)*sin(theta);
	Adpsitheta[1][1] = cos(psi)*cos(theta)*sin(phi);
	Adpsitheta[1][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adpsitheta[2][0] = 0;
	Adpsitheta[2][1] = 0;
	Adpsitheta[2][2] = 0;

	Matrix Adpsiphi(3, 3);
	Adpsiphi[0][0] = 0;
	Adpsiphi[0][1] = cos(psi)*sin(phi) - sin(psi)*sin(theta)*cos(phi);
	Adpsiphi[0][2] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsiphi[1][0] = 0;
	Adpsiphi[1][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsiphi[1][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsiphi[2][0] = 0;
	Adpsiphi[2][1] = 0;
	Adpsiphi[2][2] = 0;

	Matrix Adthetaphi(3, 3);
	Adthetaphi[0][0] = 0;
	Adthetaphi[0][1] = cos(psi)*cos(theta)*cos(phi);
	Adthetaphi[0][2] = cos(psi)*cos(theta)*sin(phi);
	Adthetaphi[1][0] = 0;
	Adthetaphi[1][1] = sin(psi)*cos(theta)*cos(phi);
	Adthetaphi[1][2] = sin(psi)*cos(theta)*sin(phi);
	Adthetaphi[2][0] = 0;
	Adthetaphi[2][1] = sin(theta)*cos(phi);
	Adthetaphi[2][2] = sin(theta)*sin(phi);

	Matrix Addd = 3.0 * (Adpsipsi*psid + Adpsitheta*thetad + Adpsiphi*phid)*psidd; // simplified equations from PP116
	Vector3x vec = Rddd + Addd*this->RightProfile_Local(sr2);
	return vec;
}
Vector3x Track::RightRail_T1DS1DS2(double sr1, double sr2)
{
	Vector3x vec3(0, 1, RProfile_D1st.ValueAt(sr2));//b spline
	return this->getRightAdd(sr1,sr2)*vec3;
}
Vector3x Track::RightRail_T1DS2DS2(double sr1, double sr2)
{
	Vector3x vec3(0, 0, RProfile_D2nd.ValueAt(sr2));
	return this->getRightAd(sr1,sr2)*vec3;
}
Vector3x Track::RightRail_T2DS1(double sr1, double sr2)
{
	Vector3x vec3(0,1, RProfile_D1st.ValueAt(sr2));//b spline
	return this->getRightAd(sr1,sr2)*vec3;
}
Vector3x Track::RightRail_T2DS2(double sr1, double sr2)
{
	Matrix RotMat=this->RightRotationMat(sr1);
	Vector3x vec3(0,0, RProfile_D2nd.ValueAt(sr2));
	Vector3x vec = RotMat*vec3;
	return vec;
}
Vector3x Track::RightRail_T2DS1DS2(double sr1, double sr2)
{
	Vector3x vec3(0, 0, RProfile_D2nd.ValueAt(sr2));//b spline
	return this->getRightAd(sr1,sr2)*vec3;
}
Vector3x Track::RightRail_T2DS2DS2(double sr1, double sr2)
{
	Matrix RotMat = this->RightRotationMat(sr1);
	Vector3x vec3(0, 0, RProfile_D2nd.SlopeAt(sr2)); // b spline
	Vector3x vec = RotMat*vec3;
	return vec;
}
Vector3x Track::RightRail_T2DS1DS1(double sr1, double sr2)
{
	Vector3x localvec3(0, 1, RProfile_D1st.ValueAt(sr2));//b spline
	return this->getRightAdd(sr1,sr2)*localvec3;
}
Vector3x Track::RightRail_NDS1(double sr1, double sr2)
{
	Vector3x vec1=this->RightRail_T1DS1(sr1,sr2).crossProduct(this->RightRail_T2(sr1,sr2));
	Vector3x vec2=this->RightRail_T1(sr1,sr2).crossProduct(this->RightRail_T2DS1(sr1,sr2));
	return vec1+vec2;
}
Vector3x Track::RightRail_NDS2(double sr1, double sr2)
{
	Vector3x vec1=this->RightRail_T1DS2(sr1,sr2).crossProduct(this->RightRail_T2(sr1,sr2));
	Vector3x vec2=this->RightRail_T1(sr1,sr2).crossProduct(this->RightRail_T2DS2(sr1,sr2));
	return vec1+vec2;
}
double Track::RightRail_curvature(double sr2)
{
	double s=xindic[1].ValueAt(sr2);
	Vector3x vec(rcur[4].ValueAt(s),rcur[5].ValueAt(s),0);
	return vec.getNorm();
}

Matrix Track::getRightCurvatureMat(void)
{
	int nrow = RProfile.Row();
	Matrix mx(nrow, 2);
	for (int i = 0; i < nrow; i++)
	{
		mx[i][0] = RProfile[i][0];
		mx[i][1] = this->RightRail_curvature(mx[i][0]);
	}
	return mx;
}

Vector3x Track::RightRail_NDS1DS1(double sr1, double sr2)
{
	Vector3x vec1 = this->RightRail_T1DS1DS1(sr1, sr2).crossProduct(this->RightRail_T2(sr1, sr2));
	Vector3x vec2 = this->RightRail_T1DS1(sr1, sr2).crossProduct(this->RightRail_T2DS1(sr1, sr2));
	Vector3x vec3 = this->RightRail_T1(sr1, sr2).crossProduct(this->RightRail_T2DS1DS1(sr1, sr2));
	return vec1 + 2 * vec2 + vec3;
}
Vector3x Track::RightRail_NDS1DS2(double sr1, double sr2)
{
	Vector3x vec1 = this->RightRail_T1DS1DS2(sr1, sr2).crossProduct(this->RightRail_T2(sr1, sr2));
	Vector3x vec2 = this->RightRail_T1DS1(sr1, sr2).crossProduct(this->RightRail_T2DS2(sr1, sr2));
	Vector3x vec3 = this->RightRail_T1DS2(sr1, sr2).crossProduct(this->RightRail_T2DS1(sr1, sr2));
	Vector3x vec4 = this->RightRail_T1(sr1, sr2).crossProduct(this->RightRail_T2DS1DS2(sr1, sr2));
	return vec1 + vec2 + vec3 + vec4;
}
Vector3x Track::RightRail_NDS2DS2(double sr1, double sr2)
{
	Vector3x vec1 = this->RightRail_T1DS2DS2(sr1, sr2).crossProduct(this->RightRail_T2(sr1, sr2));
	Vector3x vec2 = this->RightRail_T1DS2(sr1, sr2).crossProduct(this->RightRail_T2DS2(sr1, sr2));
	Vector3x vec3 = this->RightRail_T1(sr1, sr2).crossProduct(this->RightRail_T2DS2DS2(sr1, sr2));
	return vec1 + 2 * vec2 + vec3;
}
Matrix Track::getLeftAd(double sl1, double sl2)
{
	double psi = LeftRailCS[3].ValueAtC(sl1);
	double psid = LRProfDeri_1st[3].ValueAtC(sl1);
	double theta = LeftRailCS[4].ValueAtC(sl1);
	double thetad = LRProfDeri_1st[4].ValueAtC(sl1);
	double phi = LeftRailCS[5].ValueAtC(sl1);
	double phid = LRProfDeri_1st[5].ValueAtC(sl1);

	Matrix Adpsi(3, 3);
	Adpsi[0][0] = (-1)*sin(psi)*cos(theta);
	Adpsi[0][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsi[0][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsi[1][0] = cos(psi)*cos(theta);
	Adpsi[1][1] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsi[1][2] = (-1)*sin(psi)*sin(phi) - cos(psi)*sin(theta)*cos(phi);
	Adpsi[2][0] = 0;
	Adpsi[2][1] = 0;
	Adpsi[2][2] = 0;

	Matrix Adtheta(3, 3);
	Adtheta[0][0] = (-1)*cos(psi)*sin(theta);
	Adtheta[0][1] = cos(psi)*cos(theta)*sin(phi);
	Adtheta[0][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adtheta[1][0] = (-1)*sin(psi)*sin(theta);
	Adtheta[1][1] = sin(psi)*cos(theta)*sin(phi);
	Adtheta[1][2] = (-1)*sin(psi)*cos(theta)*cos(phi);
	Adtheta[2][0] = cos(theta);
	Adtheta[2][1] = sin(theta)*sin(phi);
	Adtheta[2][2] = (-1)*sin(theta)*cos(phi);

	Matrix Adphi(3, 3);
	Adphi[0][0] = 0;
	Adphi[0][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphi[0][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adphi[1][0] = 0;
	Adphi[1][1] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphi[1][2] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	Adphi[2][0] = 0;
	Adphi[2][1] = (-1)*cos(theta)*cos(phi);
	Adphi[2][2] = (-1)*cos(theta)*sin(phi);

	Matrix mx = Adpsi*psid + Adtheta*thetad + Adphi*phid;
	return mx;
}
Matrix Track::getLeftAdd(double sl1, double sl2)
{
	double psi = LeftRailCS[3].ValueAtC(sl1);
	double psid = LRProfDeri_1st[3].ValueAtC(sl1);
	double psidd = LRProfDeri_2nd[3].ValueAtC(sl1);//add psidd

	double theta = LeftRailCS[4].ValueAtC(sl1);
	double thetad = LRProfDeri_1st[4].ValueAtC(sl1);
	double thetadd = LRProfDeri_2nd[4].ValueAtC(sl1);

	double phi = LeftRailCS[5].ValueAtC(sl1);
	double phid = LRProfDeri_1st[5].ValueAtC(sl1);
	double phidd = LRProfDeri_2nd[5].ValueAtC(sl1);

	Matrix Adpsi(3, 3);
	Adpsi[0][0] = (-1)*sin(psi)*cos(theta);
	Adpsi[0][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsi[0][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsi[1][0] = cos(psi)*cos(theta);
	Adpsi[1][1] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsi[1][2] = (-1)*sin(psi)*sin(phi) - cos(psi)*sin(theta)*cos(phi);
	Adpsi[2][0] = 0;
	Adpsi[2][1] = 0;
	Adpsi[2][2] = 0;

	Matrix Adtheta(3, 3);
	Adtheta[0][0] = (-1)*cos(psi)*sin(theta);
	Adtheta[0][1] = cos(psi)*cos(theta)*sin(phi);
	Adtheta[0][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adtheta[1][0] = (-1)*sin(psi)*sin(theta);
	Adtheta[1][1] = sin(psi)*cos(theta)*sin(phi);
	Adtheta[1][2] = (-1)*sin(psi)*cos(theta)*cos(phi);
	Adtheta[2][0] = cos(theta);
	Adtheta[2][1] = sin(theta)*sin(phi);
	Adtheta[2][2] = (-1)*sin(theta)*cos(phi);

	Matrix Adphi(3, 3);
	Adphi[0][0] = 0;
	Adphi[0][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphi[0][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adphi[1][0] = 0;
	Adphi[1][1] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphi[1][2] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	Adphi[2][0] = 0;
	Adphi[2][1] = (-1)*cos(theta)*cos(phi);
	Adphi[2][2] = (-1)*cos(theta)*sin(phi);

	Matrix Adpsipsi(3, 3);
	Adpsipsi[0][0] = (-1)*cos(psi)*cos(theta);
	Adpsipsi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adpsipsi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsipsi[1][0] = (-1)*sin(psi)*cos(theta);
	Adpsipsi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsipsi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsipsi[2][0] = 0;
	Adpsipsi[2][1] = 0;
	Adpsipsi[2][2] = 0;

	Matrix Adthetatheta(3, 3);
	Adthetatheta[0][0] = (-1)*cos(psi)*cos(theta);
	Adthetatheta[0][1] = (-1)*cos(psi)*sin(theta)*sin(phi);
	Adthetatheta[0][2] = cos(psi)*sin(theta)*cos(phi);
	Adthetatheta[1][0] = (-1)*sin(psi)*cos(theta);
	Adthetatheta[1][1] = (-1)*sin(psi)*sin(theta)*sin(phi);
	Adthetatheta[1][2] = sin(psi)*sin(theta)*cos(phi);
	Adthetatheta[2][0] = (-1)*sin(theta);
	Adthetatheta[2][1] = cos(theta)*sin(phi);
	Adthetatheta[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adphiphi(3, 3);
	Adphiphi[0][0] = 0;
	Adphiphi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adphiphi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphiphi[1][0] = 0;
	Adphiphi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adphiphi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphiphi[2][0] = 0;
	Adphiphi[2][1] = cos(theta)*sin(phi);
	Adphiphi[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adpsitheta(3, 3);
	Adpsitheta[0][0] = sin(psi)*sin(theta);
	Adpsitheta[0][1] = (-1)*sin(psi)*cos(theta)*sin(phi);
	Adpsitheta[0][2] = sin(psi)*cos(theta)*cos(phi);
	Adpsitheta[1][0] = (-1)*cos(psi)*sin(theta);
	Adpsitheta[1][1] = cos(psi)*cos(theta)*sin(phi);
	Adpsitheta[1][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adpsitheta[2][0] = 0;
	Adpsitheta[2][1] = 0;
	Adpsitheta[2][2] = 0;

	Matrix Adpsiphi(3, 3);
	Adpsiphi[0][0] = 0;
	Adpsiphi[0][1] = cos(psi)*sin(phi) - sin(psi)*sin(theta)*cos(phi);
	Adpsiphi[0][2] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsiphi[1][0] = 0;
	Adpsiphi[1][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsiphi[1][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsiphi[2][0] = 0;
	Adpsiphi[2][1] = 0;
	Adpsiphi[2][2] = 0;

	Matrix Adthetaphi(3, 3);
	Adthetaphi[0][0] = 0;
	Adthetaphi[0][1] = cos(psi)*cos(theta)*cos(phi);
	Adthetaphi[0][2] = cos(psi)*cos(theta)*sin(phi);
	Adthetaphi[1][0] = 0;
	Adthetaphi[1][1] = sin(psi)*cos(theta)*cos(phi);
	Adthetaphi[1][2] = sin(psi)*cos(theta)*sin(phi);
	Adthetaphi[2][0] = 0;
	Adthetaphi[2][1] = sin(theta)*cos(phi);
	Adthetaphi[2][2] = sin(theta)*sin(phi);

	Matrix Add = Adpsi*psidd + Adtheta*thetadd + Adphi*phidd + Adpsipsi*psid*psid + Adthetatheta*thetad*thetad + Adphiphi*phid*phid +
		Adpsitheta*(2 * psid*thetad) + Adpsiphi*(2 * psid*phid) + Adthetaphi*(2 * thetad*phid);
	return Add;
}
Matrix Track::getRightAd(double sr1, double sr2)
{
	double psi = RightRailCS[3].ValueAtC(sr1);
	double psid = RRProfDeri_1st[3].ValueAtC(sr1);
	double theta = RightRailCS[4].ValueAtC(sr1);
	double thetad = RRProfDeri_1st[4].ValueAtC(sr1);
	double phi = RightRailCS[5].ValueAtC(sr1);
	double phid = RRProfDeri_1st[5].ValueAtC(sr1);

	Matrix Adpsi(3, 3);
	Adpsi[0][0] = (-1)*sin(psi)*cos(theta);
	Adpsi[0][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsi[0][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsi[1][0] = cos(psi)*cos(theta);
	Adpsi[1][1] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsi[1][2] = (-1)*sin(psi)*sin(phi) - cos(psi)*sin(theta)*cos(phi);
	Adpsi[2][0] = 0;
	Adpsi[2][1] = 0;
	Adpsi[2][2] = 0;

	Matrix Adtheta(3, 3);
	Adtheta[0][0] = (-1)*cos(psi)*sin(theta);
	Adtheta[0][1] = cos(psi)*cos(theta)*sin(phi);
	Adtheta[0][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adtheta[1][0] = (-1)*sin(psi)*sin(theta);
	Adtheta[1][1] = sin(psi)*cos(theta)*sin(phi);
	Adtheta[1][2] = (-1)*sin(psi)*cos(theta)*cos(phi);
	Adtheta[2][0] = cos(theta);
	Adtheta[2][1] = sin(theta)*sin(phi);
	Adtheta[2][2] = (-1)*sin(theta)*cos(phi);

	Matrix Adphi(3, 3);
	Adphi[0][0] = 0;
	Adphi[0][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphi[0][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adphi[1][0] = 0;
	Adphi[1][1] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphi[1][2] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	Adphi[2][0] = 0;
	Adphi[2][1] = (-1)*cos(theta)*cos(phi);
	Adphi[2][2] = (-1)*cos(theta)*sin(phi);

	Matrix mx = Adpsi*psid + Adtheta*thetad + Adphi*phid;
	return mx;
}
Matrix Track::getRightAdd(double sr1, double sr2)
{
	double psi = RightRailCS[3].ValueAtC(sr1);
	double psid = RRProfDeri_1st[3].ValueAtC(sr1);
	double psidd = RRProfDeri_2nd[3].ValueAtC(sr1);

	double theta = RightRailCS[4].ValueAtC(sr1);
	double thetad = RRProfDeri_1st[4].ValueAtC(sr1);
	double thetadd = RRProfDeri_2nd[4].ValueAtC(sr1);

	double phi = RightRailCS[5].ValueAtC(sr1);
	double phid = RRProfDeri_1st[5].ValueAtC(sr1);
	double phidd = RRProfDeri_2nd[5].ValueAtC(sr1);

	Matrix Adpsi(3, 3);
	Adpsi[0][0] = (-1)*sin(psi)*cos(theta);
	Adpsi[0][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsi[0][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsi[1][0] = cos(psi)*cos(theta);
	Adpsi[1][1] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsi[1][2] = (-1)*sin(psi)*sin(phi) - cos(psi)*sin(theta)*cos(phi);
	Adpsi[2][0] = 0;
	Adpsi[2][1] = 0;
	Adpsi[2][2] = 0;

	Matrix Adtheta(3, 3);
	Adtheta[0][0] = (-1)*cos(psi)*sin(theta);
	Adtheta[0][1] = cos(psi)*cos(theta)*sin(phi);
	Adtheta[0][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adtheta[1][0] = (-1)*sin(psi)*sin(theta);
	Adtheta[1][1] = sin(psi)*cos(theta)*sin(phi);
	Adtheta[1][2] = (-1)*sin(psi)*cos(theta)*cos(phi);
	Adtheta[2][0] = cos(theta);
	Adtheta[2][1] = sin(theta)*sin(phi);
	Adtheta[2][2] = (-1)*sin(theta)*cos(phi);

	Matrix Adphi(3, 3);
	Adphi[0][0] = 0;
	Adphi[0][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphi[0][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adphi[1][0] = 0;
	Adphi[1][1] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphi[1][2] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
	Adphi[2][0] = 0;
	Adphi[2][1] = (-1)*cos(theta)*cos(phi);
	Adphi[2][2] = (-1)*cos(theta)*sin(phi);

	Matrix Adpsipsi(3, 3);
	Adpsipsi[0][0] = (-1)*cos(psi)*cos(theta);
	Adpsipsi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adpsipsi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsipsi[1][0] = (-1)*sin(psi)*cos(theta);
	Adpsipsi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsipsi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adpsipsi[2][0] = 0;
	Adpsipsi[2][1] = 0;
	Adpsipsi[2][2] = 0;

	Matrix Adthetatheta(3, 3);
	Adthetatheta[0][0] = (-1)*cos(psi)*cos(theta);
	Adthetatheta[0][1] = (-1)*cos(psi)*sin(theta)*sin(phi);
	Adthetatheta[0][2] = cos(psi)*sin(theta)*cos(phi);
	Adthetatheta[1][0] = (-1)*sin(psi)*cos(theta);
	Adthetatheta[1][1] = (-1)*sin(psi)*sin(theta)*sin(phi);
	Adthetatheta[1][2] = sin(psi)*sin(theta)*cos(phi);
	Adthetatheta[2][0] = (-1)*sin(theta);
	Adthetatheta[2][1] = cos(theta)*sin(phi);
	Adthetatheta[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adphiphi(3, 3);
	Adphiphi[0][0] = 0;
	Adphiphi[0][1] = sin(psi)*cos(phi) - cos(psi)*sin(theta)*sin(phi);
	Adphiphi[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adphiphi[1][0] = 0;
	Adphiphi[1][1] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adphiphi[1][2] = (-1)*cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
	Adphiphi[2][0] = 0;
	Adphiphi[2][1] = cos(theta)*sin(phi);
	Adphiphi[2][2] = (-1)*cos(theta)*cos(phi);

	Matrix Adpsitheta(3, 3);
	Adpsitheta[0][0] = sin(psi)*sin(theta);
	Adpsitheta[0][1] = (-1)*sin(psi)*cos(theta)*sin(phi);
	Adpsitheta[0][2] = sin(psi)*cos(theta)*cos(phi);
	Adpsitheta[1][0] = (-1)*cos(psi)*sin(theta);
	Adpsitheta[1][1] = cos(psi)*cos(theta)*sin(phi);
	Adpsitheta[1][2] = (-1)*cos(psi)*cos(theta)*cos(phi);
	Adpsitheta[2][0] = 0;
	Adpsitheta[2][1] = 0;
	Adpsitheta[2][2] = 0;

	Matrix Adpsiphi(3, 3);
	Adpsiphi[0][0] = 0;
	Adpsiphi[0][1] = cos(psi)*sin(phi) - sin(psi)*sin(theta)*cos(phi);
	Adpsiphi[0][2] = (-1)*cos(psi)*cos(phi) - sin(psi)*sin(theta)*sin(phi);
	Adpsiphi[1][0] = 0;
	Adpsiphi[1][1] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
	Adpsiphi[1][2] = (-1)*sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
	Adpsiphi[2][0] = 0;
	Adpsiphi[2][1] = 0;
	Adpsiphi[2][2] = 0;

	Matrix Adthetaphi(3, 3);
	Adthetaphi[0][0] = 0;
	Adthetaphi[0][1] = cos(psi)*cos(theta)*cos(phi);
	Adthetaphi[0][2] = cos(psi)*cos(theta)*sin(phi);
	Adthetaphi[1][0] = 0;
	Adthetaphi[1][1] = sin(psi)*cos(theta)*cos(phi);
	Adthetaphi[1][2] = sin(psi)*cos(theta)*sin(phi);
	Adthetaphi[2][0] = 0;
	Adthetaphi[2][1] = sin(theta)*cos(phi);
	Adthetaphi[2][2] = sin(theta)*sin(phi);

	Matrix Add = Adpsi*psidd + Adtheta*thetadd + Adphi*phidd + Adpsipsi*psid*psid + Adthetatheta*thetad*thetad + Adphiphi*phid*phid +
		Adpsitheta*(2 * psid*thetad) + Adpsiphi*(2 * psid*phid) + Adthetaphi*(2 * thetad*phid);
	return Add;
}