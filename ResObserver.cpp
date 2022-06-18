#include "pch.h"
#include "ResObserver.h"

#ifndef _INERTIAELEMHDR_H_
#define _INERTIAELEMHDR_H_
#include "InertiaElemHdr.h"
#endif

ResObserver::ResObserver(void):Observer()
{
	kinematics_res.clear();
}
ResObserver::ResObserver(int index,CString mname,int type)
{
	m_index=index;
	m_name=mname;
	m_type=type;
	kinematics_res.clear();
}
ResObserver::ResObserver(const ResObserver& src)
{
	m_index=src.getIndex();
	m_name=src.getName();
	m_type=src.getType();
	kinematics_res=src.getKinematicsRes();
}
ResObserver& ResObserver::operator=(const ResObserver& src)
{
	if(this==&src)
	{
		return *this;
	}
	else
	{
		m_index=src.getIndex();
		m_name=src.getName();
		m_type=src.getType();
		kinematics_res=src.getKinematicsRes();
		return *this;
	}
}
void ResObserver::IntPntResPushBackprocess(InertiaElem &ie)
{
	kinematics_res.push_back(ie.getCurrentKinematicPara());
}
void ResObserver::save(void)
{
	int row=static_cast<int>(kinematics_res.size());
	int col=kinematics_res[0].Dim();
	Matrix resmat(row,col);
	for (int i=0;i<row;i++)
	{
		for (int j=0;j<col;j++)
		{
			resmat[i][j]=kinematics_res[i][j];
		}
	}
	resmat.SaveMatrix(this->getName()+".txt",1);
}
ResObserver::~ResObserver(void)
{
	kinematics_res.clear();
}
