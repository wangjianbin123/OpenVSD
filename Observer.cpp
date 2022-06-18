// Observer.cpp : implementation file
//
#include "pch.h"
#include "Observer.h"

#ifndef _INERTIAELEMHDR_H_
#define _INERTIAELEMHDR_H_
#include "InertiaElemHdr.h"
#endif


// Observer

Observer::Observer()
{
	m_index=0;
	m_name=" ";
	m_type=0;
}
Observer::Observer(int index,CString mname,int type)
{
	m_index=index;
	m_name=mname;
	m_type=type;
}
Observer::Observer(const Observer& src)
{
	m_index=src.getIndex();
	m_name=src.getName();
	m_type=src.getType();
}

Observer& Observer::operator=(const Observer &src)
{
	if (this==&src)
	{
		return *this;
	}
	else
	{
		m_index=src.getIndex();
		m_name=src.getName();
		m_type=src.getType();
		return *this;
	}
}
Observer::~Observer()
{
}

// Observer member functions
