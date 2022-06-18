// InertiaElemHdr.cpp : 实现文件
//
#include "pch.h"
#include "InertiaElemHdr.h"

#ifndef _INERTIAELEM_H_
#define _INERTIAELEM_H_
#include "InertiaElem.h"
#endif

// InertiaElemHdr

InertiaElemHdr::InertiaElemHdr(const InertiaElem &ie):p(ie.clone()),use(new int(1))
{
}
InertiaElemHdr& InertiaElemHdr::operator =(const InertiaElemHdr &i)
{
	++*i.use;
	decr_use();
	p=i.p;
	use=i.use;
	return *this;
}
InertiaElemHdr::~InertiaElemHdr()
{
	decr_use();
}


// InertiaElemHdr 成员函数
