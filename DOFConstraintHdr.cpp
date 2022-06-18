// DOFConstraintHdr.cpp : 实现文件
//
#include "pch.h"
#include "DOFConstraintHdr.h"

// DOFConstraintHdr

DOFConstraintHdr::DOFConstraintHdr(const DOFConstraint &dc):p(dc.clone()),use(new int(1))
{
}
DOFConstraintHdr& DOFConstraintHdr::operator =(const DOFConstraintHdr &dchdr)
{
	++*dchdr.use;
	decr_use();
	p=dchdr.p;
	use=dchdr.use;
	return *this;
}
DOFConstraintHdr::~DOFConstraintHdr()
{
	decr_use();
}


// DOFConstraintHdr 成员函数
