// ForceElemHdr.cpp : ʵ���ļ�
//
#include "pch.h"
#include "ForceElemHdr.h"


// ForceElemHdr
ForceElemHdr::ForceElemHdr(const ForceElem &fe):p(fe.clone()),use(new int(1))
{
}
ForceElemHdr& ForceElemHdr::operator =(const ForceElemHdr &fehdr)
{
	++*fehdr.use;
	decr_use();
	p=fehdr.p;
	use=fehdr.use;
	return *this;
}
ForceElemHdr::~ForceElemHdr()
{
}


// ForceElemHdr ��Ա����
