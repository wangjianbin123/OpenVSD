// ObserverHdr.cpp : implementation file
//
#include "pch.h"
#include "ObserverHdr.h"

// ObserverHdr

ObserverHdr::ObserverHdr(const Observer& ob)
{
	p=ob.clone();
	use=new int(1);
}
ObserverHdr& ObserverHdr::operator=(const ObserverHdr& i)
{
	++*i.use;
	decr_use();
	p=i.p;
	use=i.use;
	return *this;
}

ObserverHdr::~ObserverHdr()
{
	decr_use();
}
// ObserverHdr member functions
