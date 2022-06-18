#pragma once

#ifndef _INERTIAELEM_H_
#define _INERTIAELEM_H_
#include "InertiaElem.h"
#endif

// InertiaElemHdr ÃüÁîÄ¿±ê

class InertiaElemHdr : public CObject
{
public:
	InertiaElemHdr():p(NULL),use(new int(1)) { }
	InertiaElemHdr(const InertiaElem& ie);
	InertiaElemHdr(const InertiaElemHdr& i):p(i.p),use(i.use) { ++*use; }

	virtual ~InertiaElemHdr();
	InertiaElemHdr& operator=(const InertiaElemHdr& i);

	InertiaElem *operator->(void) const { return p; }
	InertiaElem &operator*(void) const { return *p; }
	InertiaElem * getPointer(void) const {return p;}
private:
	void decr_use(void) { if (--*use==0) { delete p; delete use; } }
private:
	InertiaElem *p;
	int *use;
};
