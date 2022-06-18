#pragma once

#ifndef _FORCEELEM_H_
#define _FORCEELEM_H_
#include "ForceElem.h"
#endif


// ForceElemHdr ÃüÁîÄ¿±ê

class ForceElemHdr : public CObject
{
public:
	ForceElemHdr():p(NULL),use(new int(1)) { }
	ForceElemHdr(const ForceElem& fe);
	ForceElemHdr(const ForceElemHdr& fehdr):p(fehdr.p),use(fehdr.use) { ++*use; }
	virtual ~ForceElemHdr();

	ForceElemHdr& operator=(const ForceElemHdr& fehdr);
	ForceElem *operator->(void) const{return p;};//20130713 added
	ForceElem &operator&(void) const {return *p;};//20130713 added 

private:
	void decr_use(void) { if (--*use==0) { delete p; delete use; } }
private:
	ForceElem* p;
	int* use;
};


