#pragma once

#ifndef _DOFCONSTRAINT_H_
#define _DOFCONSTRAINT_H_
#include "DOFConstraint.h"
#endif

// DOFConstraintHdr ÃüÁîÄ¿±ê

class DOFConstraintHdr : public CObject
{
public:
	DOFConstraintHdr():p(NULL),use(new int(1)) {}
	DOFConstraintHdr(const DOFConstraint &dc);
	DOFConstraintHdr(const DOFConstraintHdr &dchdr):p(dchdr.p),use(dchdr.use) {}
	DOFConstraintHdr& operator=(const DOFConstraintHdr &dchdr);

	DOFConstraint* operator->(void) const { return p; }
	DOFConstraint& operator*(void) const { return *p;}
	virtual ~DOFConstraintHdr();
private:
	void decr_use(void) { if (--*use==0) { delete p; delete use; } }
private:
	DOFConstraint* p;
	int *use;
};


