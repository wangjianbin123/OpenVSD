#pragma once

// ObserverHdr command target
#ifndef _OBSERVER_H_
#define _OBSERVER_H_
#include "Observer.h"
#endif

class ObserverHdr : public CObject
{
public:
	ObserverHdr():p(NULL),use(new int(1)){ }
	ObserverHdr(const Observer &ob);
	ObserverHdr(const ObserverHdr &src):p(src.p),use(src.use) {++*use;}
	ObserverHdr& operator=(const ObserverHdr& i);

	Observer* operator->(void) const { return p; }
	Observer& operator*(void) const { return *p; }
	Observer* getPointer(void) const {return p;}
	virtual ~ObserverHdr();
private:
	void decr_use(void) { if (--*use==0) { delete p; delete use; } }
public:
	Observer* p;
	int* use;
};


