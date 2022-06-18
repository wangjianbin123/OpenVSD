#pragma once
#include "Observer.h"

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

class InertiaElem;

class ResObserver :
	public Observer
{
public:
	ResObserver(void);
	ResObserver(int index,CString mname,int type);
	ResObserver(const ResObserver& src);
	ResObserver& operator=(const ResObserver& src);
	std::vector<VectorN> getKinematicsRes(void) const { return kinematics_res; }
	Observer* clone(void) const { return new ResObserver(*this); }
	void IntPntResPushBackprocess(InertiaElem &ie);
	void save(void);
	virtual ~ResObserver(void);
public:
	std::vector<VectorN> kinematics_res;
};

