// DOFConstraint.cpp : 实现文件
//
#include "pch.h"
#include "DOFConstraint.h"


// DOFConstraint
//IMPLEMENT_SERIAL(CObject,DOFConstraint,VERSIONABLE_SCHEMA|2)

DOFConstraint::DOFConstraint(void):sn(0),m_name("")
{
	body_i=NULL;
	body_j=NULL;
}
DOFConstraint::DOFConstraint(int n, CString name,InertiaElem *bodyi,InertiaElem *bodyj):sn(n),m_name(name)
{
	body_i=bodyi;
	body_j=bodyj;
}
DOFConstraint::DOFConstraint(const DOFConstraint& dofcons)
{
	sn=dofcons.sn;
	m_name=dofcons.m_name;
	body_i=dofcons.body_i;
	body_j=dofcons.body_j;
}
DOFConstraint& DOFConstraint::operator =(const DOFConstraint &dofcons)
{
	if (this==&dofcons)
	{
		return *this;
	}
	else
	{
		sn=dofcons.sn;
		m_name=dofcons.m_name;
		body_i=dofcons.body_i;
		body_j=dofcons.body_j;
		return *this;
	}
}
Matrix DOFConstraint::AssJacobian(Vector3x &Ri,EulerAngle &Thetai,Vector3x &Rj,EulerAngle &Thetaj)
{
	Matrix TMP;
	return TMP;
}
Matrix DOFConstraint::AssConstraintEquations(Vector3x &Ri, EulerAngle &Thetai, Vector3x &Rj, EulerAngle &Thetaj)
{
	Matrix TMP;
	return TMP;
}
DOFConstraint::~DOFConstraint()
{
}


// DOFConstraint 成员函数
