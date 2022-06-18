#include "pch.h"
#include "RevoluteJoint.h"

//IMPLEMENT_SERIAL(RevoluteJoint,DOFConstraint,VERSIONABLE_SCHEMA|2)

RevoluteJoint::RevoluteJoint(void):DOFConstraint()
{
	m_type=1;
	decr_dofs=5;
	iconnect_pnt=0;
	v1_pnt=0;
	v2_pnt=0;
	jconnect_pnt=0;
	vj_pnt=0;
}
RevoluteJoint::RevoluteJoint(int n, CString name, InertiaElem *i, int connecti, int v1, int v2, InertiaElem *j, int connectj, int vj):DOFConstraint(n,name,i,j),
		iconnect_pnt(connecti),v1_pnt(v1),v2_pnt(v2),jconnect_pnt(connectj),vj_pnt(vj)
{
	m_type=1;
	decr_dofs=5;
}
RevoluteJoint::RevoluteJoint(const RevoluteJoint &rj)
{
	DOFConstraint::DOFConstraint(rj);
	m_type=rj.m_type;
	decr_dofs=rj.decr_dofs;
	iconnect_pnt=rj.iconnect_pnt;
	v1_pnt=rj.v1_pnt;
	v2_pnt=rj.v2_pnt;
	jconnect_pnt=rj.jconnect_pnt;
	vj_pnt=rj.vj_pnt;
}
RevoluteJoint& RevoluteJoint::operator =(const RevoluteJoint &rj)
{
	if (this==&rj)
	{
		return *this;
	}
	else
	{
		DOFConstraint::operator =(rj);
		m_type=rj.m_type;
	    decr_dofs=rj.decr_dofs;
		iconnect_pnt=rj.iconnect_pnt;
		v1_pnt=rj.v1_pnt;
		v2_pnt=rj.v2_pnt;
		jconnect_pnt=rj.jconnect_pnt;
		vj_pnt=rj.vj_pnt;
		return *this;
	}
}
Matrix RevoluteJoint::AssJacobian(Vector3x &Ri,EulerAngle &Thetai,Vector3x &Rj,EulerAngle &Thetaj)
{
	//about the body i
	Vector3x ilocpos_connectpnt=body_i->getMarkerLocalVec(iconnect_pnt);
	//about the body j
	Vector3x jlocpos_connectpnt=body_j->getMarkerLocalVec(jconnect_pnt);
	//get jacobian CI Ri-Rj=0
	Matrix CIsub0(3,3);
	CIsub0[0][0] = 1.0; CIsub0[1][1] = 1.0; CIsub0[2][2] = 1.0;
	Matrix CIsub1=(-1.0)*Thetai.getRotMat()*ilocpos_connectpnt.toSkewMatrix()*Thetai.getGconjMat();
	Matrix CIsub2(3, 3);
	CIsub2[0][0] = -1.0; CIsub2[1][1] = -1.0; CIsub2[2][2] = -1.0;
	Matrix CIsub3=Thetaj.getRotMat()*jlocpos_connectpnt.toSkewMatrix()*Thetaj.getGconjMat();
	Matrix CI(3,12);
	CI.putSubMat(0,0,CIsub0); CI.putSubMat(0,3,CIsub1); CI.putSubMat(0,6,CIsub2); CI.putSubMat(0,9,CIsub3);

	//get jacobian CII and CIII
	Vector3x vj=body_j->getMarkerLocalVec(vj_pnt)-body_j->getMarkerLocalVec(jconnect_pnt);
	Matrix vjT=~(vj.toMatrix());
	Vector3x v1=body_i->getMarkerLocalVec(v1_pnt)-body_i->getMarkerLocalVec(iconnect_pnt);
	Matrix v1T=~(v1.toMatrix());
	Vector3x v2=body_i->getMarkerLocalVec(v2_pnt)-body_i->getMarkerLocalVec(iconnect_pnt);
	Matrix v2T=~(v2.toMatrix());

	//CII
	Matrix CIIsub0(1,3);
	Matrix CIIsub1=(-1.0)*vjT*Thetai.getRotMat()*v1.toSkewMatrix()*Thetai.getGconjMat();
	Matrix CIIsub2(1,3);
	Matrix CIIsub3=(-1.0)*v1T*Thetaj.getRotMat()*vj.toSkewMatrix()*Thetaj.getGconjMat();
	Matrix CII(1,12); CII.putSubMat(0,0,CIIsub0); CII.putSubMat(0,3,CIIsub1); CII.putSubMat(0,6,CIIsub2); CII.putSubMat(0,9,CIIsub3);

	//CIII
	Matrix CIIIsub0(1,3);
	Matrix CIIIsub1=(-1.0)*vjT*Thetai.getRotMat()*v2.toSkewMatrix()*Thetai.getGconjMat();
	Matrix CIIIsub2(1,3);
	Matrix CIIIsub3=(-1.0)*v2T*Thetaj.getRotMat()*vj.toSkewMatrix()*Thetaj.getGconjMat();
	Matrix CIII(1,12); CIII.putSubMat(0,0,CIIIsub0); CIII.putSubMat(0,3,CIIIsub1); CIII.putSubMat(0,6,CIIIsub2); CIII.putSubMat(0,9,CIIIsub3);

	//get C
	Matrix C(5,12); C.putSubMat(0,0,CI); C.putSubMat(3,0,CII); C.putSubMat(4,0,CIII);
	return C;
}
Matrix RevoluteJoint::AssConstraintEquations(Vector3x &Ri, EulerAngle &Thetai, Vector3x &Rj, EulerAngle &Thetaj)
{
	//connection pnts
	//revolution axis attached to the ground if it exists
	Vector3x ilocpos_connectpnt = body_i->getMarkerLocalVec(iconnect_pnt);
	Vector3x jlocpos_connectpnt = body_j->getMarkerLocalVec(jconnect_pnt);
	Vector3x v1 = body_i->getMarkerLocalVec(v1_pnt) - body_i->getMarkerLocalVec(iconnect_pnt);
	Vector3x v2 = body_i->getMarkerLocalVec(v2_pnt) - body_i->getMarkerLocalVec(iconnect_pnt);
	Vector3x vj = body_j->getMarkerLocalVec(vj_pnt) - body_j->getMarkerLocalVec(jconnect_pnt);
	Vector3x C1 = Ri + Thetai.getRotMat()*ilocpos_connectpnt - Rj - Thetaj.getRotMat()*jlocpos_connectpnt;
	double c4 = (Thetai.getRotMat()*v1).dotProduct(Thetaj.getRotMat()*vj);
	double c5 = (Thetai.getRotMat()*v2).dotProduct(Thetaj.getRotMat()*vj);
	Matrix tmp(5, 1);
	tmp[0][0] = C1[0];
	tmp[1][0] = C1[1];
	tmp[2][0] = C1[2];
	tmp[3][0] = c4;
	tmp[4][0] = c5;
	return tmp;
}
RevoluteJoint::~RevoluteJoint(void)
{

}
