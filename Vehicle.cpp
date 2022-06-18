// Vehicle.cpp : 实现文件
#include "pch.h"
#include "Vehicle.h"

// Vehicle

Vehicle::Vehicle()
{
	//20130829 Ground addedd when intialized
	Vector3x r0(0,0,0);
	Vector3x rd0(0,0,0);
	Vector3x rdd0(0,0,0);
	EulerAngle ea0(0,0,0);
	EulerAngle ead0(0,0,0);
	EulerAngle eadd0(0,0,0);
	Vector3x ia(1,1,1);
	RigidBody GND(0,"Ground",0,0,1.0,ia,r0,rd0,rdd0,ea0,ead0,eadd0);
	InertiaElemHdr* GndHdr=new InertiaElemHdr(GND);
	Inertia_array.push_back(*GndHdr);
	InertiaElemIndex.push_back(0);
	q0vec.push_back(GND.getq0());
	qd0vec.push_back(GND.getqd0());
	qdd0vec.push_back(GND.getqdd0());
}
void Vehicle::CreateInertiaElemHdr(InertiaElem &ie)
{
	InertiaElemHdr* pNewIEHdr=new InertiaElemHdr(ie);
	Inertia_array.push_back(*pNewIEHdr);
	InertiaElemIndex.push_back(ie.getSN());
	q0vec.push_back(ie.getq0());
	qd0vec.push_back(ie.getqd0());
	qdd0vec.push_back(ie.getqdd0());
}
void Vehicle::CreateForceElemHdr(ForceElem &fe)
{
	ForceElemHdr *pNewFEHdr=new ForceElemHdr(fe);
	ForceElem_array.push_back(*pNewFEHdr);
}
void Vehicle::CreateDOFConstraintHdr(DOFConstraint &cons)
{
	DOFConstraintHdr *pNewDOFHdr=new DOFConstraintHdr(cons);
	DOFConstraint_array.push_back(*pNewDOFHdr);
}
void Vehicle::CreateSpline(Matrix &mx)
{
	CubicSpline sp(mx);
	spl.push_back(sp);
}
void Vehicle::AddGroundMarker(Marker &mk)
{
	Inertia_array[0]->addMarker(mk);
}
//assemble system inertia matrix,force vectors
Matrix Vehicle::AssSysMat(std::vector<VectorN> &qvector)//assembled by InettiaElem SN
{
	int index=0;//counting number of inertiaelements
	std::vector<Matrix> tmp;
	std::vector<int> inertiaindexvec;
	int inertiaindex=0;
	for(std::vector<InertiaElemHdr>::iterator ite=Inertia_array.begin()+1; ite<Inertia_array.end();ite++)
	{
		inertiaindex=(*ite)->getSN();
		inertiaindexvec.push_back(inertiaindex);
		VectorN qi=qvector[inertiaindex];//q is assembled by InetriaElem SN
		EulerAngle thetai(qi[3],qi[4],qi[5]);
		Matrix submati=(*ite)->AssMassMat(thetai);
		tmp.push_back(submati);
		index++;
	}
	Matrix sysmat(6*index,6*index);
	for (int i=0;i<index;i++)
	{
		sysmat.putSubMat(6*(inertiaindexvec[i]-1),6*(inertiaindexvec[i]-1),tmp[i]);
	}
	return sysmat;
}
VectorN Vehicle::AssSysForVec(std::vector<VectorN> &q,std::vector<VectorN> &qd,double splineindex)
{
	//number of the inertiaelem NG
	int index=0;
	std::vector<VectorN> tmp;
	std::vector<int> inertiaelementindexvec;
	int inertiaindex=0;
	for (std::vector<InertiaElemHdr>::iterator ite=Inertia_array.begin()+1;ite<Inertia_array.end();ite++)
	{
		inertiaindex=(*ite)->getSN();
		inertiaelementindexvec.push_back(inertiaindex);
		VectorN qi=q[inertiaindex];
		VectorN qdi=qd[inertiaindex];
		Vector3x ri(qi[0],qi[1],qi[2]);
		Vector3x rdi(qdi[0],qdi[1],qdi[2]);
		EulerAngle thetai(qi[3],qi[4],qi[5]);
		EulerAngle thetadi(qdi[3],qdi[4],qdi[5]);
		VectorN fi=(*ite)->AssembleForceVector(ri,rdi,thetai,thetadi,splineindex);
		tmp.push_back(fi);
		index++;
	}
	VectorN fvec(6*index);
	for (int i=0;i<index;i++)
	{
		fvec.putSubVec((inertiaelementindexvec[i]-1)*6,tmp[i]);
	}
	//forces vector of ForceElem
	VectorN fevec(6*index);
	for (std::vector<ForceElemHdr>::iterator fit=ForceElem_array.begin();fit<ForceElem_array.end();fit++)
	{
		VectorN tmpvec(6*index);
		int fromnum=(*fit)->getfrom()->getSN();
		int tonum=(*fit)->getto()->getSN();
		VectorN fq=q[fromnum];
		VectorN tq=q[tonum];
		VectorN fqd=qd[fromnum];
		VectorN tqd=qd[tonum];
		Vector3x fr(fq[0],fq[1],fq[2]);
		Vector3x tr(tq[0],tq[1],tq[2]);
		Vector3x frd(fqd[0],fqd[1],fqd[2]);
		Vector3x trd(tqd[0],tqd[1],tqd[2]);
		EulerAngle ftheta(fq[3],fq[4],fq[5]);
		EulerAngle fthetad(fqd[3],fqd[4],fqd[5]);
		EulerAngle ttheta(tq[3],tq[4],tq[5]);
		EulerAngle tthetad(tqd[3],tqd[4],tqd[5]);
		std::vector<VectorN> res=(*fit)->AssForceVec(fr,frd,ftheta,fthetad,tr,trd,ttheta,tthetad,splineindex);
		tmpvec.putSubVec((fromnum-1)*6,res[0]);
		if ((*fit)->getto()->getGround())
		{
			tmpvec.putSubVec((tonum-1)*6,res[1]);
		}
		fevec+=tmpvec;
	}
	VectorN allforcevec=fvec+fevec;
	return allforcevec;
}
VectorN Vehicle::AssSysConstraintEquations(std::vector<VectorN> &q, std::vector<VectorN> &qd)
{
	VectorN tmp;
	return tmp;
}
void Vehicle::UpdateKinematics(std::vector<VectorN> &q,std::vector<VectorN> &qd)
{
	for (std::vector<InertiaElemHdr>::iterator ite=Inertia_array.begin()+1;ite<Inertia_array.end();ite++)
	{
		int index=(*ite)->getSN();
		(*ite)->UpdateKimematicsPara(q[index],qd[index]);
	}
}

void Vehicle::UpateEquivalent(std::vector<VectorN> &q, std::vector<VectorN> &qd)
{
	for (std::vector<InertiaElemHdr>::iterator ite = Inertia_array.begin() + 1; ite<Inertia_array.end(); ite++)
	{
		int index = (*ite)->getSN();
		(*ite)->UpdateEquivalentPara(q[index], qd[index]);
	}
}

void Vehicle::UpdateKinematicsAcc(std::vector<VectorN> &q, std::vector<VectorN> &qd, std::vector<VectorN> &qdd)
{
	for (std::vector<InertiaElemHdr>::iterator ite = Inertia_array.begin() + 1; ite<Inertia_array.end(); ite++)
	{
		int index = (*ite)->getSN();
		(*ite)->UpdateKinematicsParaAcc(q[index], qd[index],qdd[index]);
	}
}
void Vehicle::UpdateInitialPos(std::vector<VectorN> &q)
{
	for (std::vector<InertiaElemHdr>::iterator ite = Inertia_array.begin() + 1; ite<Inertia_array.end(); ite++)
	{
		int index = (*ite)->getSN();
		(*ite)->UpdateInitialKinematicsParaPos(q[index]);
	}
}
void Vehicle::UpdateForces(std::vector<VectorN> &stdvecn)
{
	for (std::vector<InertiaElemHdr>::iterator ite = Inertia_array.begin() + 1; ite < Inertia_array.end(); ite++)//ground not included
	{
		int index = (*ite)->getSN();
		(*ite)->UpdateForcePara(stdvecn[index]);
	}
}
void Vehicle::TriggerIntPntResPushBack(void)
{
	for (std::vector<InertiaElemHdr>::iterator ite=Inertia_array.begin()+1;ite<Inertia_array.end();ite++)
	{
		(*ite)->TriggeIntPntResPushBackrProcess();
	}
}
void Vehicle::TriggerSysSave(void)
{
	for(std::vector<InertiaElemHdr>::iterator ite=Inertia_array.begin(); ite<Inertia_array.end();ite++)
	{
		(*ite)->TriggerSave();
	}
}
Matrix Vehicle::AssSysJacobMat(std::vector<VectorN> &q,std::vector<VectorN> &qd)
{
	std::vector<int> dofi;
	std::vector<int> dofj;
	std::vector<Matrix> subjcobmat;
	std::vector<int> decrdofvec;
	int dofsum=0;
	int dofindex=0;
	for (std::vector<DOFConstraintHdr>::iterator dof=DOFConstraint_array.begin();dof!=DOFConstraint_array.end();dof++)
	{
		int bodyinum=(*dof)->getbodyi()->getSN();
		int bodyjnum=(*dof)->getbodyj()->getSN();
		int subdof=(*dof)->getDecrDofs();
		dofsum+=subdof;
		decrdofvec.push_back(dofsum);
		dofi.push_back(bodyinum);
		dofj.push_back(bodyjnum);
		VectorN qi=q[bodyinum];
		VectorN qdi=qd[bodyinum];
		Vector3x ri(qi[0],qi[1],qi[2]);
		Vector3x rdi(qdi[0],qdi[1],qdi[2]);
		EulerAngle thetai(qi[3],qi[4],qi[5]);
		EulerAngle thetadi(qdi[3],qdi[4],qdi[5]);
		VectorN qj=q[bodyjnum];
		VectorN qdj=qd[bodyjnum];
		Vector3x rj(qj[0],qj[1],qj[2]);
		Vector3x rdj(qdj[0],qdj[1],qdj[2]);
		EulerAngle thetaj(qj[3],qj[4],qj[5]);
		EulerAngle thetadj(qdj[3],qdj[4],qdj[5]);
		Matrix tmp=(*dof)->AssJacobian(ri,thetai,rj,thetaj);
		subjcobmat.push_back(tmp);
		dofindex++;
	}
	//20141120 constrains jacobian matrix
	int col_num = static_cast<int>(Inertia_array.size()-1)*6;
	Matrix sysjacobmat(dofsum,col_num);
	int i = 0;
	for (std::vector<DOFConstraintHdr>::iterator dof = DOFConstraint_array.begin(); dof != DOFConstraint_array.end(); dof++)
	{
		int row_sum = 0;
		int col_i = dofi[i];
		int col_j = dofj[i];
		int row = decrdofvec[i];
		for (int j = row_sum; j < row_sum + row; j++)
		{
			for (int k = 0; k < 6; k++)
			{
				if ((*dof)->getbodyi()->getGround())
				{
					sysjacobmat[j][(col_i - 1) * 6 + k] = subjcobmat[i][j-row_sum][k];
				}
				if ((*dof)->getbodyj()->getGround())
				{
					sysjacobmat[j][(col_j - 1) * 6 + k] = subjcobmat[i][j - row_sum][k + 6];
				}
			}
		}
		row_sum += row;
		i++;
	}
	return sysjacobmat;
}

Vehicle::~Vehicle()
{

}
VectorN Vehicle::assembleVecn(std::vector<VectorN> &stdvecn)
//exclude 0 ground inertiaelement, from index 1
{
	int size=static_cast<int>(stdvecn.size())-1;
	VectorN vecn(size*6);
	for (int i=0;i<size;i++)
	{
		vecn.putSubVec(i*6,stdvecn[i+1]);
	}
	return vecn;
}
std::vector<VectorN> Vehicle::assembleStdVecn(VectorN &vecn)
{
	int size=vecn.Dim()/6;
	std::vector<VectorN> stdvecn;
	//zeros for GND
	VectorN gnd(6);
	stdvecn.push_back(gnd);
	for (int i=0;i<size;i++)
	{
		VectorN tmp=vecn.getSubVec(i*6,i*6+5);
		stdvecn.push_back(tmp);
	}
	return stdvecn;
}
void Vehicle::RK4integration(double stepsize,double time)
{
	this->InitConsolWnd();
	//this->TriggerIntPntResPushBack();
	printf("RUNGE KUTTA EXPLICIT STARTED......\n");
	double t = 0.0;
	std::vector<VectorN> x0 = this->SortbyInertiaIndex(this->getq0vec());
	std::vector<VectorN> v0 = this->SortbyInertiaIndex(this->getqd0vec());
	while (t <= time)
	{
		VectorN xvecn0 = this->assembleVecn(x0);
		VectorN velo0 = this->assembleVecn(v0);

		VectorN k1 = stepsize*velo0;
		//VectorN K1 = stepsize*((this->AssSysMat(x0)).Inv())*(this->AssSysForVec(x0, v0, t));
		VectorN K1 = stepsize*((this->AssSysMat(x0)).IterationInv())*(this->AssSysForVec(x0, v0, t));

		VectorN k2 = stepsize*(velo0 + 0.5*K1);
		VectorN x01 = xvecn0 + 0.5*k1;
		VectorN v01 = velo0 + 0.5*K1;
		//VectorN K2 = stepsize*((this->AssSysMat(this->assembleStdVecn(x01))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x01), this->assembleStdVecn(v01), t + 0.5*stepsize));
		VectorN K2 = stepsize*((this->AssSysMat(this->assembleStdVecn(x01))).IterationInv())*(this->AssSysForVec(this->assembleStdVecn(x01), this->assembleStdVecn(v01), t + 0.5*stepsize));

		VectorN k3 = stepsize*(velo0 + 0.5*K2);
		VectorN x02 = xvecn0 + 0.5*k2;
		VectorN v02 = velo0 + 0.5*K2;
		//VectorN K3 = stepsize*((this->AssSysMat(this->assembleStdVecn(x02))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x02), this->assembleStdVecn(v02), t + 0.5*stepsize));
		VectorN K3 = stepsize*((this->AssSysMat(this->assembleStdVecn(x02))).IterationInv())*(this->AssSysForVec(this->assembleStdVecn(x02), this->assembleStdVecn(v02), t + 0.5*stepsize));

		VectorN k4 = stepsize*(velo0 + K3);
		VectorN x03 = xvecn0 + k3;
		VectorN v03 = velo0 + K3;
		//VectorN K4 = stepsize*((this->AssSysMat(this->assembleStdVecn(x03))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x03), this->assembleStdVecn(v03), t + stepsize));
		VectorN K4 = stepsize*((this->AssSysMat(this->assembleStdVecn(x03))).IterationInv())*(this->AssSysForVec(this->assembleStdVecn(x03), this->assembleStdVecn(v03), t + stepsize));
		VectorN newqvec = xvecn0 + (1 / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
		VectorN newqdvec = velo0 + (1 / 6.0)*(K1 + 2.0*K2 + 2.0*K3 + K4);

		std::vector<VectorN> newqstdvec = this->assembleStdVecn(newqvec);
		std::vector<VectorN> newqdstdvec = this->assembleStdVecn(newqdvec);

		x0.clear();
		x0 = newqstdvec;
		v0.clear();
		v0 = newqdstdvec;
		std::vector<VectorN> cforcestdvecn = this->assembleStdVecn(this->AssSysForVec(x0, v0, t));

		this->UpdateKinematics(x0, v0);
		this->UpdateForces(cforcestdvecn);
		this->TriggerIntPntResPushBack();

		t = t + stepsize;
		printf("%f\n", t);
	}
	this->TriggerSysSave();
	FreeConsole();
}
void Vehicle::RK4(double stepsize, double time)
{
	this->InitConsolWnd();
	//this->TriggerIntPntResPushBack();
	printf("RUNGE KUTTA EXPLICIT STARTED......\n");
	double t = 0.0;
	std::vector<VectorN> x0 = this->SortbyInertiaIndex(this->getq0vec());
	std::vector<VectorN> v0 = this->SortbyInertiaIndex(this->getqd0vec());
	while (t <= time)
	{
		VectorN xvecn0 = this->assembleVecn(x0);
		VectorN velo0 = this->assembleVecn(v0);

		VectorN k1 = stepsize*velo0;
		VectorN K1 = stepsize*((this->AssSysMat(x0)).Inv())*(this->AssSysForVec(x0, v0, t));

		VectorN k2 = stepsize*(velo0 + 0.5*K1);
		VectorN x01 = xvecn0 + 0.5*k1;
		VectorN v01 = velo0 + 0.5*K1;
		VectorN K2 = stepsize*((this->AssSysMat(this->assembleStdVecn(x01))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x01), this->assembleStdVecn(v01), t + 0.5*stepsize));

		VectorN k3 = stepsize*(velo0 + 0.5*K2);
		VectorN x02 = xvecn0 + 0.5*k2;
		VectorN v02 = velo0 + 0.5*K2;
		VectorN K3 = stepsize*((this->AssSysMat(this->assembleStdVecn(x02))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x02), this->assembleStdVecn(v02), t + 0.5*stepsize));

		VectorN k4 = stepsize*(velo0 + K3);
		VectorN x03 = xvecn0 + k3;
		VectorN v03 = velo0 + K3;
		VectorN K4 = stepsize*((this->AssSysMat(this->assembleStdVecn(x03))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x03), this->assembleStdVecn(v03), t + stepsize));

		VectorN newqvec = xvecn0 + (1 / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
		VectorN newqdvec = velo0 + (1 / 6.0)*(K1 + 2.0*K2 + 2.0*K3 + K4);

		std::vector<VectorN> newqstdvec = this->assembleStdVecn(newqvec);
		std::vector<VectorN> newqdstdvec = this->assembleStdVecn(newqdvec);

		x0.clear();
		x0 = newqstdvec;
		v0.clear();
		v0 = newqdstdvec;
		std::vector<VectorN> cforcestdvecn = this->assembleStdVecn(this->AssSysForVec(x0, v0, t));
		this->UpdateKinematics(x0, v0);
		this->UpdateForces(cforcestdvecn);
		this->TriggerIntPntResPushBack();

		t = t + stepsize;
		printf("%f\n", t);
	}
	this->TriggerSysSave();
	FreeConsole();
}
void Vehicle::InitBalancedPosition(double stepsize, double time)
{
	this->InitConsolWnd();
	printf("RUNGE KUTTA EXPLICIT STARTED FOR EQUIVALENT ANALYSIS......\n");
	//
	std::vector<VectorN> x0 = this->SortbyInertiaIndex(this->getq0vec());
	std::vector<VectorN> v0 = this->SortbyInertiaIndex(this->getqd0vec());
	//
	double t = 0.0;
	while (t <= time+stepsize)
	{
		VectorN xvecn0 = this->assembleVecn(x0);
		VectorN velo0 = this->assembleVecn(v0);

		VectorN k1 = stepsize*velo0;
		VectorN K1 = stepsize*((this->AssSysMat(x0)).Inv())*(this->AssSysForVec(x0, v0, t));

		VectorN k2 = stepsize*(velo0 + 0.5*K1);
		VectorN x01 = xvecn0 + 0.5*k1;
		VectorN v01 = velo0 + 0.5*K1;
		VectorN K2 = stepsize*((this->AssSysMat(this->assembleStdVecn(x01))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x01), this->assembleStdVecn(v01), t + 0.5*stepsize));

		VectorN k3 = stepsize*(velo0 + 0.5*K2);
		VectorN x02 = xvecn0 + 0.5*k2;
		VectorN v02 = velo0 + 0.5*K2;
		VectorN K3 = stepsize*((this->AssSysMat(this->assembleStdVecn(x02))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x02), this->assembleStdVecn(v02), t + 0.5*stepsize));

		VectorN k4 = stepsize*(velo0 + K3);
		VectorN x03 = xvecn0 + k3;
		VectorN v03 = velo0 + K3;
		VectorN K4 = stepsize*((this->AssSysMat(this->assembleStdVecn(x03))).Inv())*(this->AssSysForVec(this->assembleStdVecn(x03), this->assembleStdVecn(v03), t + stepsize));

		VectorN newqvec = xvecn0 + (1 / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
		VectorN newqdvec = velo0 + (1 / 6.0)*(K1 + 2.0*K2 + 2.0*K3 + K4);
		std::vector<VectorN> newqstdvec = this->assembleStdVecn(newqvec);
		std::vector<VectorN> newqdstdvec = this->assembleStdVecn(newqdvec);

		x0.clear();
		x0 = newqstdvec;
		v0.clear();
		v0 = newqdstdvec;
		std::vector<VectorN> cforcestdvecn = this->assembleStdVecn(this->AssSysForVec(x0, v0, t));
		// Update status for each integration step
		this->UpdateKinematics(x0, v0);
		//this->UpateEquivalent(x0, v0);
		//20210412
		this->UpdateForces(cforcestdvecn);
		printf("%f\n", t);
		t = t + stepsize;
		this->UpdateInitialPos(x0);
	}
}
void Vehicle::ABAM(double stepsize, double time)
{
	this->InitConsolWnd();
	//this->TriggerIntPntResPushBack();
	printf("ABAM......\n");
	double t = 0.0;
	//start pnt 1
	t += stepsize;
	std::vector<VectorN> x0 = this->SortbyInertiaIndex(this->getq0vec());
	std::vector<VectorN> v0 = this->SortbyInertiaIndex(this->getqd0vec());
	VectorN velo0 = this->assembleVecn(v0);
	VectorN k1 = stepsize*velo0;
	VectorN K1 = stepsize*(!(this->AssSysMat(x0)))*(this->AssSysForVec(x0, v0, t));
	VectorN k2 = stepsize*(velo0 + 0.5*K1);
	VectorN x01 = this->assembleVecn(x0) + 0.5*k1;
	std::vector<VectorN> incre_x01 = this->assembleStdVecn(x01);
	VectorN v01 = this->assembleVecn(v0) + 0.5*K1;
	std::vector<VectorN> incre_v01 = this->assembleStdVecn(v01);
	VectorN K2 = stepsize*(!(this->AssSysMat(incre_x01)))*(this->AssSysForVec(incre_x01, incre_v01, t + 0.5*stepsize));
	VectorN k3 = stepsize*(velo0 + 0.5*K2);
	VectorN x02 = this->assembleVecn(x0) + 0.5*k2;
	std::vector<VectorN> incre_x02 = this->assembleStdVecn(x02);
	VectorN v02 = this->assembleVecn(v0) + 0.5*K2;
	std::vector<VectorN> incre_v02 = this->assembleStdVecn(v02);
	VectorN K3 = stepsize*(!(this->AssSysMat(incre_x02)))*(this->AssSysForVec(incre_x02, incre_v02, t + 0.5*stepsize));
	VectorN k4 = stepsize*(velo0 + K3);
	VectorN x03 = this->assembleVecn(x0) + k3;
	std::vector<VectorN> incre_x03 = this->assembleStdVecn(x03);
	VectorN v03 = this->assembleVecn(v0) + K3;
	std::vector<VectorN> incre_v03 = this->assembleStdVecn(v03);
	VectorN K4 = stepsize*(!(this->AssSysMat(incre_x03)))*(this->AssSysForVec(incre_x03, incre_v03, t + stepsize));
	VectorN x1= this->assembleVecn(x0) + (1 / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
	VectorN v1= this->assembleVecn(v0) + (1 / 6.0)*(K1 + 2.0*K2 + 2.0*K3 + K4);
	std::vector<VectorN> newqstdvec = this->assembleStdVecn(x1);
	std::vector<VectorN> newqdstdvec = this->assembleStdVecn(v1);
	x0.clear();x0 = newqstdvec;
	v0.clear(); v0 = newqdstdvec;
	this->UpdateKinematics(x0, v0);
	VectorN cforcevectorn = this->AssSysForVec(x0, v0, t);
	std::vector<VectorN> cforcestdvecn = this->assembleStdVecn(cforcevectorn);
	this->UpdateForces(cforcestdvecn);
	this->TriggerIntPntResPushBack();
	//start pnt 2
	t += stepsize;
	velo0 = this->assembleVecn(v0);
	k1 = stepsize*velo0;
	K1 = stepsize*(!(this->AssSysMat(x0)))*(this->AssSysForVec(x0, v0, t));
	k2 = stepsize*(velo0 + 0.5*K1);
	x01 = this->assembleVecn(x0) + 0.5*k1;
	incre_x01 = this->assembleStdVecn(x01);
	v01 = this->assembleVecn(v0) + 0.5*K1;
	incre_v01 = this->assembleStdVecn(v01);
	K2 = stepsize*(!(this->AssSysMat(incre_x01)))*(this->AssSysForVec(incre_x01, incre_v01, t + 0.5*stepsize));
	k3 = stepsize*(velo0 + 0.5*K2);
	x02 = this->assembleVecn(x0) + 0.5*k2;
	incre_x02 = this->assembleStdVecn(x02);
	v02 = this->assembleVecn(v0) + 0.5*K2;
	incre_v02 = this->assembleStdVecn(v02);
	K3 = stepsize*(!(this->AssSysMat(incre_x02)))*(this->AssSysForVec(incre_x02, incre_v02, t + 0.5*stepsize));
	k4 = stepsize*(velo0 + K3);
	x03 = this->assembleVecn(x0) + k3;
	incre_x03 = this->assembleStdVecn(x03);
	v03 = this->assembleVecn(v0) + K3;
	incre_v03 = this->assembleStdVecn(v03);
	K4 = stepsize*(!(this->AssSysMat(incre_x03)))*(this->AssSysForVec(incre_x03, incre_v03, t + stepsize));
	VectorN x2 = this->assembleVecn(x0) + (1 / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
	VectorN v2 = this->assembleVecn(v0) + (1 / 6.0)*(K1 + 2.0*K2 + 2.0*K3 + K4);
	newqstdvec = this->assembleStdVecn(x2);
	newqdstdvec = this->assembleStdVecn(v2);
	x0.clear();x0 = newqstdvec;
	v0.clear();v0 = newqdstdvec;
	this->UpdateKinematics(x0, v0);
	cforcevectorn = this->AssSysForVec(x0, v0, t);
	cforcestdvecn = this->assembleStdVecn(cforcevectorn);
	this->UpdateForces(cforcestdvecn);
	this->TriggerIntPntResPushBack();
	//start pnt 3
	t += stepsize;
	velo0 = this->assembleVecn(v0);
	k1 = stepsize*velo0;
	K1 = stepsize*(!(this->AssSysMat(x0)))*(this->AssSysForVec(x0, v0, t));
	k2 = stepsize*(velo0 + 0.5*K1);
	x01 = this->assembleVecn(x0) + 0.5*k1;
	incre_x01 = this->assembleStdVecn(x01);
	v01 = this->assembleVecn(v0) + 0.5*K1;
	incre_v01 = this->assembleStdVecn(v01);
	K2 = stepsize*(!(this->AssSysMat(incre_x01)))*(this->AssSysForVec(incre_x01, incre_v01, t + 0.5*stepsize));
	k3 = stepsize*(velo0 + 0.5*K2);
	x02 = this->assembleVecn(x0) + 0.5*k2;
	incre_x02 = this->assembleStdVecn(x02);
	v02 = this->assembleVecn(v0) + 0.5*K2;
	incre_v02 = this->assembleStdVecn(v02);
	K3 = stepsize*(!(this->AssSysMat(incre_x02)))*(this->AssSysForVec(incre_x02, incre_v02, t + 0.5*stepsize));
	k4 = stepsize*(velo0 + K3);
	x03 = this->assembleVecn(x0) + k3;
	incre_x03 = this->assembleStdVecn(x03);
	v03 = this->assembleVecn(v0) + K3;
	incre_v03 = this->assembleStdVecn(v03);
	K4 = stepsize*(!(this->AssSysMat(incre_x03)))*(this->AssSysForVec(incre_x03, incre_v03, t + stepsize));
	VectorN x3 = this->assembleVecn(x0) + (1 / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
	VectorN v3 = this->assembleVecn(v0) + (1 / 6.0)*(K1 + 2.0*K2 + 2.0*K3 + K4);
	newqstdvec = this->assembleStdVecn(x3);
	newqdstdvec = this->assembleStdVecn(v3);
	x0.clear();x0 = newqstdvec;
	v0.clear();v0 = newqdstdvec;
	this->UpdateKinematics(x0, v0);
	cforcevectorn = this->AssSysForVec(x0, v0, t);
	cforcestdvecn = this->assembleStdVecn(cforcevectorn);
	this->UpdateForces(cforcestdvecn);
	this->TriggerIntPntResPushBack();
	//start pnt 4
	t += stepsize;
	velo0 = this->assembleVecn(v0);
	k1 = stepsize*velo0;
	K1 = stepsize*(!(this->AssSysMat(x0)))*(this->AssSysForVec(x0, v0, t));
	k2 = stepsize*(velo0 + 0.5*K1);
	x01 = this->assembleVecn(x0) + 0.5*k1;
	incre_x01 = this->assembleStdVecn(x01);
	v01 = this->assembleVecn(v0) + 0.5*K1;
	incre_v01 = this->assembleStdVecn(v01);
	K2 = stepsize*(!(this->AssSysMat(incre_x01)))*(this->AssSysForVec(incre_x01, incre_v01, t + 0.5*stepsize));
	k3 = stepsize*(velo0 + 0.5*K2);
	x02 = this->assembleVecn(x0) + 0.5*k2;
	incre_x02 = this->assembleStdVecn(x02);
	v02 = this->assembleVecn(v0) + 0.5*K2;
	incre_v02 = this->assembleStdVecn(v02);
	K3 = stepsize*(!(this->AssSysMat(incre_x02)))*(this->AssSysForVec(incre_x02, incre_v02, t + 0.5*stepsize));
	k4 = stepsize*(velo0 + K3);
	x03 = this->assembleVecn(x0) + k3;
	incre_x03 = this->assembleStdVecn(x03);
	v03 = this->assembleVecn(v0) + K3;
	incre_v03 = this->assembleStdVecn(v03);
	K4 = stepsize*(!(this->AssSysMat(incre_x03)))*(this->AssSysForVec(incre_x03, incre_v03, t + stepsize));
	VectorN x4 = this->assembleVecn(x0) + (1 / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
	VectorN v4 = this->assembleVecn(v0) + (1 / 6.0)*(K1 + 2.0*K2 + 2.0*K3 + K4);
	newqstdvec = this->assembleStdVecn(x4);
	newqdstdvec = this->assembleStdVecn(v4);
	x0.clear();x0 = newqstdvec;
	v0.clear();v0 = newqdstdvec;
	this->UpdateKinematics(x0, v0);
	cforcevectorn = this->AssSysForVec(x0, v0, t);
	cforcestdvecn = this->assembleStdVecn(cforcevectorn);
	this->UpdateForces(cforcestdvecn);
	this->TriggerIntPntResPushBack();
	//adams-bashforth-adams-moulton started
	while (t < time)
	{
		t += stepsize;
		VectorN xp5 = this->assembleVecn(x0) + stepsize / 24 * (-9.0*v1 + 37 * v2 - 59 * v3 + 55 * v4);
		VectorN x5 = this->assembleVecn(x0) + stepsize / 24 * (v2 -5 * v3 + 19 * v4 + 9 * xp5);
		/////////////////////////
		VectorN f11 = (!(this->AssSysMat(this->assembleStdVecn(x1))))*(this->AssSysForVec(this->assembleStdVecn(x1), this->assembleStdVecn(v1), t - 4*stepsize));
		VectorN f22 = (!(this->AssSysMat(this->assembleStdVecn(x2))))*(this->AssSysForVec(this->assembleStdVecn(x2), this->assembleStdVecn(v2), t - 3 * stepsize));
		VectorN f32 = (!(this->AssSysMat(this->assembleStdVecn(x3))))*(this->AssSysForVec(this->assembleStdVecn(x3), this->assembleStdVecn(v3), t - 3* stepsize));
		VectorN f33 = (!(this->AssSysMat(this->assembleStdVecn(x3))))*(this->AssSysForVec(this->assembleStdVecn(x3), this->assembleStdVecn(v3), t - 2 * stepsize));
		VectorN f44 = (!(this->AssSysMat(this->assembleStdVecn(x4))))*(this->AssSysForVec(this->assembleStdVecn(x4), this->assembleStdVecn(v4), t - 1 * stepsize));
		////////////////////////
		VectorN vp5 = this->assembleVecn(v0) + stepsize / 24 * (-9.0*f11 + 37 * f22 - 59 * f32 + 55 * f44);
		VectorN vf5 = (!(this->AssSysMat(this->assembleStdVecn(xp5))))*(this->AssSysForVec(this->assembleStdVecn(xp5), this->assembleStdVecn(vp5), t - 0 * stepsize));
		VectorN v5 = this->assembleVecn(v0) + stepsize / 24 * (f22 - 5 * f33 + 19 * f44 + 9 * vf5);
		while ((x5 - xp5).getNorm() >= 1e-8)
		{
			xp5 = x5;
			x5 = this->assembleVecn(x0) + stepsize / 24 * (v2 - 5 * v3 + 19 * v4 + 9 * xp5);
		}
		while ((v5 - vp5).getNorm() >= 1e-8)
		{
			vp5 = v5;
			vf5 = (!(this->AssSysMat(this->assembleStdVecn(xp5))))*(this->AssSysForVec(this->assembleStdVecn(xp5), this->assembleStdVecn(vp5), t - 0 * stepsize));
			v5 = this->assembleVecn(v0) + stepsize / 24 * (f22 - 5 * f33 + 19 * f44 + 9 * vf5);
		}
		////////////////////////
		std::vector<VectorN> nqstdvec = this->assembleStdVecn(x5);
		std::vector<VectorN> nqdstdvec = this->assembleStdVecn(v5);
		x0.clear();x0 = nqstdvec;
		v0.clear();v0 = nqdstdvec;
		this->UpdateKinematics(x0, v0);
		VectorN cforcevectorn = this->AssSysForVec(x0, v0, t);
		std::vector<VectorN> cforcestdvecn = this->assembleStdVecn(cforcevectorn);
		this->UpdateForces(cforcestdvecn);
		this->TriggerIntPntResPushBack();
		x1 = x2; v1 = v2;
		x2 = x3; v2 = v3;
		x3 = x4; v3 = v4;
		x4 = x5; v4 = v5;
		printf("%f\n", t);
	}
	this->TriggerSysSave();
}
void Vehicle::Newmark(double stepsize, double time, double beta, double garma)
{
	this->InitConsolWnd();
	//this->TriggerIntPntResPushBack();
	printf("Newmark beta......\n");
	double t = 0.0;
	std::vector<VectorN> x0 = this->SortbyInertiaIndex(this->getq0vec());
	std::vector<VectorN> v0 = this->SortbyInertiaIndex(this->getqd0vec());
	std::vector<VectorN> a0 = this->SortbyInertiaIndex(this->getqdd0vec());
	while (t < time)
	{
		t += stepsize;
		VectorN pos0 = this->assembleVecn(x0);
		VectorN vel0 = this->assembleVecn(v0);
		VectorN acc0 = this->assembleVecn(a0);
		//////////////////////////////////////
		VectorN addons = (1.0 / (beta*stepsize*stepsize))*pos0 + (1.0 / (beta*stepsize))*vel0 + (1.0 / (2.0*beta)- 1.0 )*acc0;
		VectorN pos1 = (beta*stepsize*stepsize)*((!(this->AssSysMat(this->assembleStdVecn(pos0))))*this->AssSysForVec(this->assembleStdVecn(pos0), this->assembleStdVecn(vel0), t) + addons);
		VectorN vel1 = vel0 + stepsize*(1.0 - garma)*acc0 + stepsize*garma*((1.0 / (beta*stepsize*stepsize))*(pos1 - pos0) - vel0*(1.0 / (beta*stepsize)) - (1.0 / (2.0*beta) - 1.0)*acc0);
		//////////////////////////////////////
		VectorN pos2 = (beta*stepsize*stepsize)*((!(this->AssSysMat(this->assembleStdVecn(pos1))))*this->AssSysForVec(this->assembleStdVecn(pos1), this->assembleStdVecn(vel1), t) + addons);
		//implicit fixed point interactions///
		while ((pos2 - pos1).getNorm() >= 1e-8)
		{
			pos1 = pos2;
			vel1 = vel0 + stepsize*(1.0 - garma)*acc0 + stepsize*garma*((1.0 / (beta*stepsize*stepsize))*(pos1 - pos0) - vel0*(1.0 / (beta*stepsize)) - (1.0 / (2.0*beta) - 1.0)*acc0);
			pos2 = (beta*stepsize*stepsize)*((!(this->AssSysMat(this->assembleStdVecn(pos1))))*this->AssSysForVec(this->assembleStdVecn(pos1), this->assembleStdVecn(vel1), t) + addons);
		}
		std::vector<VectorN> x1 = this->assembleStdVecn(pos1);
		std::vector<VectorN> v1 = this->assembleStdVecn(vel1);
		VectorN acc1 = (1.0 / (beta*stepsize*stepsize))*(pos1 - pos0) - (1.0 / (beta*stepsize))*vel0 - (1.0 / (2.0*beta)-1.0)*acc0;
		std::vector<VectorN> a1 = this->assembleStdVecn(acc1);
		x0.clear(); x0 = x1;
		v0.clear(); v0 = v1;
		a0.clear(); a0 = a1;
		this->UpdateKinematicsAcc(x1, v1, a1);
		VectorN cforcevectorn = this->AssSysForVec(x1, v1, t);
		std::vector<VectorN> cforcestdvecn = this->assembleStdVecn(cforcevectorn);
		this->UpdateForces(cforcestdvecn);
		this->TriggerIntPntResPushBack();
		printf("%f\n", t);
	}
	this->TriggerSysSave();
}
void Vehicle::HHT(double stepsize,double time,double alfa)
{
	this->InitConsolWnd();
	double beta=pow(1-alfa,2)/4.0;
	double gama=(1.0-2.0*alfa)/2.0;

}
std::vector<VectorN> Vehicle::SortbyInertiaIndex(std::vector<VectorN> &stdvec)
{
	std::vector<VectorN> tmp(stdvec.size());
	int index=0;
	for (std::vector<InertiaElemHdr>::iterator ite=Inertia_array.begin();ite<Inertia_array.end();ite++)
	{
		int inertiaindex=(*ite)->getSN();
		tmp[inertiaindex]=stdvec[index];
		index++;
	}
	return tmp;
}
void Vehicle::static_analysis(void)
{
}
void Vehicle::UpdateStaticConfiguration(void)
{
	q0vec.clear();
	for (std::vector<InertiaElemHdr>::iterator ite = Inertia_array.begin(); ite < Inertia_array.end(); ite++)
	{
		q0vec.push_back(ite->getPointer()->getq0());
	}
}
void Vehicle::UpdateVelocityConfiguration(void)
{
	qd0vec.clear();
	for (std::vector<InertiaElemHdr>::iterator ite = Inertia_array.begin(); ite < Inertia_array.end(); ite++)
	{
		qd0vec.push_back(ite->getPointer()->getqd0());
	}
}
void Vehicle::SetRunSpeed(double V)
{
	// start at the second inertialelem, the first is GND
	for (std::vector<InertiaElemHdr>::iterator ite = Inertia_array.begin()+1; ite < Inertia_array.end(); ite++)
	{
		ite->getPointer()->SetRunSpeed(V);
	}
}
//void Vehicle::InitConsolWnd(void)
//{
//	int nCrt = 0;
//    FILE* fp;
//    AllocConsole();
//    nCrt = _open_osfhandle((intptr_t)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
//    fp = _fdopen(nCrt, "w");
//    *stdout = *fp;
//    setvbuf(stdout, NULL, _IONBF, 0);
//}

void Vehicle::InitConsolWnd(void)
{
	FILE* file = nullptr;
	AllocConsole();
	freopen_s(&file, "CONOUT$", "w", stdout);
}