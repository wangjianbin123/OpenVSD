// VectorN.cpp : 实现文件
//
#include "pch.h"
#include "VectorN.h"

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _MATH_H_
#define _MATH_H_
#include <math.h>
#endif

#ifndef _EPSINON_
#define EPSINON 1E-18
#endif

#ifndef _MINDOUBLE_
#define MINDOUBLE -1.7E+308
#endif

#ifndef _MAXDOUBLE_
#define MAXDOUBLE 1.7E+308
#endif

IMPLEMENT_SERIAL(VectorN,CObject,VERSIONABLE_SCHEMA|2)

// 构造函数

VectorN::VectorN()
{
	m_nDim=0;
	m_matrix.Empty();
}

VectorN::VectorN(int nDim)
{
	m_nDim=nDim;
	m_matrix.SetMatrix(nDim,1);
}
VectorN::VectorN(int nDim, double *pAry)
{
	m_nDim=nDim;
	m_matrix.SetMatrix(nDim,1);
	for (int i=0;i<nDim;i++)
	{
		m_matrix[i][0]=pAry[i];
	}
}
VectorN::VectorN(Matrix &mx)
{
	m_nDim=mx.Row();
	m_matrix=mx;
}
VectorN::VectorN(const VectorN &src)
{
	m_nDim=src.Dim();
	m_matrix=src.getMat();
}

//析构函数

VectorN::~VectorN()
{
	this->Empty();
}

//文件流

void VectorN::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);

	if (ar.IsStoring())
	{
		ar<<m_nDim;
		for (int i=0;i<m_nDim;i++)
		{
			ar<<(*this)[i];
		}
	}
	else
	{
		int nDim;
		ar>>nDim;
		SetVectorN(nDim);
		for (int i=0;i<nDim;i++)
		{
			ar>>(*this)[i];
		}
	}
}

//单目运算符重载

double & VectorN::operator [](int nDim)
{
	return m_matrix[nDim][0];
}
VectorN & VectorN::operator =(const VectorN &vec)
{
	if (this==&vec)
	{
		return *this;
	}
	m_nDim=vec.Dim();
	m_matrix=vec.getMat();
	return *this;
}
VectorN & VectorN::operator +=(VectorN &vec)
{
	ASSERT(this->Dim()==vec.Dim());
	m_matrix+=vec.getMat();
	return *this;
}
VectorN & VectorN::operator -=(VectorN &vec)
{
	ASSERT(this->Dim()==vec.Dim());
	m_matrix-=vec.getMat();
	return *this;
}
VectorN &VectorN::operator *=(double k)
{
	for (int i=0;i<m_nDim;i++)
	{
		m_matrix[i][0]*=k;
	}
	return *this;
}
VectorN &VectorN::operator/=(double k)
{

	for (int i = 0; i < m_nDim; i++)
	{
		m_matrix[i][0] /= k;
	}
	return *this;
}
void VectorN::Empty(void)
{
	m_nDim=0;
	m_matrix.Empty();
}
BOOL VectorN::IsEmpty(void)
{
	if (m_matrix.IsEmpty())
	{
		return TRUE;
	}
	else
		return FALSE;
}
void VectorN::SetVectorN(int nDim)
{
	m_nDim=nDim;
	m_matrix.SetMatrix(nDim,1);
}
void VectorN::SetVectorN(int nDim, double *pAry)
{
	this->Empty();
	m_nDim=nDim;
	m_matrix.SetMatrix(nDim,1,pAry);
}
void VectorN::SetVectorN(double *pAry)
{
	ASSERT(pAry!=NULL);
	for (int i=0;i<m_nDim;i++)
	{
		m_matrix[i][0]=pAry[i];
	}
}
void VectorN::SetVectorN(Matrix &mx)
{
	ASSERT(mx.Col()==1);
	m_nDim=mx.Row();
	m_matrix=mx;
}
void VectorN::toZero(void)
{
	m_matrix.toZero();
}
double VectorN::getNorm(void)
{
	double norm=0.0;
	for (int i=0;i<m_nDim;i++)
	{
		norm+=pow(m_matrix[i][0],2.0);
	}
	norm=sqrt(norm);
	return norm;
}
double VectorN::getSum(void)
{	
	double sum=0.0;
	for (int i=0;i<m_nDim;i++)
	{
		sum+=m_matrix[i][0];
	}
	return sum;
}
void VectorN::toUnit(void)
{
	double norm=this->getNorm();
	if (norm!=0)
	{
		for (int i=0;i<m_nDim;i++)
		{
			m_matrix[i][0]/=norm;
		}
	}
}
double VectorN::getMax(void)
{
	double fMax=(*this)[0];
	for (int i=1;i<m_nDim;i++)
	{
		if ((*this)[i]>fMax)
		{
			fMax=(*this)[i];
		}
	}
	return fMax;
}
double VectorN::getMin(void)
{
	double fMin=(*this)[0];
	for (int i=1;i<m_nDim;i++)
	{
		if ((*this)[i]<fMin)
		{
			fMin=(*this)[i];
		}
	}
	return fMin;
}
void VectorN::setCellValue(int dim,double v)
{
	(*this)[dim]=v;
}
Matrix VectorN::toMatrix(void)
{
	Matrix mxTmp(m_matrix);
	return mxTmp;
}
Matrix VectorN::toSkewMatrix(void)
{
	Matrix tmpMx(3,3);
	tmpMx[0][1]=(*this)[2]*(-1);
	tmpMx[1][0]=(*this)[2];
	tmpMx[0][2]=(*this)[1];
	tmpMx[2][0]=(*this)[1]*(-1);
	tmpMx[1][2]=(*this)[0]*(-1);
	tmpMx[2][1]=(*this)[0];
	return tmpMx;
}
void VectorN::putSubVec(int loc,VectorN &subvec)
{
	ASSERT(loc+subvec.m_nDim<=this->m_nDim);
	for(int i=0;i<subvec.m_nDim;i++)
	{
		(*this)[loc+i]=subvec[i];
	}
}
VectorN VectorN::getSubVec(int from,int to)
{
	VectorN tmp(to-from+1);
	for (int i=0;i<to-from+1;i++)
	{
		tmp[i]=(*this)[i+from];
	}
	return tmp;
}
VectorN VectorN::Concat(VectorN &v2)
{
	int dim1 = this->Dim();
	int dim2 = v2.Dim();
	VectorN tmp(dim1 + dim2);
	for (int i = 0; i < dim1; i++)
	{
		tmp[i] = (*this)[i];
	}
	for (int j = 0; j < dim2; j++)
	{
		tmp[dim1 + j] = v2[j];
	}
	return tmp;
}
BOOL VectorN::SaveVectorN()
{
	CString szFilter="ASCII FILES(*.TXT)|*.TXT|BINARY FILES(*.VECB)|*.VECB";
	CFileDialog dlg(FALSE,"VECTORFile.VECB",NULL,OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT,szFilter);
	if (dlg.DoModal()!=IDOK)
	{
		return FALSE;
	}
	CString strExt=dlg.GetFileExt();
	strExt.MakeUpper();
	BOOL bASCII=(strExt=="TXT");
	BOOL bFlag=SaveVectorN(dlg.GetPathName(),bASCII);
	return bFlag;
}
BOOL VectorN::SaveVectorN(CString strPath, BOOL bAscii)
{
	CFile file(strPath,CFile::modeCreate|CFile::modeReadWrite);
	CArchive ar(&file,CArchive::store);
	if (bAscii)
	{
		CString strTmp;
		for (int i=0;i<this->Dim();i++)
		{
			strTmp.Format(_T("%.8lf"),(*this)[i]);
			ar.WriteString(strTmp);
			ar.WriteString(_T("\r\n"));
		}
		return TRUE;
	}
	else
	{
		Serialize(ar);
		return TRUE;
	}
	return FALSE;
}
BOOL VectorN::LoadVectorN(CString strPath)
{
	this->Empty();
	CFile file(strPath,CFile::modeRead);
	CArchive ar(&file,CArchive::load);
	Serialize(ar);
	return TRUE;
}
BOOL VectorN::LoadVectorN()
{
	this->Empty();
	CString szFilter="BINARY FILES(*.VECB)|*.VECB|";
	CFileDialog dlg(TRUE,"BINARY FILES *.VECB",NULL,OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT,szFilter);
	if (dlg.DoModal()==IDOK)
	{
		CString strPath=dlg.GetPathName();
		CFile file(strPath,CFile::modeRead);
		CArchive ar(&file,CArchive::load);
		Serialize(ar);
		return TRUE;
	}
	return FALSE;
}
VectorN operator +(VectorN &vec, double k)
{
	Matrix tmpMx=vec.toMatrix()+k;
	VectorN tmpVec(tmpMx);
	return tmpVec;
}
VectorN operator +(double k, VectorN &vec)
{
	Matrix tmpMx=k+vec.toMatrix();
	VectorN tmpVec(tmpMx);
	return tmpVec;
}
VectorN operator +(VectorN &vec1, VectorN &vec2)
{
	Matrix tmpMx=vec1.toMatrix()+vec2.toMatrix();
	VectorN tmpVec(tmpMx);
	return tmpVec;
}
VectorN operator -(VectorN &vec, double k)
{
	Matrix tmpMx=vec.toMatrix()-k;
	VectorN tmpVec(tmpMx);
	return tmpVec;
}
VectorN operator -(double k, VectorN &vec)
{
	Matrix tmpMx=k-vec.toMatrix();
	VectorN tmpVec(tmpMx);
	return tmpVec;
}
VectorN operator -(VectorN &vec1, VectorN &vec2)
{
	Matrix tmpMx=vec1.toMatrix()-vec2.toMatrix();
	VectorN tmpVec(tmpMx);
	return tmpVec;
}
double operator *(VectorN &vec1,VectorN &vec2)
{
	ASSERT(vec1.Dim()==vec2.Dim());
	double tmp=0.0;
	for (int i=0;i<vec1.Dim();i++)
	{
		tmp+=(vec1[i]*vec2[i]);
	}
	return tmp;
}
VectorN operator *(VectorN &vec, double k)
{
	Matrix tmpMx=vec.toMatrix()*k;
	VectorN tmpVec(tmpMx);
	return tmpVec;
}
VectorN operator *(double k, VectorN &vec)
{
	Matrix mxTmp=k*vec.toMatrix();
	VectorN tmpVec(mxTmp);
	return tmpVec;
}
VectorN operator *(Matrix &mx,VectorN &vec)
{
	Matrix tmpMx=mx*vec.toMatrix();
	VectorN tmpVec(tmpMx);
	return tmpVec;
}