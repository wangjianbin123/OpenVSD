#pragma once

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _MATH_H_
#define _MATH_H_
#include <math.h>
#endif

// VectorN����
// Version 1.0.1
// ������ 2009.2.14

class VectorN : public CObject
{
public:
	
	//���캯��
	VectorN();
	VectorN(int nDim);
	VectorN(int nDim, double * pAry);
	VectorN(Matrix &mx);
	VectorN(const VectorN &src); //090404 -> const function

	//��������
    virtual ~VectorN();

	//�ļ�������
	void Serialize(CArchive &ar);
	DECLARE_SERIAL(VectorN)

	//���������
	double  & operator[](int nDim);
	VectorN & operator=(const VectorN &vec);
	VectorN & operator+=(VectorN &vec);
	VectorN & operator-=(VectorN &vec);
	VectorN & operator*=(double k);
	VectorN & operator/=(double k);

	//ʸ������
	void Empty(void);
	BOOL IsEmpty(void);
	void SetVectorN(int nDim);
	void SetVectorN(int nDim, double *pAry);
	void SetVectorN(double *pAry);
	void SetVectorN(Matrix &mx);
	void toZero(void);
	double getNorm(void);
	double getSum(void);
	void toUnit(void);
	double getMax(void);
	double getMin(void);
	void setCellValue(int dim,double v);
	Matrix toMatrix(void);
	Matrix toSkewMatrix(void);
	void putSubVec(int loc,VectorN &subvec);
	VectorN getSubVec(int from,int to);
	VectorN Concat(VectorN &v2);
	
	int Dim(void) const {return m_nDim;}
	Matrix getMat(void) const {return m_matrix;};
	double getValue(int i) const {return m_matrix.getValue(i,0);} 

	//ʸ���ļ�����
	BOOL LoadVectorN();
	BOOL LoadVectorN(CString strPath);
	BOOL SaveVectorN(CString strPath, BOOL bAscii=TRUE);
	BOOL SaveVectorN();

	//��Ԫ����
	friend VectorN operator +(VectorN &vec, double k);
	friend VectorN operator +(double k, VectorN &vec);
	friend VectorN operator +(VectorN &vec1, VectorN &vec2);
	friend VectorN operator -(VectorN &vec, double k);
	friend VectorN operator -(double k, VectorN &vec);
	friend VectorN operator -(VectorN &vec1, VectorN &vec2);
	friend double operator *(VectorN &vec1,VectorN &vec2);
	friend VectorN operator *(VectorN &vec, double k);
	friend VectorN operator *(double k, VectorN &vec);
	friend VectorN operator *(Matrix &mx,VectorN &vec);
    	
private:
	int m_nDim;
	Matrix m_matrix;
};