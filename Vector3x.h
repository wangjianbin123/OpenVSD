#pragma once

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif


// Vector3x 命令目标

class Vector3x : public CObject
{
public:
	Vector3x();
	Vector3x(double Rx, double Ry, double Rz);
	Vector3x(VectorN &vec);
	Vector3x(const Vector3x &sc);//090404 -> const function

	//操作符重载

	Vector3x &operator=(const Vector3x &vec3); //090404 ->const function
	Vector3x &operator+=(Vector3x &vec3);
	Vector3x &operator-=(Vector3x &vec3);
	Vector3x &operator*=(double k);
	double &operator[](int dim);
    
	//文件流操作
	void Serialize(CArchive &ar);
	DECLARE_SERIAL(Vector3x)
    
	void Empty(void);
	double getRx(void);
	double getRy(void);
	double getRz(void);
	double getNorm(void);
	void reSet(double x,double y,double z);
	void toUnit(void);
	Matrix toSkewMatrix(void);
	VectorN toVectorN(void);
	Matrix toMatrix(void);
	double dotProduct(Vector3x &vec3);
	Vector3x crossProduct(Vector3x &vec3);

	virtual ~Vector3x();

	double getValue(int i) const {  return m_vec.getValue(i);}; 
	VectorN getVecN(void) const {return m_vec;};
	
	//友元函数
	friend Vector3x operator+(Vector3x &vec,double k);
	friend Vector3x operator+(double k,Vector3x &vec);
	friend Vector3x operator+(Vector3x &vec1, Vector3x &vec2);

	friend Vector3x operator-(Vector3x &vec,double k);
	friend Vector3x operator-(double k,Vector3x &vec);
	friend Vector3x operator-(Vector3x &vec1, Vector3x &vec2);

	friend Vector3x operator*(Vector3x &vec,double k);
	friend Vector3x operator*(double k,Vector3x &vec);
	friend double operator*(Vector3x &vec1, Vector3x &vec2);
	friend Vector3x operator*(Matrix &mx,Vector3x &vec3);

private:
	VectorN m_vec;
};
