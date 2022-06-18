#pragma once

// Matrix
// Version 1.0.2
// 王建斌 2008.12.30
#include <vector>

class Matrix : public CObject
{
public:
	
	//构造函数 1 默认构造 2 赋值构造 3 拷贝构造
	Matrix();
	Matrix(int nRow,int nCol);
	Matrix(int nRow,int nCol,double * pAry);
	Matrix(int row,double *px,double *py); 
	Matrix(const Matrix &src);
    
	//析构函数
	virtual ~Matrix();

	//文件流操作
	void Serialize(CArchive &ar);
	DECLARE_SERIAL(Matrix)

	//运算符重载
	double * & operator[](int nRow);
	Matrix& operator =(const Matrix &mx); 
	Matrix& operator +=(Matrix &mx);
	Matrix& operator -=(Matrix &mx);
	Matrix& operator *=(Matrix &mx);

	//矩阵计算及操作
	void Empty(void);
	BOOL IsEmpty(void) const; 
	void SetMatrix(int nRow,int nCol);
	void SetMatrix(int nRow,int nCol,double *pAry);
	void SetMatrix(double *pAry);
	int Pivot(int nRow);
	void SwapRow(int nRow1,int nRow2);
	void SwapCol(int nCol1,int nCol2);
	void toUnit(void);
	void ColtoUnit(int num);
	void toZero(void);
	double MaxCell(void);
	double MinCell(void);
	double Det(void);
	Matrix getCol(int col);
	Matrix getDoubleCols(int col1,int col2);
	Matrix getSubMat(int i,int j,int iNum,int jNum); 
	void putSubMat(int i, int j, Matrix &mx);
	void InverseSort(void);
	Matrix getInverseSort(void);
	Matrix Inv(void);
	//20210525
	Matrix IterationInv(void);

	//Thomas法求解线性方程组
	Matrix thomasEq(Matrix &b);
	Matrix LUEqu(Matrix &b);
	int Row() const {return m_nRow;}
	int Col() const {return m_nCol;}
	double getValue(int i, int j) const { return m_pData[i][j];} 

	std::vector<Matrix> QR(void);
	std::vector<Matrix> LUC(void);
	std::vector<Matrix> LUD(void);

	//矩阵文件操作
	BOOL LoadMatrix();
	BOOL LoadMatrix(CString strPath);
	BOOL LoadAscii();//20130608 read ascii file
	BOOL SaveMatrix(CString strPath, BOOL bAscii=TRUE);
	BOOL SaveAscii();//20130608 save ascii file

	//友元函数
	friend Matrix operator +(Matrix &mx,double k);
	friend Matrix operator +(double k,Matrix &mx);
	friend Matrix operator +(Matrix &mx1,Matrix &mx2);
	friend Matrix operator -(Matrix &mx,double k);
	friend Matrix operator -(double k,Matrix &mx);
	friend Matrix operator -(Matrix &mx1,Matrix &mx2);
	friend Matrix operator *(Matrix &mx,double k);
	friend Matrix operator *(double k, Matrix &mx);
	friend Matrix operator *(Matrix &mx1,Matrix &mx2);
	friend Matrix operator /(Matrix &mx,double k);
	friend Matrix operator ~(Matrix &mx);
	friend Matrix operator !(Matrix &mx);

private:
	int m_nRow;
	int m_nCol;
	double **m_pData;
};


