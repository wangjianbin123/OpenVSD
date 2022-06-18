// Matrix.cpp : 实现文件
//
#include "pch.h"
#include "Matrix.h"
#include <vector>

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

IMPLEMENT_SERIAL(Matrix,CObject,VERSIONABLE_SCHEMA|2)

// Matrix
//默认构造函数
Matrix::Matrix()
{
	m_nRow=0;
	m_nCol=0;
	m_pData=NULL;
}
//行列数构造函数，矩阵初始化为0
Matrix::Matrix(int nRow, int nCol)
{
	m_nRow=nRow;
	m_nCol=nCol;
	m_pData=new double *[nRow];
	ASSERT(m_pData!=NULL);
	for (int i=0;i<nRow;i++)
	{
		m_pData[i]=new double [nCol];
		ASSERT(m_pData[i]!=NULL);
		for (int j=0;j<nCol;j++)
		{
			(*this)[i][j]=0;
		}
	}
}
//赋值构造函数
Matrix::Matrix(int nRow, int nCol, double *pAry)
{
	m_nRow=nRow;
	m_nCol=nCol;
	m_pData=new double*[m_nRow];
	ASSERT(m_pData!=NULL);
	for (int i=0;i<m_nRow;i++)
	{
		m_pData[i]=new double[m_nCol];
		ASSERT(m_pData[i]!=NULL);
		for (int j=0;j<m_nCol;j++)
		{
			(*this)[i][j]=pAry[i*m_nCol+j];
		}
	}
}
Matrix::Matrix(int row, double *px, double *py)
{
	m_nRow=row;
	m_nCol=2;
	m_pData=new double *[m_nRow];
	ASSERT(m_pData!=NULL);
	for (int i=0;i<m_nRow;i++)
	{
		m_pData[i]=new double[2];
		ASSERT(m_pData[i]!=NULL);
		(*this)[i][0]=px[i];
		(*this)[i][1]=py[i];
	}
}
//拷贝构造函数
Matrix::Matrix(const Matrix &src)
{
	m_nRow=src.Row();
	m_nCol=src.Col();
	m_pData=new double *[src.Row()];
	ASSERT(m_pData!=NULL);
	for(int i=0;i<src.Row();i++)
	{
		m_pData[i]=new double[src.Col()];
		ASSERT(m_pData[i]!=NULL);
		for (int j=0;j<src.Col();j++)
		{
			(*this)[i][j]=src.getValue(i,j);
		}
	}
}
//析构函数
Matrix::~Matrix()
{
	this->Empty();
}
// 数据流操作
void Matrix::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	if (ar.IsStoring())
	{
		ar<<m_nRow<<m_nCol;
		for (int i=0;i<m_nRow;i++)
		{
			for (int j=0;j<m_nCol;j++)
			{
				ar<<(*this)[i][j];
			}
		}
	}
	else
	{
		int nRow,nCol;
		ar>>nRow>>nCol;
		SetMatrix(nRow,nCol);
		for (int i=0;i<nRow;i++)
		{
			for (int j=0;j<nCol;j++)
			{
				ar>>(*this)[i][j];
			}
		}
	}
}
//清空矩阵
void Matrix::Empty(void)
{
	if(m_pData!=NULL)
	{
		for (int i=0;i<Row();i++)
		{
			delete[] m_pData[i];
		}
		delete[] m_pData;
		m_pData=NULL;
	}
	m_nRow=0;
	m_nCol=0;
}
//判断矩阵是否为空
BOOL Matrix::IsEmpty(void) const
{
	if (m_pData!=NULL)
	{
		return FALSE;
	}
	else
		return TRUE;
}
//设置矩阵行列数，数据单元赋值为0.0
void Matrix::SetMatrix(int nRow, int nCol)
{
	this->Empty();
	m_nRow=nRow;
	m_nCol=nCol;
	//
	m_pData=new double *[nRow];
	ASSERT(m_pData!=NULL);
	for (int i=0;i<nRow;i++)
	{
		m_pData[i]=new double[nCol];
		ASSERT(m_pData[i]!=NULL);
		for (int j=0;j<nCol;j++)
		{
			(*this)[i][j]=0.0;
		}
	}
}
//设置矩阵行列数，向量赋值
void Matrix::SetMatrix(int nRow, int nCol, double *pAry)
{
	ASSERT(pAry!=NULL);
	this->Empty();
	m_nRow=nRow;
	m_nCol=nCol;
	//
	m_pData=new double *[nRow];
	ASSERT(m_pData!=NULL);
	for (int i=0;i<nRow;i++)
	{
		m_pData[i]=new double[nCol];
		ASSERT(m_pData[i]!=NULL);
		for (int j=0;j<nCol;j++)
		{
			(*this)[i][j]=pAry[i*nCol+j];
		}
	}
}
//向量赋值
void Matrix::SetMatrix(double *pAry)
{
	ASSERT(!this->IsEmpty());
	for (int i=0;i<m_nRow;i++)
	{
		for (int j=0;j<m_nCol;j++)
		{
			(*this)[i][j]=pAry[i*m_nCol+j];
		}
	}
}
//返回矩阵行向量
double* &Matrix::operator [](int nRow)
{
	//20210528 ASSERT Canceled
	//ASSERT(m_pData!=NULL);
    //ASSERT((nRow>=0)&&(nRow<m_nRow));
	return m_pData[nRow];
}
//=操作符重载
Matrix& Matrix::operator =(const Matrix &mx)
{
	if(this==&mx)
	{
		return (*this);
	}
	this->Empty();
	m_nRow=mx.Row();
	m_nCol=mx.Col();
	//
	m_pData=new double *[m_nRow];
	ASSERT(m_pData!=NULL);
	for (int i=0;i<m_nRow;i++)
	{
		m_pData[i]=new double [m_nCol];
		ASSERT(m_pData[i]!=NULL);
		for (int j=0;j<m_nCol;j++)
		{
			(*this)[i][j]=mx.getValue(i,j);
		}
	}
	return (*this);
}
//
Matrix& Matrix::operator +=(Matrix &mx)
{
	//ASSERT((mx.Row()==m_nRow)&&(mx.Col()==m_nCol));
	for (int i=0;i<m_nRow;i++)
	{
		for (int j=0;j<m_nCol;j++)
		{
			(*this)[i][j]+=mx[i][j];
		}
	}
	return (*this);
}

Matrix& Matrix::operator -=(Matrix &mx)
{
	//ASSERT((mx.Row()==m_nRow)&&(mx.Col()==m_nCol));
	for (int i=0;i<m_nRow;i++)
	{
		for (int j=0;j<m_nCol;j++)
		{
			(*this)[i][j]-=mx[i][j];
		}
	}
	return (*this);
}
Matrix& Matrix::operator *=(Matrix &mx)
{
	//ASSERT((*this).Col()==mx.Row());
	Matrix mxTmp(*this);
	this->Empty();
	this->SetMatrix(mxTmp.Row(),mx.Col());
	for (int i=0;i<mxTmp.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			for (int k=0;k<mxTmp.Col();k++)
			{
				(*this)[i][j]+=mxTmp[i][k]*mx[k][j];
			}
		}
	}
	return (*this);
}
int Matrix::Pivot(int nRow)
{
	double fMax=(*this)[nRow][nRow];
	int nMaxRow=nRow;
	for (int i=nRow+1;i<m_nRow;i++)
	{
		double fTmp=fabs((*this)[i][nRow]);
		if (fTmp>fMax)
		{
			fMax=fTmp;
			nMaxRow=i;
		}
	}
	if (fabs((*this)[nMaxRow][nRow])<EPSINON)
	{
		return -1;
	}
	if (nMaxRow!=nRow)
	{
		SwapRow(nRow,nMaxRow);
		return nMaxRow;
	}
	return 0;
}
void Matrix::SwapRow(int nRow1, int nRow2)
{
	//ASSERT((nRow1>=0)&&(nRow1<m_nRow));
	//ASSERT((nRow2>=0)&&(nRow2<m_nRow));
	//if (nRow1==nRow2)
	//{
	//	return;
	//}
	double *p;
	p=m_pData[nRow1];
	m_pData[nRow1]=m_pData[nRow2];
	m_pData[nRow2]=p;
}
void Matrix::SwapCol(int nCol1, int nCol2)
{
	//ASSERT((nCol1>=0)&&(nCol1<m_nCol));
	//ASSERT((nCol2>=0)&&(nCol2<m_nCol));
	//if (nCol1==nCol2)
	//{
	//	return;
	//}
	double fTmp;
	for(int i=0;i<m_nRow;i++)
	{
		fTmp=(*this)[i][nCol1];
		(*this)[i][nCol1]=(*this)[i][nCol2];
		(*this)[i][nCol2]=fTmp;
	}
}
void Matrix::toUnit(void)
{
	for (int i=0;i<m_nRow;i++)
	{
		for (int j=0;j<m_nCol;j++)
		{
			if (i==j)
			{
				(*this)[i][j]=1.0;
			}
			else
			{
				(*this)[i][j]=0;
			}
		}
	}
}
void Matrix::ColtoUnit(int num)
{
	double sum=0.0;
	int row=this->Row();
	for (int i=0;i<row;i++)
	{
		sum+=pow((*this)[i][num],2);
	}
	double root=sqrt(sum);
	for(int j=0;j<row;j++)
	{
		(*this)[j][num]/=root;
	}
}
void Matrix::toZero(void)
{
	for (int i=0;i<m_nRow;i++)
	{
		for (int j=0;j<m_nCol;j++)
		{
			(*this)[i][j]=0;
		}
	}
}
double Matrix::MaxCell(void)
{
	double fMax=(*this)[0][0];
	for (int i=0;i<m_nRow;i++)
	{
		for (int j=0;j<m_nCol;j++)
		{
			if ((*this)[i][j]>fMax)
			{
				fMax=(*this)[i][j];
			}
		}
	}
	return fMax;
}
double Matrix::MinCell(void)
{
	double fMin=(*this)[0][0];
	for (int i=0;i<m_nRow;i++)
	{
		for (int j=0;j<m_nCol;j++)
		{
			if ((*this)[i][j]<fMin)
			{
				fMin=(*this)[i][j];
			}
		}
	}
	return fMin;
}
double Matrix::Det(void)
{
	//ASSERT(m_nRow==m_nCol);
	double fDet=1.0f;
	int *pI,*pJ;
	pI=new int[this->Row()];
	ASSERT(pI!=NULL);
	pJ=new int[this->Col()];
	ASSERT(pJ!=NULL);
	Matrix mxTmp(*this);
	int nSign=1;
	int k,i,j;
	for (k=0;k<mxTmp.Row();k++)
	{
		pI[k]=-1;
		pJ[k]=-1;
	}
	for (k=0;k<mxTmp.Row();k++)
	{
		double fMax=MINDOUBLE;
		for (i=k;i<mxTmp.Row();i++)
		{
			for (j=k;j<mxTmp.Col();j++)
			{
				double fTmp=fabs(mxTmp[i][j]);
				if (fTmp>fMax)
				{
					fMax=fTmp;
					pI[k]=i;
					pJ[k]=j;
				}
			}
		}
		if (fabs(fMax)<EPSINON)
		{
			return 0;
		}
		if (pI[k]!=k)
		{
			nSign=-nSign;
			mxTmp.SwapRow(k,pI[k]);
		}
		if (pJ[k]!=k)
		{
			nSign=-nSign;
			mxTmp.SwapCol(k,pJ[k]);
		}
		fDet*=mxTmp[k][k];
		mxTmp[k][k]=1.0f/mxTmp[k][k];
		for (j=0;j<mxTmp.Col();j++)
		{
			if (j!=k)
			{
				mxTmp[k][j]*=mxTmp[k][k];
			}
		}
		for (i=0;i<mxTmp.Row();i++)
		{
			if (i!=k)
			{
				for (j=0;j<mxTmp.Col();j++)
				{
					if (j!=k)
					{
						mxTmp[i][j]=mxTmp[i][j]-mxTmp[i][k]*mxTmp[k][j];
					}
				}
			}
		}
		for (i=0;i<mxTmp.Row();i++)
		{
			if (i!=k)
			{
				mxTmp[i][k]*=-mxTmp[k][k];
			}
		}
	}
	for (k=mxTmp.Row()-1;k>=0;k--)
	{
		if ((pJ[k]!=-1)&&(pJ[k]!=k))
		{
			mxTmp.SwapCol(k,pJ[k]);
		}
		if ((pI[k]!=-1)&&(pI[k]!=k))
		{
			mxTmp.SwapRow(k,pI[k]);
		}
	}
	delete[] pI;
	delete[] pJ;
	return fDet*nSign;
}
Matrix Matrix::getCol(int col)
{
	Matrix tmpMx(m_nRow,1);
	for (int i=0;i<m_nRow;i++)
	{
		tmpMx[i][0]=(*this)[i][col];
	}
	return tmpMx;
}
Matrix Matrix::getDoubleCols(int col1, int col2)
{
	Matrix tmpMx(m_nRow,2);
	for (int i=0;i<m_nRow;i++)
	{
		tmpMx[i][0]=(*this)[i][col1];
		tmpMx[i][1]=(*this)[i][col2];
	}
	return tmpMx;
}
Matrix Matrix::getSubMat(int i, int j, int iNum, int jNum)
{
	ASSERT((i>=0)&&((i+iNum-1)<m_nRow));
	ASSERT((j>=0)&&((j+jNum-1)<m_nCol));
	Matrix tmpMx(iNum,jNum);
	for (int ii=i;ii<i+iNum;ii++)
	{
		for (int jj=j;jj<j+jNum;jj++)
		{
			tmpMx[ii-i][jj-j]=(*this)[ii][jj];
		}
	}
	return tmpMx;
}
void Matrix::putSubMat(int i, int j, Matrix &mx)
{
	int iNum=mx.Row();
	int jNum=mx.Col();
	ASSERT((i>=0)&&((i+iNum-1)<m_nRow));
	ASSERT((j>=0)&&((j+jNum-1)<m_nCol));
	for (int ii=i;ii<i+iNum;ii++)
	{
		for (int jj=j;jj<j+jNum;jj++)
		{
			(*this)[ii][jj]=mx[ii-i][jj-j];
		}
	}
}
void Matrix::InverseSort(void)
{
	int row=this->Row();
	int col=this->Col();
	Matrix tmp(*this);
	for (int i=0;i<row;i++)
	{
		for(int j=0;j<col;j++)
		{
			(*this)[i][j]=tmp[row-i-1][j];
		}
	}
}

Matrix Matrix::getInverseSort(void)
{
	int row = this->Row();
	int col = this->Col();
	Matrix tmp(row,col);
	for (int i = 0; i<row; i++)
	{
		for (int j = 0; j<col; j++)
		{
			tmp[i][j] = (*this)[row - i - 1][j];
		}
	}
	return tmp;
}
Matrix Matrix::Inv(void)
{
	int n = this->Col();
	Matrix b(n, n);
	b.toUnit();
	return this->LUEqu(b);
}

Matrix Matrix::IterationInv(void)
{
	int n = this->Col();
	Matrix V0(n,n);
	Matrix A(*this);
	//
	
	for (int i = 0; i < n; i++)
	{
		V0[i][i] = 1.0 / A[i][i];
	}
	//
	Matrix V1 = V0;
	Matrix I(n, n); 
	I.toUnit();
	for (int ii = 0; ii < 5; ii++)
	{
		V1 = (I + 0.25*(I - V0*A)*(3 * I - V0*A)*(3 * I - V0*A))*V0;
		V0 = V1;
	}
	return V0;
}
//Thomas法线性方程组，对角系数矩阵
Matrix Matrix::thomasEq(Matrix &b)
{
	Matrix x(m_nRow,1);
	Matrix w(m_nRow,1);
	Matrix c(m_nRow,1);
	Matrix d(m_nRow,1);
	Matrix e(m_nRow,1);
	//
	for (int i=0;i<m_nRow;i++)
	{
		d[i][0]=(*this)[i][i];
	    if (i<(m_nRow-1))
		{
			e[i][0]=(*this)[i][i+1];
		}
		if (i>=1)
		{
			c[i][0]=(*this)[i][i-1];
		}
	}
	for (int i=1;i<m_nRow;i++)
	{
		c[i][0]/=d[i-1][0];
		d[i][0]-=c[i][0]*e[i-1][0];
	}
	for (int i=1;i<m_nRow;i++)
	{
		b[i][0]-=c[i][0]*b[i-1][0];
	}
	x[m_nRow-1][0]=b[m_nRow-1][0]/d[m_nRow-1][0];
	for (int i=m_nRow-2;i>=0;i--)
	{
		x[i][0]=(b[i][0]-e[i][0]*x[i+1][0])/d[i][0];
	}
    return x;
}
//LU分解求解线性方程组
Matrix Matrix::LUEqu(Matrix &b)
{
	int n = this->Col();
	int m = this->Row();
	Matrix l = this->LUC()[0];
	Matrix u = this->LUC()[1];
	Matrix x(n,n);
	int i=0, j=0, k=0;
	double z = 0;
	for (i = 0; i < n; i++)//by column
	{
		Matrix w(n, 1);
		w[0][0] = b[0][i] / l[0][0];
		for (j = 0; j < n; j++)
		{
			z = 0;
			for (k = 0; k <= j-1; k++)
			{
				z += l[j][k] * w[k][0];
			}
			w[j][0] = (b[j][i] - z) / l[j][j];
		}
		for (j = n - 1; j >= 0; j--)
		{
			z = 0;
			for (k = j + 1; k < n; k++)
			{
				z += u[j][k] * x[k][i];
			}
			x[j][i] = w[j][0] - z;
		}
	}
	return x;
}
std::vector<Matrix> Matrix::QR(void)
{
	std::vector<Matrix> res;int col=this->Col();
	double m,n;
	Matrix tmp(*this);
	Matrix tmpcol;
	tmp.ColtoUnit(0);
	for (int i=1;i<col;i++)
	{
		tmpcol=tmp.getCol(i);
		for(int j=0;j<=i-1;j++)
		{
			m=(~tmpcol*tmp.getCol(j))[0][0];
			tmpcol-=m*tmp.getCol(j);
		}
		tmpcol.ColtoUnit(0);
		tmp.putSubMat(0,i,tmpcol);
	}
	res.push_back(tmp);
	Matrix R(col,col);//R
	for (int k=0;k<col;k++)
	{
		for (int l=k;l<col;l++)
		{
			n=(~(tmp.getCol(k))*this->getCol(l))[0][0];
			R[k][l]=n;
		}
	}
	res.push_back(R);
	return res;
}
std::vector<Matrix> Matrix::LUC(void)
{
	std::vector<Matrix> res;
	int m = this->Row();
	int n = this->Col();
	
	//ASSERT(m == n);
	Matrix l(m+1, n+1), u(m+1, n+1);
	u.toUnit();
	int i = 0, j = 0, k = 0,r = 0;
	double z = 0;
	for (i = 1; i <= n; i++)
		l[i][1] = (*this)[i-1][0];
	for (j = 2; j <= n; j++)
		u[1][j] = (*this)[0][j-1] / l[1][1];
	//
	for (j = 2; j <= n - 1; j++)
	{
		for (i = j; i <= n; i++)
		{
			z = 0;
			for (k = 1; k <= j - 1; k++)
			{
				z += l[i][k] * u[k][j];
			}
			l[i][j] = (*this)[i-1][j-1] - z;
		}
		for (k = j+1; k <= n; k++)
		{
			z = 0;
			for (r = 1; r <= j - 1; r++)
			{
				z += l[j][r] * u[r][k];
			}
			u[j][k] = ((*this)[j-1][k-1] - z) / l[j][j];
		}
	}
	z = 0;
	for (k = 1; k <= n - 1; k++)
	{
		z += l[n][k] * u[k][n];
	}
	l[n][n] = (*this)[n - 1][n - 1] - z;
	res.push_back(l.getSubMat(1,1,n,n));
	res.push_back(u.getSubMat(1,1,n,n));
	return res;
}
std::vector<Matrix> Matrix::LUD(void)
{
	std::vector<Matrix> res;
	int m = this->Row();
	int n = this->Col();
	Matrix l(m + 1, n + 1), u(m + 1, n + 1);
	l.toUnit();
	int i = 0, j = 0, k = 0, r = 0;
	double z = 0;
	for (j = 1; j <= n; j++)
		u[1][j] = (*this)[0][j - 1];
	for (i = 2; i <= n; i++)
		l[i][1] = (*this)[i - 1][0] / u[1][1];
	//
	for (i = 2; i <= n - 1; i++)
	{
		for (j = i; j <= n; j++)
		{
			z = 0;
			for (k = 1; k <= i- 1; k++)
			{
				z += l[i][k] * u[k][j];
			}
			u[i][j] = (*this)[i - 1][j - 1] - z;
		}
		for (k = i + 1; k <= n; k++)
		{
			z = 0;
			for (r = 1; r <= i - 1; r++)
			{
				z += l[k][r] * u[r][i];
			}
			l[k][i] = ((*this)[k- 1][i - 1] - z) / u[i][i];
		}
	}
	z = 0;
	for (k = 1; k <= n - 1; k++)
	{
		z += l[n][k] * u[k][n];
	}
	u[n][n] = (*this)[n - 1][n - 1] - z;
	res.push_back(l.getSubMat(1, 1, n, n));
	res.push_back(u.getSubMat(1, 1, n, n));
	return res;
}
BOOL Matrix::SaveAscii()
{
	CString szFilter="ASCII FILES(*.TXT)|*.TXT|BINARY FILES(*.MXB)|*.MXB";
	CFileDialog dlg(FALSE,"MatrixFile.MXB",NULL,OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT,szFilter);
	if (dlg.DoModal()!=IDOK)
	{
		return FALSE;
	}
	CString strExt=dlg.GetFileExt();
	strExt.MakeUpper();
	BOOL bASCII=(strExt=="TXT");
	BOOL bFlag=SaveMatrix(dlg.GetPathName(),bASCII);
	return bFlag;
}
BOOL Matrix::SaveMatrix(CString strPath, BOOL bAscii)
{
	CFile file(strPath,CFile::modeCreate|CFile::modeReadWrite);
	CArchive ar(&file,CArchive::store);
	if (bAscii)
	{
		CString strTmp, strRow;
		for (int i=0;i<m_nRow;i++)
		{
			strRow="";
			for (int j=0;j<m_nCol;j++)
			{
				strTmp.Format("%.16lf",(*this)[i][j]);
				strRow+=strTmp;
				strRow+=(" ");
			}
			ar.WriteString(strRow);
			ar.WriteString("\r\n");
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
BOOL Matrix::LoadMatrix(CString strPath)
{
	this->Empty();
	CFile file(strPath,CFile::modeRead);
	CArchive ar(&file,CArchive::load);
	Serialize(ar);
	return TRUE;
}
BOOL Matrix::LoadMatrix()
{
	this->Empty();
	//CString szFilter=_T("BINARY FILES(*.MXB)|*.MXB|");
	CString szFilter="BINARY FILES(*.MXB)|*.MXB|";
	CFileDialog dlg(TRUE,_T("BINARY FILES *.MXB"),NULL,OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT,szFilter);
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

BOOL Matrix::LoadAscii()
{
	this->Empty();
	int Row=0;
	int Col;
	// read original data file of Unicode ascii format
	CString strPath;
	CFileDialog dlg(TRUE);
	if (dlg.DoModal()==IDOK)
	{
		strPath=dlg.GetPathName();
	}
	else
	{
		return FALSE;
	}
	CFile Unicode_ascii(strPath,CFile::modeRead);
	//skip the first 2 bytes of BOM
	Unicode_ascii.Seek(0,CFile::begin);
	CArchive ar(&Unicode_ascii,CArchive::load);
	std::vector<double> Data;
	double tmpData;
	CString tmpInput;
	char * getWord;
	char * tmpLine;
	CString csTok=" ,\t\t";
	char * Tok=(char *)csTok.GetBuffer(csTok.GetLength()+1);
	char * nextTok=NULL;
	while (ar.ReadString(tmpInput))
	{
		tmpLine=(char *)tmpInput.GetBuffer(tmpInput.GetLength()+1);
		Row++;
		Col=0;
		getWord=strtok_s(tmpLine,Tok,&nextTok);
		while (getWord!=NULL)
		{
			Col++;			
			tmpData=atof(getWord);
			Data.push_back(tmpData);
			getWord=strtok_s(NULL,Tok,&nextTok);
		}
	}
	double *pData=new double[Row*Col];
	for (int i=0;i<Row*Col;i++)
	{
		pData[i]=Data[i];
	}
	this->SetMatrix(Row,Col,pData);
	
	delete[] pData;
	Data.clear();
	return TRUE;
}

//操作符重载
Matrix operator +(Matrix &mx, double k)
{
	Matrix mxTmp(mx.Row(),mx.Col());
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[i][j]=mx[i][j]+k;
		}
	}
	return mxTmp;
}
Matrix operator +(double k, Matrix &mx)
{
	Matrix mxTmp(mx.Row(),mx.Col());
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[i][j]=mx[i][j]+k;
		}
	}
	return mxTmp;
}
Matrix operator +(Matrix &mx1, Matrix &mx2)
{
	ASSERT((mx1.Row()==mx2.Row())&&(mx1.Col()==mx2.Col()));
	Matrix mxTmp(mx1.Row(),mx1.Col());
	for (int i=0;i<mx1.Row();i++)
	{
		for (int j=0;j<mx1.Col();j++)
		{
			mxTmp[i][j]=mx1[i][j]+mx2[i][j];
		}
	}
	return mxTmp;
}
Matrix operator -(Matrix &mx, double k)
{
	Matrix mxTmp(mx.Row(),mx.Col());
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[i][j]=mx[i][j]-k;
		}
	}
	return mxTmp;
}
Matrix operator -(double k, Matrix &mx)
{
	Matrix mxTmp(mx.Row(),mx.Col());
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[i][j]=k-mx[i][j];
		}
	}
	return mxTmp;
}
Matrix operator -(Matrix &mx1, Matrix &mx2)
{
	ASSERT((mx1.Row()==mx2.Row())&&(mx1.Col()==mx2.Col()));
	Matrix mxTmp(mx1.Row(),mx1.Col());
	for (int i=0;i<mx1.Row();i++)
	{
		for (int j=0;j<mx1.Col();j++)
		{
			mxTmp[i][j]=mx1[i][j]-mx2[i][j];
		}
	}
	return mxTmp;
}
Matrix operator *(Matrix &mx, double k)
{
	Matrix mxTmp(mx.Row(),mx.Col());
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[i][j]=mx[i][j]*k;
		}
	}
	return mxTmp;
}
Matrix operator *(double k, Matrix &mx)
{
	Matrix mxTmp(mx.Row(),mx.Col());
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[i][j]=k*mx[i][j];
		}
	}
	return mxTmp;
}
Matrix operator *(Matrix &mx1, Matrix &mx2)
{
	ASSERT(mx1.Col()==mx2.Row());
	Matrix mxTmp(mx1.Row(),mx2.Col());
	for (int i=0;i<mx1.Row();i++)
	{
		for (int j=0;j<mx2.Col();j++)
		{
			for (int k=0;k<mx1.Col();k++)
			{
				mxTmp[i][j]+=mx1[i][k]*mx2[k][j];
			}
		}
	}
	return mxTmp;
}
Matrix operator /(Matrix &mx,double k)
{
	ASSERT(k!=0.0);
	Matrix mxTmp=mx;
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[i][j]/=k;
		}
	}
	return mxTmp;
}
Matrix operator ~(Matrix &mx)
{
	Matrix mxTmp(mx.Col(),mx.Row());
	for (int i=0;i<mx.Row();i++)
	{
		for (int j=0;j<mx.Col();j++)
		{
			mxTmp[j][i]=mx[i][j];
		}
	}
	return mxTmp;
}

Matrix operator !(Matrix &mx)//20130619 modified
{
	ASSERT(mx.Row()==mx.Col());
	int n=mx.Row();
	Matrix t(mx),B(n,n);
	B.toUnit();
	int i,j,k;
	double max,temp;
	for (i = 0; i < n; i++)
    {
		max = t[i][i];
        k = i;
        for (j = i+1; j < n; j++)
        {
            if (fabs(t[j][i]) > fabs(max))
            {
                max = t[j][i];
                k = j;
            }
        }
        if (k != i)
        {
            for (j = 0; j < n; j++)
            {
                temp = t[i][j];
                t[i][j] = t[k][j];
                t[k][j] = temp;
                temp = B[i][j];
                B[i][j] = B[k][j];
                B[k][j] = temp; 
            }
        }
        temp = t[i][i];
        for (j = 0; j < n; j++)
        {
            t[i][j] = t[i][j] / temp;
            B[i][j] = B[i][j] / temp;
        }
        for (j = 0; j < n; j++)
        {
            if (j != i)
            {
                temp = t[j][i];
                for (k = 0; k < n; k++)
                {
                    t[j][k] = t[j][k] - t[i][k]*temp;
                    B[j][k] = B[j][k] - B[i][k]*temp;
                }
            }
        }
    }
	return B;
}






























