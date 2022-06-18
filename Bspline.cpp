// Bspline.cpp : 实现文件
//
#include "pch.h"
#include "Bspline.h"


// Bspline

Bspline::Bspline()
{
	eps = 1e-8;
	mx0.Empty();
}
Bspline::Bspline(Matrix &mx)
{
	eps = 1e-8;
	mx0 = mx;
}
Bspline::Bspline(const Bspline &bsp)
{
	eps = bsp.GetEps();
	mx0 = bsp.GetMatrix();
}
void Bspline::SetEps(double Eps)
{
	if (Eps < 0.1)
		return;
	else
		eps = Eps;
}
double Bspline::GetEps(void) const
{
	return eps;
}
Matrix Bspline::GetMatrix(void) const
{
	return mx0;
}
double Bspline::N(int i, int k, double su[], double u)
{
	if (k == 0)
	{
		if (u - su[0] < eps)
		{
			if (i == 0)
				return 1.0;
			else
				return 0.0;
		}
		if (u >= su[i] && u < su[i + 1])
			return 1.0;
		else
			return 0.0;
	}
	else if (k>0)
	{
		if (MBiger(su[i + k] - su[i], eps) && MBiger(su[i + k + 1] - su[i + 1], eps))
		{
			return (u - su[i] * N(i, k - 1, su, u) / (su[i + k] - su[i])) + (su[i + k + 1] - u)*N(i + 1, k - 1, su, u) / (su[i + k + 1] - su[i + 1]);
		}
		else if (!MBiger(su[i + k] - su[i], eps) && MBiger(su[i + k + 1] - su[i + 1], eps))
		{
			return (su[i + k + 1] - u)*N(i + 1, k - 1, su, u) / (su[i + k + 1] - su[i + 1]);
		}
		else if (MBiger(su[i + k] - su[i], eps) && !MBiger(su[i + k + 1] - su[i + 1], eps))
		{
			return (u - su[i])*N(i, k - 1, su, u) / (su[i + k] - su[i]);
		}
		else
			return 0.0;
	}
	else
		return 0.0;
}
double Bspline::BSL1(int k, double su[], double d[], int n, double u)
{
	int i;
	double s = 0.0;
	for (i = 0; i < n; i++){
		s = s + d[i] * N(i, k, su, u);
	}
	return s;
}
double Bspline::BSL(int k, double su[], double d[], int n, double u)
{
	int i;
	for (i = 1; i < n + k; i++){
		if (u < su[i])
			break;
		if (i>k&&u == su[i])
			break;
	}
	i--;
	return d1(k, i, k, su, d, u);
}
double Bspline::d1(int I, int j, int k, double su[], double d[], double u)
{
	if (I <= 0)
		return d[j];
	double a = this->alpha(I, j, k, su, u);
	return (1 - a)*d1(I - 1, j - 1, k, su, d, u) + a*d1(I - 1, j, k, su, d, u);
}
double Bspline::d2(int I, int j, int k, double su[], double d[], double u)
{
	if (I <= 0)
		return d[j];
	double z = su[j + k + 1 - I] - su[j];
	if (z<eps&&z>(-1.0 * eps))
		return 0.0;
	return (k + 1 - I)*(d2(I - 1, j, k, su, d, u) - d2(I - 1, j - 1, k, su, d, u)) / z;
}
double Bspline::GetDerValue(int k, double su[], double d[], int n, int r, double u)
{
	int i, j;
	double s = 0.0;
	for (i = 1; i < n + k; i++){
		if (u < su[i])
			break;
		if (i>k&&u == su[i])
			break;
	}
	i--;
	for (j = i - k + r; j <= i; j++){
		s += d2(r, j, k, su, d, u)*N(j, k - r, su, u);
	}
	return s;
}
double Bspline::BSS(int k, double su[], int I, double sv[], double **d, int m, double u, int n, double v)
{
	int i, j;
	double s = 0.0;
	for (i = 0; i < m; i++){
		for (j = 0; j < n; j++){
			s = s + d[i][j] * N(i, k, su, u)*N(j, I, sv, v);
		}
	}
	return s;
}
double Bspline::NURBSL(int k, double su[], double d[], double w[], int n, double u)
{
	int i, t = k;
	double s1 = 0.0, s2 = 0.0, wN;
	for (i = 0; i < n + 1; i++){
		if (u <= su[i]){
			t = i - 1;
			break;
		}
	}
	if (t < k)
		t = k;
	for (i = t - k; i <= t; i++){
		wN = w[i] * N(i, k, su, u);
		s1 = s1 + d[i] * wN;
		s2 = s2 + wN;
	}
	if (s2>eps || s2 < -eps)
		return s1 / s2;
	else
		return 0.0;
}
double Bspline::NURBSS(int k, double su[], int I, double sv[], double **d, double **w, int m, double u, int n, double v)
{
	int i, j, tu = k, tv = I;
	double s1 = 0.0, s2 = 0.0, wN;
	for (i = 0; i < m + 1; i++){
		if (u <= su[i]){
			tu = i - 1;
			break;
		}
	}
	for (j = 0; j < n + 1; j++){
		if (v <= sv[j]){
			tv = j - 1;
			break;
		}
	}
	if (tu < k)
		tu = k;
	if (tv < I)
		tv = I;
	for (i = tu - k; i <= tu; i++){
		for (j = tv - 1; j <= tv; j++){
			wN = w[i][j] * this->N(i, k, su, u)*this->N(j, I, sv, v);
			s1 = s1 + d[i][j] * wN;
			s2 = s2 + wN;
		}
	}
	if (s2>eps || s2 < -eps)
		return s1 / s2;
	else
		return 0.0;
}

double Bspline::UBSL(int k, double d[], int n, double u)
{
	double s, *su = new double[n + k + 1];
	int i;
	for (i = 0; i < n + k + 1; i++){
		su[i] = double(i) / (n + k);
	}
	s = BSL(k, su, d, n, u);
	delete[]su;
	return s;
}

double Bspline::QUBSL(int k, double d[], int n, double u)
{
	double s, *su = new double[n + k + 1];
	int i;
	for (i = 0; i < k; i++){
		su[i] = 0.0;
	}
	for (i = k; i < n + k + 1; i++){
		su[i] = double(i - k) / (n - k);
	}
	for (i = n + 1; i < n + k + 1; i++){
		su[i] = 1.0;
	}
	s = BSL(k, su, d, n, u);
	delete[]su;
	return s;
}
double Bspline::UBSS(int k, int I, double **d, int m, double u, int n, double v)
{
	double s, *su = new double[m + k + 1], *sv = new double[n + I + 1];
	int i;
	for (i = 0; i < m + k + 1; i++){
		su[i] = double(i) / (m + k);
	}
	for (i = 0; i < n + I + 1; i++){
		sv[i] = double(i) / (n + I);
	}
	s = BSS(k, su, I, sv, d, m, u, n, v);
	delete[]su;
	delete[]sv;
	return s;
}
double Bspline::QUBSS(int k, int I, double **d, int m, double u, int n, double v)
{
	double s, *su = new double[m + k + 1], *sv = new double[n + I + 1];
	int i;
	for (i = 0; i < k; i++){
		su[i] = 0.0;
	}
	for (i = k; i <= m; i++){
		su[i] = double(i - k) / (m - k);
	}
	for (i = m + 1; i < m + k + 1; i++){
		su[i] = 1.0;
	}
	for (i = 0; i < I; i++){
		sv[i] = 0.0;
	}
	for (i = I; i <= n; i++){
		sv[i] = double(i - I) / (n - I);
	}
	for (i = n + 1; i < n + I + 1; i++){
		sv[i] = 1.0;
	}
	s = BSS(k, su, I, sv, d, m, u, n, v);
	delete[]su;
	delete[]sv;
	return s;
}
void Bspline::RiBSL(double *dx, double *dy, int n, double u, double *x, double *y)
{
	int i;
	double *su = new double[n + 3];
	double *I = new double[n];
	double L = 0.0, I1;
	I[0] = 0.0;
	for (i = 1; i < n; i++){
		I[i] = sqrt(pow(dx[i] - dx[i - 1], 2) + pow(dy[i] - dy[i - 1], 2));
		L = L + I[i];
	}
	su[0] = 0.0;
	su[1] = 0.0;
	su[2] = 0.0;
	su[3] = 0.0;
	I1 = I[1];
	for (i = 4; i < n - 1; i++){
		I1 = I1 + I[i - 2];
		su[i] = I1 / L;
	}
	su[n] = 1.0;
	su[n + 1] = 1.0;
	su[n + 2] = 1.0;
	su[n - 1] = 1.0;
	*x = BSL(3, su, dx, n, u);
	*y = BSL(3, su, dy, n, u);
	delete[]su;
	delete[]I;
}
Matrix Bspline::RiBSL(Matrix &mx, int output)
{
	int i;
	int nrow = mx.Row();
	double *su = new double[nrow + 3];
	double *I = new double[nrow];
	double *dx = new double[nrow];
	double *dy = new double[nrow];
	for (int ii = 0; ii < nrow; ii++){
		dx[ii] = mx[ii][0];
		dy[ii] = mx[ii][1];
	}
	double L = 0.0, I1;
	I[0] = 0.0;
	for (i = 1; i < nrow; i++){
		I[i] = sqrt(pow(mx[i][0] - mx[i - 1][0], 2) + pow(mx[i][1] - mx[i - 1][1], 2));
		L = L + I[i];
	}
	su[0] = 0.0;
	su[1] = 0.0;
	su[2] = 0.0;
	su[3] = 0.0;
	I1 = I[1];
	for (i = 4; i < nrow - 1; i++){
		I1 = I1 + I[i - 2];
		su[i] = I1 / L;
	}
	su[nrow] = 1.0;
	su[nrow + 1] = 1.0;
	su[nrow + 2] = 1.0;
	su[nrow - 1] = 1.0;
	Matrix mxoutput(output, 2);
	for (int j = 0; j < output; j++){
		double u = double(j)*(1.0 / (double(output) - 1.0));
		mxoutput[j][0] = this->BSL(3, su, dx, nrow, u);
		mxoutput[j][1] = this->BSL(3, su, dy, nrow, u);
	}
	delete[]su;
	delete[]I;
	delete[]dx;
	delete[]dy;
	return mxoutput;
}
double Bspline::ValueAt(double x)
{
	int i;
	int nrow = mx0.Row();
	double *su = new double[nrow + 3];
	double *I = new double[nrow];
	double *dx = new double[nrow];
	double *dy = new double[nrow];
	for (int ii = 0; ii < nrow; ii++){
		dx[ii] = mx0[ii][0];
		dy[ii] = mx0[ii][1];
	}
	double L = 0.0, I1;
	I[0] = 0.0;
	for (i = 1; i < nrow; i++){
		I[i] = sqrt(pow(mx0[i][0] - mx0[i - 1][0], 2) + pow(mx0[i][1] - mx0[i - 1][1], 2));
		L = L + I[i];
	}
	su[0] = 0.0;
	su[1] = 0.0;
	su[2] = 0.0;
	su[3] = 0.0;
	I1 = I[1];
	for (i = 4; i < nrow - 1; i++){
		I1 = I1 + I[i - 2];
		su[i] = I1 / L;
	}
	su[nrow] = 1.0;
	su[nrow + 1] = 1.0;
	su[nrow + 2] = 1.0;
	su[nrow - 1] = 1.0;

	if (mx0[nrow - 1][0] - mx0[0][0] == 0.0)
		return 0.0;
	else{
		double u = (x - mx0[0][0]) / (mx0[nrow - 1][0] - mx0[0][0]);
		return this->BSL(3, su, dx, nrow, u);
	}
	delete[]su;
	delete[]I;
	delete[]dx;
	delete[]dy;
}
Bspline::~Bspline()
{
	mx0.Empty();
}


// Bspline 成员函数
