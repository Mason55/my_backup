#include <stdio.h>
#include <math.h>

#include "matrix.h"

MATRIX_D Rot(int xyz, double cc, double ss)
{
	double aij[3][3];	
	memset(aij, 0, sizeof(double)*9);

	switch(xyz){
	case 0: // rotation around x axis 
		aij[0][0] =  1.0;		
		aij[1][1] =  cc;
		aij[1][2] = -ss;
		aij[2][1] =  ss;
		aij[2][2] = cc;
		break;
	case 1: // rotation around y axis 
		aij[0][0] =  cc;
		aij[0][2] =  ss;
		aij[1][1] =  1.0;		
		aij[2][0] = -ss;
		aij[2][2] =  cc;
		break;
	case 2: // rotation around z axis 
		aij[0][0] =  cc;
		aij[0][1] = -ss;
		aij[1][0] =  ss;
		aij[1][1] =  cc;
		aij[2][2] =  1.0;		
	}
	MATRIX_D m(3,3,(double *)aij);

	return m;	
}

MATRIX_D RotD(int xyz, double cc, double ss)
{
	double aij[3][3];
	memset(aij, 0, sizeof(double)*9);

	switch(xyz){
	case 0: // rotation around x axis 
		aij[1][1] = -ss;
		aij[1][2] = -cc;
		aij[2][1] =  cc;
		aij[2][2] = -ss;
		break;
	case 1: // rotation around y axis 
		aij[0][0] = -ss;
		aij[0][2] =  cc;
		aij[2][0] = -cc;
		aij[2][2] = -ss;
		break;
	case 2: // rotation around z axis 
		aij[0][0] = -ss;
		aij[0][1] = -cc;
		aij[1][0] =  cc;
		aij[1][1] = -ss;
	}
	MATRIX_D m(3,3,(double *)aij);

	return m;	
}

MATRIX_D Skew(double x, double y, double z)/* —vŠm”F */
{
	double aij[3][3];
	memset(aij, 0, sizeof(double)*9);

	aij[0][1] = -z; 	
	aij[0][2] =  y;	
	aij[1][0] =  z; 	
	aij[1][2] = -x;	
	aij[2][0] = -y; 	
	aij[2][1] =  x;
	MATRIX_D m(3,3,(double *)aij);

	return m;	
}

MATRIX_D MatD31(double x, double y, double z)
{
	double aij[3];

	aij[0] = x; 	
	aij[1] = y;	
	aij[2] = z; 	
	MATRIX_D m(3,1,(double *)aij);

	return m;
}

MATRIX_D MatD33(double x11, double x12, double x13,
				double x21, double x22, double x23,
				double x31, double x32, double x33)
{
	double aij[9];

	aij[0] = x11; 	aij[1] = x12;	aij[2] = x13; 	
	aij[3] = x21; 	aij[4] = x22;	aij[5] = x23; 	
	aij[6] = x31; 	aij[7] = x32;	aij[8] = x33; 	
	MATRIX_D m(3,3,(double *)aij);

	return m;	
}

MATRIX_D MatD61(double x0, double x1, double x2, double x3, double x4, double x5)
{
	double aij[6];

	aij[0] = x0;
	aij[1] = x1;
	aij[2] = x2;
	aij[3] = x3;
	aij[4] = x4;
	aij[5] = x5;
	MATRIX_D m(6,1,(double *)aij);

	return m;	
}

MATRIX_D Zeros(int ii, int jj)
{
	MATRIX_D m(ii, jj);

	return m;
}

MATRIX_D Ones(int ii, int jj)
{
	MATRIX_D m(ii, jj);

	for(int i = 1; i <= ii; i++)
		for(int j = 1; j <= jj; j++)
			m(i,j) = 1.0;

	return m;
}

MATRIX_D Eye(int ii)
{
	MATRIX_D m(ii, ii);
	
    for(int i = 1; i <= ii; i++)
        m(i,i) = 1.0;   
	
	return m;
}

MATRIX_D logMatrix(MATRIX_D &r)
{
// This function can only be applied for 3x3 matrix
	MATRIX_D result(3,3);
	double trace,theta,temp;
	trace=r.tr();
	theta=acos((trace-1)/2); // notice the treatment when trace==-1 
    temp=theta/sin(theta)/2.0;
	result=temp*(r-r.t());

	return result;
}

MATRIX_D eMatrix(MATRIX_D &theta)
{
// this function can only be applied for 3x3 matrix
	double theta_norm,temp;
	int i,j;
	MATRIX_D part1(3,3),part2(3,3),result(3,3);
    temp=0.0;
	for(i=1;i<4;i++)
		for(j=1;j<4;j++)
			temp+=theta(i,j)*theta(i,j);	

	theta_norm=sqrt(temp);
	if(theta_norm>0.000001)
	{
	temp=sin(theta_norm)/theta_norm;
	part1=temp*theta;
	temp=(1-cos(theta_norm))/(theta_norm*theta_norm);
	part2=temp*(theta*theta);
	result=Eye(3);
	result=result+part1+part2;}
	else
		result=Eye(3);	
	return result;
}
MATRIX_D Triu(MATRIX_D &m, int n)
{ 
 // this function is to produce the 
 //  upper triagular part of matrix
 //  m is the input matrix, n is the dimension
  int i,j;
  MATRIX_D result(n,n);
    result=Zeros(n,n);
  for(i=1; i<=n ; i++)
   for(j=i; j<=n; j++)
     result(i,j)=m(i,j);
     
   return result;  
}
