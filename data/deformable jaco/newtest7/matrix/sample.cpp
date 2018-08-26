// Matrix Class Library ‚ðŽg‚Á‚½program

#include <stdio.h>
#include <math.h>
#include "matrix.h"
void mat_sample(void)
{     
      // int i,j;

}

void mat_sample2(void)
{
	MATRIX_D A = MatD31(1, 2, 3), B = MatD31(4, 5, 6);
	double C;
	C = A & B;
	printf("C = %f\n",C);
}
