
#include "adrobot_kinematics.h"
MATRIX_D RPY2RotMat(double alpha, double beta, double gamma)
{
	MATRIX_D rotMat=Zeros(3,3);

	rotMat=Rot(2,cos(gamma),sin(gamma))
			* Rot(1,cos(beta),sin(beta))
			* Rot(0,cos(alpha),sin(alpha));

	return rotMat;

	}

MATRIX_D RotMat2EulerAngle(MATRIX_D rotMat)
{
	MATRIX_D EulerAngle=Zeros(3,1);
	double alpha, beta,gamma, r11, r12, r13,r21, r22,r23,r31;
	r31=rotMat(3,1); r11=rotMat(1,1);
	r21=rotMat(2,1); r12=rotMat(1,2);
	r22=rotMat(2,2); r13=rotMat(1,3);
	r23=rotMat(2,3);

	/*beta = atan2(-r31,sqrt(r11*r11+r21*r21));

	if(abs(cos(beta))>0.0001)
	{ alpha=atan2(r21/cos(beta),r11/cos(beta));
	  gamma=atan2(r32/cos(beta),r33/cos(beta));
	}
	else
	{ beta=M_PI;
	  alpha=0;
	  gamma=atan2(r12,r22);
	}*/
	gamma=atan2(r21,r11);
	beta=atan2(-r31,r11*cos(gamma)+r21*sin(gamma));
	alpha=atan2(r13*sin(gamma)-r23*cos(gamma),-r12*sin(gamma)+r22*cos(gamma));
	EulerAngle=MatD31(alpha,beta,gamma);
	return EulerAngle;
}

MATRIX_D CalcB0(double alpha, double beta, double gamma)
{

	MATRIX_D B0 = MatD33 (cos(beta)*cos(gamma), -sin(gamma),0.0,
			              cos(beta)*sin(gamma), cos(gamma),0.0,
						  -sin(beta),0.0, 1.0);
	return B0;

}
MATRIX_D ur_kinematics(JOINTLINK* jnt, MATRIX_D &rx_h)
{  MATRIX_D jnt1_p=MatD31(0,0,UR_ARM_OFFSET_D1),
    jnt2_p=MatD31(0,0,0),  jnt3_p=MatD31(0,0,0), jnt4_p=MatD31(0,0,0),
    jnt5_p=MatD31(0,0,0),  hnd_p=MatD31(0,0,0);
    jnt2_p=jnt1_p;
   MATRIX_D AxisAngle=MatD31(0.0,0.0,0.0);
   MATRIX_D EulerAngle=MatD31(0.0,0.0,0.0);

   MATRIX_D rx_s  = (Rot(2, jnt->c[0], jnt->s[0])
		               * Rot(0, cos(M_PI/2), sin(M_PI/2))
		               * Rot(2, jnt->c[1], jnt->s[1]));
       jnt3_p = jnt2_p + rx_s * MatD31(-UR_ARM_OFFSET_A2,0,0);
   MATRIX_D rx_3 = rx_s * Rot(2, jnt->c[2],jnt->s[2]);
       jnt4_p = jnt3_p + rx_3 * MatD31(-UR_ARM_OFFSET_A3,0,UR_ARM_OFFSET_D4);
   MATRIX_D rx_4 = rx_3 * Rot(2, jnt->c[3],jnt->s[3]);
      jnt5_p = jnt4_p + rx_4 * MatD31(0,-UR_ARM_OFFSET_D5,0);
   MATRIX_D rx_5 = rx_4 *Rot(0,cos(M_PI/2),sin(M_PI/2))* Rot(2, jnt->c[4],jnt->s[4]);
    hnd_p = jnt5_p + rx_5 * MatD31(0,UR_ARM_OFFSET_D6,0);
    rx_h = rx_5 * Rot(0,cos(-M_PI/2),sin(-M_PI/2)) * Rot(2, jnt->c[5], jnt->s[5]);
    AxisAngle = RotMat2AxisAngle(rx_h);
    EulerAngle=RotMat2EulerAngle(rx_h);

    return ((hnd_p || EulerAngle));
}

MATRIX_D ur_jacobian(JOINTLINK*jnt, JACOBIAN *jcbn)
{
	int flag;
    MATRIX_D jnt1_p=MatD31(0.0, 0.0, UR_ARM_OFFSET_D1);
	MATRIX_D jnt1_axis=MatD31(0.0, 0.0, 1.0);

	MATRIX_D rx_s = Rot(2,jnt->c[0],jnt->s[0])
			* Rot(0,cos(M_PI/2),sin(M_PI/2))
			* Rot(2,jnt->c[1],jnt->s[1]);
	MATRIX_D jnt2_p = jnt1_p;
	MATRIX_D jnt2_axis= MatD31(rx_s(1,3),rx_s(2,3),rx_s(3,3));
	MATRIX_D jnt3_p = jnt2_p + rx_s * MatD31(-UR_ARM_OFFSET_A2,0.0,0.0);
	MATRIX_D rx_3 = rx_s * Rot(2, jnt->c[2], jnt->s[2]);
	MATRIX_D jnt3_axis = MatD31(rx_3(1,3),rx_3(2,3),rx_3(3,3));
	MATRIX_D jnt4_p = jnt3_p + rx_3 * MatD31(-UR_ARM_OFFSET_A3,0.0,UR_ARM_OFFSET_D4);
	MATRIX_D rx_4 = rx_3 * Rot(2, jnt->c[3],jnt->s[3]);

	MATRIX_D jnt4_axis=MatD31(rx_4(1,3),rx_4(2,3),rx_4(3,3));
    MATRIX_D jnt5_p = jnt4_p + rx_4 * MatD31(0,-UR_ARM_OFFSET_D5,0);
    MATRIX_D rx_5 = rx_4 * Rot(0,cos(M_PI/2),sin(M_PI/2))*Rot(2,jnt->c[4],jnt->s[4]);

	MATRIX_D jnt5_axis = MatD31(rx_5(1,3),rx_5(2,3),rx_5(3,3));
	MATRIX_D pos = jnt5_p + rx_5 * MatD31(0,UR_ARM_OFFSET_D6,0.0);
	MATRIX_D rotM = rx_5 * Rot(0,cos(-M_PI/2),sin(-M_PI/2))*Rot(2,jnt->c[5],jnt->s[5]);
	MATRIX_D jnt6_p = pos;
	MATRIX_D jnt6_axis = MatD31(rotM(1,3),rotM(2,3),rotM(3,3));

	(double *)jcbn->Jcb<<=
     (((jnt1_axis && (pos-jnt1_p)) | (jnt2_axis&&(pos-jnt2_p)) |
      (jnt3_axis && (pos-jnt3_p)) | (jnt4_axis&&(pos-jnt4_p)) |
	  (jnt5_axis && (pos-jnt5_p)) | (jnt6_axis&&(pos-jnt6_p))) ||
	   (jnt1_axis | jnt2_axis | jnt3_axis |
	   jnt4_axis | jnt5_axis | jnt6_axis));

	MATRIX_D jcb(6,6,(double *)jcbn->Jcb);
	MATRIX_D ijcb(6,6),tjcb(6,6);
	tjcb=jcb.t();
	(double*)jcbn->TrnsJcb<<=tjcb;
	flag=CalcInverse(jcb,ijcb);
	if(flag==0)
		(double*)jcbn->invJcb<<=ijcb;
		return jcb;


}
extern int CalcInverse(MATRIX_D &jcb, MATRIX_D &ijcb)
{
  MATRIX_D mat(6,6);
  mat=jcb;
  if(mat.snglr())
	 return (-1);
  else
  {
	  ijcb=mat.inverse();
  }
	return (0);
}

MATRIX_D AxisAnglePi(MATRIX_D BMat)
{
 double b11,b22,b33,b12,b13,b23;
 b11=BMat(1,1);b22=BMat(2,2);b33=BMat(3,3);
 b12=BMat(1,2);b13=BMat(1,3);b23=BMat(2,3);

 if((b12>0)&(b13>0)&(b23>0))
 {
   return(MatD31(sqrt(b11),sqrt(b22),sqrt(b33)));
 }else if(abs(b12*b13*b23)>0.0001)
   { // no one is zero
       if(b12>0)
       {
    	return(MatD31(sqrt(b11),sqrt(b22),-sqrt(b33)));
       }
       else if(b13>0)
       {
    	return(MatD31(sqrt(b11),-sqrt(b22),sqrt(b33)));
       }
       else
       {
    	return(MatD31(-sqrt(b11),sqrt(b22),sqrt(b33)));
       }
   }else if((abs(b12)>0.0001)||(abs(b13)>0.0001)||(abs(b23)>0.0001))
        {// one ni is zero
	           if(abs(b12)>0.0001)
	              { if(b12>0)
	    	        return(MatD31(sqrt(b11),sqrt(b22),0.0));
	                else
	                return(MatD31(sqrt(b11),-sqrt(b22),0.0));
	              }
	          if(abs(b13)>0.0001)
	     	       { if(b13>0)
	     	    	 return(MatD31(sqrt(b11),0.0,sqrt(b33)));
	     	         else
	     	         return(MatD31(sqrt(b11),0.0,-sqrt(b33)));
	     	       }
	          if(abs(b23)>0.0001)
	     	       { if(b23>0)
	     	    	 return(MatD31(0.0,sqrt(b22),sqrt(b33)));
	     	         else
	     	         return(MatD31(0.0,sqrt(b22),-sqrt(b33)));
	     	       }
   }else
   {   // two n_i are zero
	   if(abs(b11)>0.0001)
		   {return (MatD31(sqrt(b11),0.0,0.0));}
	   else if(abs(b22)>0.0001)
	      {return (MatD31(0.0,sqrt(b22),0.0));}
	   else
	      {return (MatD31(0.0,0.0,sqrt(b33)));}
   }
}

MATRIX_D RotMat2AxisAngle(MATRIX_D rotMat)
{
  double angle, cos_angle;

  MATRIX_D temp=Zeros(3,3);
  MATRIX_D vN = MatD31(0,0,0);

  angle=acos(1/2.0*(rotMat.tr()-1));
  cos_angle = cos(angle);

  if(cos_angle<-0.999)
  {// condition that angle = pi
   MATRIX_D BMat = 0.5*(rotMat+Eye(3));

   return (AxisAnglePi(BMat));
          }
  else if (cos_angle > 0.999 )
  { // condition that angle = 0, axis is undetermined.
	         return (Zeros(3,1));
  }
  else {
	temp=1/(2*sin(angle))*(rotMat-rotMat.t());
	vN=MatD31(temp(3,2),temp(1,3),temp(2,1));
	return(angle * vN);
  }
}
MATRIX_D ur_InverseKinematics(MATRIX_D hand_p, MATRIX_D rotMat){

}


