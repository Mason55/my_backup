#include "Kine.h"


void Kine::printHello()
{
    std::cout<<"------Hello------"<<std::endl;
}

void Kine::dcm2Quat(float * quaternion_out, Matrix4f dcm)
{/*dcm rotation 3*3 matrix ----> quaternion quaternion_out=[x y z w] */
   float tr, sqtrp1, sqdip1, q[4], d[3];

   // 计算矩阵轨迹
   tr = dcm(0,0) + dcm(1,1) + dcm(2,2);

   // 检查矩阵轨迹是正还是负
   if (tr > 0){
       sqtrp1 = sqrt( tr + 1.0 );

       quaternion_out[3] = 0.5*sqtrp1;
       quaternion_out[0] = (dcm(1,2) - dcm(2,1))/(2.0*sqtrp1);
       quaternion_out[1] = (dcm(2,0) - dcm(0,2))/(2.0*sqtrp1);
       quaternion_out[2] = (dcm(0,1) - dcm(1,0))/(2.0*sqtrp1);
   }
    else{

        d[0] = dcm(0,0);
        d[1] = dcm(1,1);
        d[2] = dcm(2,2);
        if ((d[1] > d[0]) && (d[1] > d[2])){
        // max value at dcm(2,2,i)
        sqdip1 = sqrt(d[1] - d[0] - d[2] + 1.0 );

        quaternion_out[1] = 0.5*sqdip1;


            if ( sqdip1 != 0 )
            { sqdip1 = 0.5/sqdip1;}

        quaternion_out[3] = (dcm(2,0) - dcm(0,2))*sqdip1;
        quaternion_out[0] = (dcm(0,1) + dcm(1,0))*sqdip1;
        quaternion_out[2] = (dcm(1,2) + dcm(2,1))*sqdip1;
        }
        else if (d[2] > d[0]){
        //max value at dcm(3,3,i)
        sqdip1 = sqrt(d[2] - d[0] - d[1] + 1.0 );

        quaternion_out[2] = 0.5*sqdip1;

        if ( sqdip1 != 0 )
        sqdip1 = 0.5/sqdip1;


        quaternion_out[3] = (dcm(0,1) - dcm(1,0))*sqdip1;
        quaternion_out[0] = (dcm(2,0) + dcm(0,2))*sqdip1;
        quaternion_out[1] = (dcm(1,2) + dcm(2,1))*sqdip1;
        }
        else{
        //max value at dcm(1,1,i)
        sqdip1 = sqrt(d[0] - d[1] - d[2] + 1.0 );

        quaternion_out[0] = 0.5*sqdip1;

        if ( sqdip1 != 0 )
        sqdip1 = 0.5/sqdip1;

        quaternion_out[3] = (dcm(1,2) - dcm(2,1))*sqdip1;
        quaternion_out[1] = (dcm(0,1) + dcm(1,0))*sqdip1;
        quaternion_out[2] = (dcm(2,0) + dcm(0,2))*sqdip1;
        }
    }
}

void Kine::quat2Dcm(float * quaternion_in, Matrix4f& dcm)
{/*quaternion quaternion_in=[x y z w] -----> dcm rotation 3*3 matrix */
      dcm(0,0) = quaternion_in[6]*quaternion_in[6] + quaternion_in[3]*quaternion_in[3] - quaternion_in[4]*quaternion_in[4] - quaternion_in[5]*quaternion_in[5];
      dcm(0,1) = 2*(quaternion_in[3]*quaternion_in[4] + quaternion_in[6]*quaternion_in[5]);
      dcm(0,2) = 2*(quaternion_in[3]*quaternion_in[5] - quaternion_in[6]*quaternion_in[4]);
      dcm(1,0) = 2*(quaternion_in[3]*quaternion_in[4] - quaternion_in[6]*quaternion_in[5]);
      dcm(1,1) = quaternion_in[6]*quaternion_in[6] - quaternion_in[3]*quaternion_in[3] + quaternion_in[4]*quaternion_in[4] - quaternion_in[5]*quaternion_in[5];
      dcm(1,2) = 2*(quaternion_in[4]*quaternion_in[5] + quaternion_in[6]*quaternion_in[3]);
      dcm(2,0) = 2*(quaternion_in[3]*quaternion_in[5] + quaternion_in[6]*quaternion_in[4]);
      dcm(2,1) = 2*(quaternion_in[4]*quaternion_in[5] - quaternion_in[6]*quaternion_in[3]);
      dcm(2,2) = quaternion_in[6]*quaternion_in[6] - quaternion_in[3]*quaternion_in[3] - quaternion_in[4]*quaternion_in[4] + quaternion_in[5]*quaternion_in[5];
      dcm(0,3) = quaternion_in[0];
      dcm(1,3) = quaternion_in[1];
      dcm(2,3) = quaternion_in[2];
      dcm(3,3) = 1.0;
}

void Kine::eul2Quat(float * quaternion_out, Vector3f euler_ZYX)
{
    Vector3f c = Vector3f(cos(euler_ZYX(0)/2),cos(euler_ZYX(1)/2),cos(euler_ZYX(2)/2));
    Vector3f s = Vector3f(sin(euler_ZYX(0)/2),sin(euler_ZYX(1)/2),sin(euler_ZYX(2)/2));

    quaternion_out[0] = c(0)*c(1)*c(2)+s(0)*s(1)*s(2);
    quaternion_out[1] = c(0)*c(1)*s(2)-s(0)*s(1)*c(2);
    quaternion_out[2] = c(0)*s(1)*c(2)+s(0)*c(1)*s(2);
    quaternion_out[3] = s(0)*c(1)*c(2)-c(0)*s(1)*s(2);
}

void Kine::quat2Eul(float * quaternation_in, Vector3f &euler_ZYX)
{
    float aSinInput = -2*(quaternation_in[1]*quaternation_in[3] - quaternation_in[0]*quaternation_in[2]);
    if(aSinInput>1)
        aSinInput = 1;
    euler_ZYX(0) = atan2(2*(quaternation_in[1]*quaternation_in[2]+quaternation_in[0]*quaternation_in[3]),
            quaternation_in[0]*quaternation_in[0]+quaternation_in[1]*quaternation_in[1] - quaternation_in[2]*quaternation_in[2] - quaternation_in[3]*quaternation_in[3]);
    euler_ZYX(1) = asin(aSinInput);
    euler_ZYX(2) = atan2(2*(quaternation_in[2]*quaternation_in[3]+quaternation_in[0]*quaternation_in[1]),
            quaternation_in[0]*quaternation_in[0] - quaternation_in[1]*quaternation_in[1] - quaternation_in[2]*quaternation_in[2] + quaternation_in[3]*quaternation_in[3]);
}

void Kine::eul2Rotm(Vector3f euler_ZYX, Matrix3f& rotm)
{
    Vector3f c = Vector3f(cos(euler_ZYX(0)), cos(euler_ZYX(1)), cos(euler_ZYX(2)));
    Vector3f s = Vector3f(sin(euler_ZYX(0)), sin(euler_ZYX(1)), sin(euler_ZYX(2)));

    rotm(0,0) = c(1)*c(0);
    rotm(0,1) = s(2)*s(1)*c(0) - c(2)*s(0);
    rotm(0,2) = c(2)*s(1)*c(0) + s(2)*s(0);
    rotm(1,0) = c(1)*s(0);
    rotm(1,1) = s(2)*s(1)*s(0) + c(2)*c(0);
    rotm(1,2) = c(2)*s(1)*s(0) - s(2)*c(0);
    rotm(2,0) = -s(1);
    rotm(2,1) = s(2)*c(1);
    rotm(2,2) = c(2)*c(1);
}

void Kine::rotm2Eul(Vector3f& euler_ZYX, Matrix3f rotm)
{
    float sy = sqrt(rotm(0,0)*rotm(0,0) + rotm(1,0)*rotm(1,0));

    bool singular = sy < 1e-6;

    if(!singular)
    {
        euler_ZYX(2) = atan2(rotm(2,1), rotm(2,2));
        euler_ZYX(1) = atan2(-rotm(2,0), sy);
        euler_ZYX(0) = atan2(rotm(1,0), rotm(0,0));
    }
    else
    {
        euler_ZYX(2) = atan2(-rotm(1,2), rotm(1,1));
        euler_ZYX(1) = atan2(-rotm(2,0), sy);
        euler_ZYX(0) = 0;
    }
}

