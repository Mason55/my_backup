
#ifndef ADROBOT_KINEMATICS_H
#define ADROBOT_KINEMATICS_H
#include "../matrix/matrix.h"
#define UR_ARM_OFFSET_D1 0.089459
#define UR_ARM_OFFSET_A2 0.42500
#define UR_ARM_OFFSET_A3 0.39225
#define UR_ARM_OFFSET_D4 0.10915
#define UR_ARM_OFFSET_D5 0.09465
#define UR_ARM_OFFSET_D6 0.1863//需要在tcp的基础上考虑夹具的长度0.104
//#define UR_ARM_OFFSET_D6 0.0823

//初始位置关节角的余弦正弦值
typedef struct
{
  double c[6];
  double s[6];
}JOINTLINK;
typedef struct
{
double Jcb[6][6];
double TrnsJcb[6][6];
double invJcb[6][6];
} JACOBIAN;

extern MATRIX_D ur_kinematics(JOINTLINK* jnk, MATRIX_D &p);
extern MATRIX_D RotMat2AxisAngle(MATRIX_D rotMat);
extern MATRIX_D ur_InverseKinematics(MATRIX_D hand_p, MATRIX_D rotMat);
extern MATRIX_D RPY2RotMat(double alpha, double beta, double gamma);
extern MATRIX_D RotMat2EulerAngle(MATRIX_D rotMat);
extern MATRIX_D CalcB0(double alpha, double beta, double gamma);
extern MATRIX_D ur_jacobian(JOINTLINK*jnk,JACOBIAN *jcbn);
extern int CalcInverse(MATRIX_D &jcb, MATRIX_D &ijcb);
#endif
