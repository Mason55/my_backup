#include "adrobot_control.h"
#include "../adrobot_etc/adrobot_etc.h"
/*Eigen header*/
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
/*Eigen header*/
#define  JNT_PATH_SIN   0
#define  JNT_PATH_5JI   1
#define  JNT_PATH_3JI   2
#define  JNT_PATH_1JI   3
#define  JNT_PATH_STEP  4
#define  JNT_PATH_SIN_FF   100
#define  JNT_PATH_5JI_FF   200
#define  JNT_PATH_3JI_FF   300

using namespace Eigen;
using namespace  std;


void CalcJntRefPath(double curtime, PATH *path, THETA *theta, THETA *dtheta)
{
    int i;
    double *orig, *goal, *ref, *ref_v;

	orig = path->Orig;
	goal = path->Goal;
	ref  = (double *)theta;
	ref_v = (double *)dtheta;
    for(i = 0; i < 6; i++){
        switch(path->Mode){
		case JNT_PATH_5JI:
		case JNT_PATH_5JI_FF:
			ref[i] = Calc5JiTraje(orig[i], goal[i], path->Freq, curtime);
			break;
		case JNT_PATH_3JI:
		case JNT_PATH_3JI_FF:
			ref[i] = Calc3JiTraje(orig[i], goal[i], path->Freq, curtime);
			break;
		case JNT_PATH_1JI:
			ref[i] = Calc1JiTraje(orig[i], goal[i], path->Freq, curtime);
			break;
		case JNT_PATH_STEP:
			ref[i] = CalcStepTraje(orig[i], goal[i], path->Freq, curtime);
			break;
		default:
			ref[i] = CalcSinTraje(orig[i], goal[i], path->Freq, curtime);
        }
        switch(path->Mode){
		case JNT_PATH_3JI_FF:
			ref_v[i] = Calc3JiTrajeVelo(orig[i],goal[i], path->Freq, curtime);
			break;
		case JNT_PATH_5JI_FF:
			ref_v[i] = Calc5JiTrajeVelo(orig[i], goal[i], path->Freq, curtime);
			break;
		case JNT_PATH_SIN_FF:
			ref_v[i] = CalcSinTrajeVelo(orig[i],goal[i], path->Freq, curtime);
			break;
		default:
			ref_v[i] = 0.;
        }
    }
}

//Path planning
void CalcPosRefPath(double curtime,PATH *path,POS *refpos){
    int i;
    double *orig, *goal, *ref;

	orig = path->Orig;
	goal = path->Goal;
	ref=(double*)refpos;
	for(i=0;i<6;i++){
		switch(path->Mode){
				case JNT_PATH_5JI:
				case JNT_PATH_5JI_FF:
					ref[i] = Calc5JiTraje(orig[i], goal[i], path->Freq, curtime);
					break;
				case JNT_PATH_3JI:
				case JNT_PATH_3JI_FF:
					ref[i] = Calc3JiTraje(orig[i], goal[i], path->Freq, curtime);
					break;
				case JNT_PATH_1JI:
					ref[i] = Calc1JiTraje(orig[i], goal[i], path->Freq, curtime);
					break;
				case JNT_PATH_STEP:
					ref[i] = CalcStepTraje(orig[i], goal[i], path->Freq, curtime);
					break;
				default:
					ref[i] = CalcSinTraje(orig[i], goal[i], path->Freq, curtime);
		}
	}
}
/*Attractiveforce*/
MatrixXf ComputeAttractiveForce(float *P,int a)
{
    float x_g=0.4317,y_g=-0.1117;//x_g,y_g机器人终止位置(目标点),x,y机器人当前位置,x_g=0.455,y_g=-0.08
    float x,y;
    float k_a=2;//相应的正比例增益系数
    float d_att;
    MatrixXf F_att(3,1);
    MatrixXf cur_pos(3,1);
    cur_pos(0,0)=x=P[0];cur_pos(1,0)=y=P[1];cur_pos(2,0)=0;
    MatrixXf end_pos(3,1);
    end_pos(0,0)=x_g;end_pos(1,0)=y_g;end_pos(2,0)=0;
    MatrixXf delta_pos(3,1);
    delta_pos = end_pos - cur_pos;
    F_att = k_a*delta_pos;
    MatrixXf F(6,1);
    F(0,0)=F_att(0,0);F(1,0)=F_att(1,0);F(2,0)=F_att(2,0);F(3,0)=0;F(4,0)=0;F(5,0)=0;
    return F;
}
/*Attractiveforce*/

/*Repulsiveforce*/
//MatrixXf ComputeRepulsiveForce(float *P,int a)
//{
//    float u_g=938.622,v_g=585.32;
//    float u_0,v_0,r;
//    float d_0=30;//障碍物可对机器人运动产生影响的最大距离
//    float k_r=2;//相应的正比例增益系数
//    MatrixXf F_rep;
//    float d_rep;
//    MatrixXf obstacle_pos(3,1);
//    obstacle_pos(0,0)=u_g;obstacle_pos(1,0)=v_g;obstacle_pos(2,0)=0;
//    MatrixXf point_pos(3,1);
//    point_pos(0,0)=u_0=P[3];point_pos(1,0)=v_0=P[4];point_pos(2,0)=0;
//    d_rep= sqrt((u_0-u_g)*(u_0-u_g)+(v_0-v_g)*(v_0-v_g));//定义为图像平面内障碍物到电缆中点的距离
//    r = (k_r/d_rep*d_rep)*(1/d_rep-1/d_0);
//    MatrixXf delta_pos(3,1);
//    delta_pos = obstacle_pos - point_pos;
//    F_rep = r/d_rep*delta_pos;
//    return F_rep;
//}
/*Repulsiveforce*/

/*CalcForce*/
float ComputeAttractiveForceNorm(MatrixXf &AttractiveForce)
{
    float F,a,b;
    MatrixXf F_att(6,1);
    F_att=AttractiveForce;
    a=F_att(0,0);b=F_att(1,0);
    F=sqrt(a*a+b*b);
    return F;

}
//float ComputeRepulsiveForceNorm(MatrixXf &RepulsiveForce)
//{
//    float F,a,b;
//    MatrixXf F_rep(3,1);
//    F_rep=RepulsiveForce;
//    a=F_rep(0,0);b=F_rep(1,0);
//    F=sqrt(a*a+b*b);
//    return F;
//}
/*CalcForce*/

/*CalcDistance*/
float ComputeXyDistance(float *P,int a)
{
    float d_att;
    float x=P[0],y=P[1];
    float x_g=0.4317,y_g=-0.1117;
    d_att= sqrt((x-x_g)*(x-x_g)+(y-y_g)*(y-y_g));//目标点于机器人末端执行器当前位置的距离
    return d_att;
}
//float ComputeUvDistance(float *P,int a)
//{
//    float d_rep;
//    float u_0=P[3],v_0=P[4];
//    float u_g=0.45857,v_g=-0.09160;
//    d_rep= sqrt((u_0-u_g)*(u_0-u_g)+(v_0-v_g)*(v_0-v_g));
//    return d_rep;
//}
/*CalcDistance*/
/*deformable_jacobi*/
//float ComputeDistance(float *coordinate,int a,float *P,int b)
//{
//    float x,y,z;
//    float d;
//    x=P[0];
//    y=P[1];
//    MatrixXf m(3,1);
//    for(int i=0;i<3;i++)
//    {
//        m(i,0)=coordinate[i];
//    }
//    d=sqrt((m(0,0)-x)*(m(0,0)-x)+(m(1,0)-y)*(m(1,0)-y));/*计算末端tcp到标记点的距离*/
//    return d;
//}

//MatrixXf ComputeRadiusVector(float *coordinate,int a,float *P,int b)
//{
//    float x,y,z;/*r=点在机器人坐标系下的坐标-夹持点在机器人坐标系下的坐标*/
//    float d[3];
//    MatrixXf N(3,1);
//    x=P[0];
//    y=P[1];
//    z=P[2];/*有改动(1)*/
//    MatrixXf PointCoordinate(3,1);
//    MatrixXf Radius(3,1);
//    for(int i=0;i<3;i++)
//    {
//        PointCoordinate(i,0)=coordinate[i];
//    }
//    Radius(0,0)=PointCoordinate(0,0)-x;
//    Radius(1,0)=PointCoordinate(1,0)-y;
//    Radius(2,0)=PointCoordinate(2,0)-z;

//    N(0,0)=Radius(0,0);
//    N(1,0)=Radius(1,0);
//    N(2,0)=Radius(2,0);

//    return N;
//}


MatrixXf ComputeGripperToObjectJacobian(float GripperToPoint,MatrixXf &Radius)
{
    int num_grippers = 1;
    int num_nodes = 1;
    int num_cols = num_grippers * 6;
    int num_rows = num_nodes * 3;
    MatrixXf J(3,6);
    float MinimumDistance;/*夹持器到各个点的最短距离*/
    MinimumDistance=GripperToPoint;
    MatrixXf RadiusVetor(3,1);
    RadiusVetor=Radius;

    // Get all the data we need for a given gripper

//    for (int node_ind = 0; node_ind < num_nodes; node_ind++)
//    {
        int gripper_ind=0;
        float translation_deformability=25;
        float rotation_deformability=25;
        Matrix3f J_trans;
        J_trans<< 1,0,0,
                  0,1,0,
                  0,0,1;
        const float dist_to_gripper =MinimumDistance;
        J.block<3, 3>(0, 0) =exp(-translation_deformability * dist_to_gripper) * J_trans;
        Matrix3f J_rot;


//       b1= RadiusVetor(node_ind*3,0);     [  0, b3,-b2
//       b2= RadiusVetor(node_ind*3+1,0);     -b3, 0, b1
//       b3= RadiusVetor(node_ind*3+2,0);      b2,-b1,0 ]

        J_rot(0,0)=0;J_rot(0,1)=RadiusVetor(2,0);J_rot(0,2)=-RadiusVetor(1,0);
        J_rot(1,0)=-RadiusVetor(2,0);J_rot(1,1)=0;J_rot(1,2)=RadiusVetor(0,0);
        J_rot(2,0)=RadiusVetor(1,0);J_rot(2,1)=-RadiusVetor(0,0);J_rot(2,2)=0;
//        J_rot(0,0)=0;J_rot(0,1)=0;J_rot(0,2)=0;
//        J_rot(1,0)=0;J_rot(1,1)=0;J_rot(1,2)=0;
//        J_rot(2,0)=0;J_rot(2,1)=0;J_rot(2,2)=0;

        J.block<3, 3>(0, 3) = exp(-rotation_deformability * dist_to_gripper) * J_rot;

        //block（p, q）可理解为一个p行q列的子矩阵，该定义表示从原矩阵中第(i,j)开始，获取一个p行q列的子矩阵，返回该子矩阵组成的临时矩阵对象，原矩阵的元素不变
//    }
        return J;
}
/*deformable_jacobi*/


