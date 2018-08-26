/*
 * servo.h
 *
 *  Created on: Apr 20, 2018
 *      Author: jch
 */
#ifndef ADROBOT_CONTROL_H
#define ADROBOT_CONTROL_H
#include "../common.h"
#include "../adrobot_io/ur_driver.h"
#include "../adrobot_io/robot_state_RT.h"
#include "../adrobot_kinematics/adrobot_kinematics.h"
/*Eigen header*/
#include <eigen3/Eigen/Dense>
/*Eigen header*/
using namespace Eigen;

extern void servo_function(UrDriver* ur);
extern void CalcJntRefPath(double curtime, PATH *path, THETA *theta, THETA *dtheta);
extern void CalcPosRefPath(double curtime,PATH *path,POS*pos);
extern float ComputeDistance(float *coordinate,int a,float *P,int b);
extern MatrixXf ComputeRadiusVector(float *coordinate,int a,float *P,int b);
extern MatrixXf ComputeGripperToObjectJacobian(float GripperToPoint,MatrixXf &Radius);
extern MatrixXf ComputeAttractiveForce(float *P,int a);
extern MatrixXf ComputeRepulsiveForce(float *P,int a);
extern float ComputeAttractiveForceNorm(MatrixXf &AttractiveForce);
extern float ComputeRepulsiveForceNorm(MatrixXf &RepulsiveForce);
extern float ComputeXyDistance(float *P,int a);
extern float ComputeUvDistance(float *P,int a);
#endif
