#pragma once
#include<iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Kine
{
private:
	Matrix4f dcm;
	Matrix3f dcm2;
	float * quaternion_in,quaternion_out;
	Vector3f euler_ZYX;
public:
	void printHello();
	void dcm2Quat(float * quaternion_out, Eigen::Matrix4f dcm);
    void quat2Dcm(float * quaternion_in, Matrix4f& dcm2);
    void eul2Quat(float * quaternion_out, Vector3f euler_ZYX);
    void quat2Eul(float * quaternation_in, Vector3f &euler_ZYX);
    void eul2Rotm(Vector3f euler_ZYX, Matrix3f &rotm);
    void rotm2Eul(Vector3f& euler_ZYX, Matrix3f rotm);
};