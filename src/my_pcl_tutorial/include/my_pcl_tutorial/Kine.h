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
public:
	void printHello();
	void dcm2Quat(float * quaternion_out, Eigen::Matrix4f dcm);
	void quat2Dcm(float * quaternion_in, Matrix4f& dcm2);
};