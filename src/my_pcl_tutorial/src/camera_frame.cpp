#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <iostream>
#include "Kine.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){

  ros::init(argc, argv, "camera_frame");
  ros::NodeHandle node;

  Kine k_cal;

  Matrix4f car2base;
            car2base<<-0.0213  , -0.8264 ,  -0.5626  ,  1.1210,
                      -0.9997  ,  0.0264 ,  -0.0009  , -0.1194,
                       0.0156  ,  0.5624 ,  -0.8267  ,  1.0983,
                      -0.0000  , -0.0000 ,   0.0000  ,  1.0000;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  float q[4] ={0,0,0,0};
  
//   dcm2Quat(q, car2base);
    k_cal.dcm2Quat(q, car2base);

  cout<<q[0]<<endl<<q[1]<<endl<<q[2]<<endl<<q[3]<<endl;

  ros::Rate rate(30.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(1.1210, -0.1194, 1.0983) );
    transform.setRotation( tf::Quaternion(q[0], q[1], q[2], q[3]) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "camera"));
    // br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "base", "camera"));
    
    rate.sleep();
  }
  return 0;
};