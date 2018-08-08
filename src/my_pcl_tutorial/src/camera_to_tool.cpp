#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
 #include "pcl_ros/transforms.h"
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <fstream>


using namespace std;
using namespace Eigen;
using namespace pcl;

ros::Publisher pub;

//  void quat2Dcm(float * quaternion_in, Matrix4f& dcm,float a,float b,float c)
//   {/*quaternion quaternion_in=[x y z w] -----> dcm rotation 3*3 matrix */
//       dcm(0,0) = quaternion_in[3]*quaternion_in[3] + quaternion_in[0]*quaternion_in[0] - quaternion_in[1]*quaternion_in[1] - quaternion_in[2]*quaternion_in[2];
//       dcm(0,1) = 2*(quaternion_in[0]*quaternion_in[1] + quaternion_in[3]*quaternion_in[2]);
//       dcm(0,2) = 2*(quaternion_in[0]*quaternion_in[2] - quaternion_in[3]*quaternion_in[1]);
//       dcm(1,0) = 2*(quaternion_in[0]*quaternion_in[1] - quaternion_in[3]*quaternion_in[2]);
//       dcm(1,1) = quaternion_in[3]*quaternion_in[3] - quaternion_in[0]*quaternion_in[0] + quaternion_in[1]*quaternion_in[1] - quaternion_in[2]*quaternion_in[2];
//       dcm(1,2) = 2*(quaternion_in[1]*quaternion_in[2] + quaternion_in[3]*quaternion_in[0]);
//       dcm(2,0) = 2*(quaternion_in[0]*quaternion_in[2] + quaternion_in[3]*quaternion_in[1]);
//       dcm(2,1) = 2*(quaternion_in[1]*quaternion_in[2] - quaternion_in[3]*quaternion_in[0]);
//       dcm(2,2) = quaternion_in[3]*quaternion_in[3] - quaternion_in[0]*quaternion_in[0] - quaternion_in[1]*quaternion_in[1] + quaternion_in[2]*quaternion_in[2];
//       dcm(0,3) = a;
//       dcm(1,3) = b;
//       dcm(2,3) = c;
//   }

void changeCloud2Tool(const sensor_msgs::PointCloud2ConstPtr & cloud_msg){
  // pcl::PCLPointCloud2 blob;
  // pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  // pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
  pcl::PointCloud<PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZ>);

  //pcl::fromPCLPointCloud2 (blob, *cloud);
  pcl::fromROSMsg (*cloud_msg, *cloud);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  while (ros::ok()){
    
    try{
      listener.lookupTransform("/tool0_controller", "/camera",  
                               ros::Time(0), transform);
                               cout<<"x"<<transform.getOrigin().x()<<endl;
                               cout<<"y"<<transform.getOrigin().y()<<endl;
                               cout<<"z"<<transform.getOrigin().z()<<endl;
                               cout<<"1"<<transform.getOrigin().z()<<endl;
                               cout<<"2"<<transform.getOrigin().z()<<endl;
                               cout<<"3"<<transform.getOrigin().z()<<endl;
                               cout<<"4"<<transform.getOrigin().z()<<endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
 
  }
  
  // Matrix4f convert1;
  // quat2Dcm()
  // pcl_ros::transformPointCloud (*cloud, *cloud_filtered, transform);
  pcl_ros::transformPointCloud ("/tool0_controller",*cloud, *cloud_filtered, listener);
  pcl::PCLPointCloud2 cloud_2_filtered;
 //pcl::PointCloud<PCLPointCloud2>::Ptr cloud_2_filtered (new pcl::PointCloud<PCLPointCloud2>);

 pcl::toPCLPointCloud2 (* cloud_filtered, cloud_2_filtered);

// Convert to ROS data type

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_2_filtered, output);

  // Publish the data
  pub.publish (output);
}

// void
// Camera2Tool(const sensor_msgs::PointCloud2 & cloud_msg){
//   sensor_msgs::PointCloud2 output;
//   tf::TransformListener listener;
//   while (ros::ok()){
//     tf::StampedTransform transform;
//     try{
//       listener.lookupTransform("/tool0_controller", /*"/camera",*/"/tool0_controller",  
//                                ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//     }
//   }
//   pcl_ros::transformPointCloud ("/tool0_controller", cloud_msg, output,listener);
//   // Publish the data
//   pub.publish (output);
// }



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "camera_to_tool");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("voxel/grid", 1, changeCloud2Tool);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("tool/cloud", 1);

  // Spin
  ros::spin ();
}