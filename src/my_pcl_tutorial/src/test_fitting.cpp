/*
获取点云的第一帧做一个初始化
做前一帧PCA和后一帧PCA之间的坐标变换，将结果应用于第一帧的点云
将第一帧的点云ＰＣＡ初始化成ＮＵＲＢＳ
利用fit将新帧的点云同初始化的ＮＵＲＢＳ之间做匹配
*/
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <vector>
#include <Eigen/Dense>
/*下面是共享内存需要用到的头文件*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
/*ROS自定义的消息类型*/
#include "point_msgs/Point.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

unsigned order (3);
unsigned refinement (2);
unsigned iterations (10);

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const int BUF_SIZE=1024;
const double PI = 3.141592653;
ros::Publisher pub;
// ON_NurbsSurface nurbs (3,false,order,order,order,order);
    
void PointCloud2Vector3d (pcl::PointCloud<PointT>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
void MeshVertices2Vector3d (std::vector<pcl::Vertices>  mesh_vertices, pcl::on_nurbs::vector_vec3d &data);
void print4x4Matrix (const Eigen::Matrix4f & matrix);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  point_msgs::Point output;
  PointCloudT::Ptr cloud (new PointCloudT);//不断更新的初始化的点云
  pcl::fromROSMsg (*cloud_msg, *cloud);
  printf ("  %lu points in data set\n", cloud->size ());
  pcl::on_nurbs::NurbsDataSurface data;//用于fit的过程
  PointCloud2Vector3d (cloud, data.interior);

  printf ("  surface fitting ...\n");

  // fit B-spline surface
  pcl::on_nurbs::FittingSurfaceTDM::ParameterTDM params;

  // initialize
  
  ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
  pcl::on_nurbs::FittingSurfaceTDM fit (&data, nurbs);
  // sleep(1);
  // surface fitting with final refinement level
  for (unsigned i = 0; i < iterations; i++)
  {
    printf ("  3 ...\n");
    fit.assemble (params);
    printf ("  4 ...\n");
    fit.solve (1.2);
    printf ("  5 ...\n");
  }


      double u=0.0;
      double v=0.0;
      int NUB_U =6;//那一共就有7*9=63个点在网格上
      int NUB_V =8;
      float array[600];
      ON_3dPoint mypoint;
      

      Matrix4f car2base; //这是把手眼标定的矩阵拿过来用了
      car2base<<-0.0213  , -0.8264 ,  -0.5626   , 1.1210,
        -0.9997  ,  0.0264 ,  -0.0009 ,  -0.1194,
        0.0156   , 0.5624  , -0.8267   , 1.0983,
        -0.0000 ,  -0.0000 ,   0.0000  ,  1.0000;
      Matrix4f parend2car;
      Matrix4f parend2base;

      int i =0 , j=0, k=0;
      for(i=0;i<=NUB_U;i++)//添加等于号是为了包含边界
      {
        for(j=0;j<=NUB_V;j++)
        {
          mypoint = fit.m_nurbs.PointAt(u,v);
          array[k] = mypoint.x;
          array[k+1] =mypoint.y;
          array[k+2] =mypoint.z;
          k += 3;
          v += (double)1/NUB_V;
          // std::cout<<v<<std::endl;
        }
        u += (double)1/NUB_U;
        v=0.0;
        // std::cout<<u<<std::endl;
      }
      std::cout<<array[0]<<std::endl;
      std::cout<<array[1]<<std::endl;
      std::cout<<array[2]<<std::endl;

      std::vector<float> array1(array,array+600);
      output.data = array1;

      ON_3dPoint par_end_car = fit.m_nurbs.PointAt(0.0,0.5);
      parend2car<<1,0,0,par_end_car.x,
                  0,1,0,par_end_car.y,
                  0,0,1,par_end_car.z,
                  0,0,0,1;
      parend2base=car2base*parend2car;
      
      pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "visual_bsfitting");
  ros::NodeHandle nh;


// ros::Subscriber sub2;
//   ros::NodeHandle nh2;

  ros::Subscriber sub = nh.subscribe ("/voxel/grid", 5, &cloud_cb);


  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<point_msgs::Point> ("visual/bsfitting", 1);
  ros::spin ();
}

void
PointCloud2Vector3d (PointCloudT::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (unsigned i = 0; i < cloud->size(); i++)
  {
    PointT &p = cloud->at(i);
    if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
      // cout<<p.x<<'\t'<<p.y<<'\t'<<p.z<<endl;
  }
}

void
print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
  printf ("R = | %6.3f %6.3f %6.3f %6.3f| \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
}
