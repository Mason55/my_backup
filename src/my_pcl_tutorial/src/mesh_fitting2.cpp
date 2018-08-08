/*
获取点云的第一帧做一个初始化
做前一帧和后一帧之间的ＩＣＰ，将结果应用于第一帧的点云
将第一帧的点云ＰＣＡ初始化成ＮＵＲＢＳ
利用fit将新帧的点云同初始化的ＮＵＲＢＳ之间做匹配
*/
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
/*下面是共享内存需要用到的头文件*/
#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <errno.h>
/*ROS自定义的消息类型*/
#include "point_msgs/Point.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

unsigned order (3);
  unsigned refinement (2);
    unsigned iterations_fit (20);
      unsigned iterations_icp (30);
        unsigned mesh_resolution (256);

typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
    const int BUF_SIZE=4096;

ros::Publisher pub;
  point_msgs::Point output;
    ros::Subscriber sub2;

/*初始化一个nurbs来作为全局变量*/
ON_NurbsSurface nurbs (3, false, order, order, order, order);
  ON_NurbsSurface nurbs_pre(3, false, order, order, order, order);
    ON_NurbsSurface nurbs_nex(3, false, order, order, order, order);

pcl::PolygonMesh mesh;
  std::vector<pcl::Vertices> mesh_vertices;

PointCloudT::Ptr cloud_tmp (new PointCloudT);//tmp点云
  PointCloudT::Ptr cloud_nex (new PointCloudT);//最新的点云
    PointCloudT::Ptr cloud_pre (new PointCloudT);//上一帧的点云

PointCloudT::Ptr cloud_nurb_pre (new PointCloudT);//存储fit后的nurbs_pre上的点

PointCloudT::Ptr cloud_nurb_nex (new PointCloudT);//存储fit后的nurbs_nex上的点

      // pcl::on_nurbs::NurbsDataSurface data_pca;//用于PCA的过程
        // pcl::on_nurbs::NurbsDataSurface data_sur;//用于fit的过程


Eigen::Matrix4d matrix_icp_a = Eigen::Matrix4d::Identity ();//ICP变换矩阵
  Eigen::Matrix4d matrix_icp_b = Eigen::Matrix4d::Identity ();//ICP变换矩阵

void PointCloud2Vector3d (pcl::PointCloud<PointT>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
void visualizeCurve (ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer);
void print4x4Matrix (const Eigen::Matrix4d & matrix);

void
pca_init(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){//接受一次消息，完成初始化

  PointCloudT::Ptr cloud (new PointCloudT);
    pcl::fromROSMsg (*cloud_msg, *cloud);
      printf ("  %lu cloud points in data set\n", cloud->size ());
        pcl::on_nurbs::NurbsDataSurface pca_data;
          PointCloud2Vector3d (cloud, pca_data.interior);
            nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &pca_data);//在这里完成了nurbs的初始化
              sub2.shutdown();
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

  PointCloudT::Ptr cloud_exchange (new PointCloudT);//中间交换的点云

  ON_NurbsSurface nurbs_pre(3, false, order, order, order, order);
    ON_NurbsSurface nurbs_nex(3, false, order, order, order, order);
      pcl::on_nurbs::NurbsDataSurface data_pre;//前一帧的ＰＣＡ的数据
        pcl::on_nurbs::NurbsDataSurface data_nex;//后一帧的ＰＣＡ的数据

  pcl::on_nurbs::FittingSurface::Parameter params;//fitting过程中要用到的参数
    params.interior_smoothness = 0.3;
      params.interior_weight = 0.4;
        params.boundary_smoothness = 0.01;
          params.boundary_weight = 10;

  double u=0.0;
    double v=0.0;
      int NUB_U =6;//那一共就有7*9=48个点在网格上
        int NUB_V =8;
          int i =0 , j=0, k=0;


/*前后帧的点云进行nurbs拟合*/

  if(!cloud_nex->empty()){//第一次注释掉的地方，对于运行的时候还是出现错误

    *cloud_pre = *cloud_nex;//取得前一帧的点云
      PointCloud2Vector3d (cloud_pre, data_pre.interior);
        nurbs_pre = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_pre);
          pcl::on_nurbs::FittingSurface fit_pre (&data_pre, nurbs_pre);

    for (unsigned i = 0; i < iterations_fit; i++){
      fit_pre.assemble (params);
        fit_pre.solve ();
    }
/*取点，生成点云cloud_pre*/
cloud_nurb_pre->width = 63;
  cloud_nurb_pre->height = 1;
    cloud_nurb_pre->points.resize (cloud_nurb_pre->width * cloud_nurb_pre->height);
          ON_3dPoint point_pre;
            for (size_t i = 0; i < cloud_nurb_pre->points.size (); ++i) {
              for(j=0;j<=NUB_U;j++) {  //添加等于号是为了包含边界
                  for(k=0;k<=NUB_V;k++)  {
                    point_pre = fit_pre.m_nurbs.PointAt(u,v);
                      cloud_nurb_pre->points[i].x = point_pre.x;
                        cloud_nurb_pre->points[i].y =point_pre.y;
                          cloud_nurb_pre->points[i].z =point_pre.z;
                              v += (double)1/NUB_V;
                  }
                u += (double)1/NUB_U;
                v=0.0;
              }
            }
  }

  pcl::fromROSMsg (*cloud_msg, *cloud_nex);

  PointCloud2Vector3d (cloud_nex, data_nex.interior);
    nurbs_nex = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_nex);
      pcl::on_nurbs::FittingSurface fit_nex (&data_nex, nurbs_nex);

  for (unsigned i = 0; i < iterations_fit; i++){//第二次注释的地方
    fit_nex.assemble (params);
      fit_nex.solve ();
  }
  /*取点，生成点云cloud_nex*/
  cloud_nurb_nex->width = 63;
    cloud_nurb_nex->height = 1;
      cloud_nurb_nex->points.resize (cloud_nurb_nex->width * cloud_nurb_nex->height);

        for (size_t i = 0; i < cloud_nurb_nex->points.size (); ++i) {
          ON_3dPoint point_nex;
            for(j=0;j<=NUB_U;j++) {  //添加等于号是为了包含边界
                for(k=0;k<=NUB_V;k++)  {
                  point_nex = fit_nex.m_nurbs.PointAt(u,v);
                    cloud_nurb_nex->points[i].x = point_nex.x;
                      cloud_nurb_nex->points[i].y =point_nex.y;
                        cloud_nurb_nex->points[i].z =point_nex.z;
                          v += (double)1/NUB_V;
                }
            u += (double)1/NUB_U;
              v=0.0;
            }
          }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setMaximumIterations (iterations_icp);
        icp.setInputSource (cloud_nurb_nex);
          icp.setInputTarget (cloud_nurb_pre);
            icp.align (*cloud_nurb_pre);
          // icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations_icp << " ICP iteration(s) in "<< std::endl;

    if (icp.hasConverged ())  {
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations_icp << " : cloud_pre -> cloud_nex" << std::endl;
          matrix_icp_b = icp.getFinalTransformation ().cast<double>();
            // Display in terminal the transformation matrix
            std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
              print4x4Matrix (matrix_icp_b);
    }
    else    {

      PCL_ERROR ("\nICP has not converged.\n");
        return;
    }

/*提取前后帧率的nurbs上的点进行icp*/
    matrix_icp_a = matrix_icp_a * matrix_icp_b;
      // std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
        std::cout << "matrix_icp_a: " << '\n';
          print4x4Matrix (matrix_icp_a);

  pcl::transformPointCloud (*cloud_nex, *cloud_exchange, matrix_icp_a);

/*使用点云进行了曲面拟合*/
// pcl::on_nurbs::Triangulation::convertSurface2Vertices (nurbs,  cloud, mesh_vertices, mesh_resolution);
  pcl::on_nurbs::NurbsDataSurface data_sur;
    PointCloud2Vector3d (cloud_exchange, data_sur.interior);
      pcl::on_nurbs::FittingSurface fit (&data_sur, nurbs);
        for (unsigned i = 0; i < iterations_fit; i++){
          fit.assemble (params);
            fit.solve ();
        }

  // double u=0.0;
  //   double v=0.0;
  //     int NUB_U =6;//那一共就有7*9=48个点在网格上
  //       int NUB_V =8;
  //         float array[600];
  //           ON_3dPoint mypoint;
  //               int i =0 , j=0, k=0;
  Matrix4f car2base; //这是把手眼标定的矩阵拿过来用了
    car2base<<-0.0213  , -0.8264 ,  -0.5626  ,  1.1210,
              -0.9997  ,  0.0264 ,  -0.0009  , -0.1194,
               0.0156  ,  0.5624 ,  -0.8267  ,  1.0983,
              -0.0000  , -0.0000 ,   0.0000  ,  1.0000;
      Matrix4f parend2car;
        Matrix4f parend2base;
          float array[600];
            ON_3dPoint mypoint;

  for(i=0;i<=NUB_U;i++){//添加等于号是为了包含边界
    for(j=0;j<=NUB_V;j++){
      mypoint = fit.m_nurbs.PointAt(u,v);
        // mypoint = nurbs.PointAt(u,v);
          array[k] = mypoint.x;
            array[k+1] =mypoint.y;
              array[k+2] =mypoint.z;
                k += 3;
                  v += (double)1/NUB_V;
    }
      u += (double)1/NUB_U;
        v=0.0;
  }

  std::vector<float> array1(array,array+600);
    output.data = array1;

  ON_3dPoint par_end_car = fit.m_nurbs.PointAt(0.0,0.5);
    // ON_3dPoint par_end_car = nurbs.PointAt(0.0,0.5);

  parend2car<<1,0,0,par_end_car.x,
              0,1,0,par_end_car.y,
              0,0,1,par_end_car.z,
              0,0,0,1;
    parend2base=car2base*parend2car;

  pub.publish (output);
}

int
main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "visual_bsfitting");
    ros::NodeHandle nh;
      ros::NodeHandle nh2;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/voxel/grid", 1, cloud_cb);
    sub2 = nh2.subscribe ("/voxel/grid", 1, pca_init);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<point_msgs::Point> ("visual/bsfitting", 1);

  // Spin
  ros::spin ();
}

void
PointCloud2Vector3d (PointCloudT::Ptr cloud, pcl::on_nurbs::vector_vec3d &data){

  for (unsigned i = 0; i < cloud->size (); i++)  {
    PointT &p = cloud->at (i);
      if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
        data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
      // cout<<p.x<<'\t'<<p.y<<'\t'<<p.z<<endl;
  }
}

// void
// NURBS2PointCloud (PointCloudT::Ptr cloud, pcl::on_nurbs::vector_vec3d &data){
//
//   for (unsigned i = 0; i < cloud->size (); i++)  {
//     PointT &p = cloud->at (i);
//       if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
//         data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
//       // cout<<p.x<<'\t'<<p.y<<'\t'<<p.z<<endl;
//   }
// }

void
print4x4Matrix (const Eigen::Matrix4d & matrix){
printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
printf ("R = | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
}
