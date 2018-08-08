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
      unsigned iterations_icp (20);
        unsigned mesh_resolution (256);

typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

const int BUF_SIZE=4096;
  const double PI = 3.141592653;

ros::Publisher pub;
  point_msgs::Point output;
    ros::Subscriber sub2;

/*初始化一个nurbs来作为全局变量*/
ON_NurbsSurface nurbs (3, false, order, order, order, order);
  ON_NurbsSurface nurbs_pre(3, false, order, order, order, order);
    ON_NurbsSurface nurbs_nex(3, false, order, order, order, order);

pcl::PolygonMesh mesh;
  std::vector<pcl::Vertices> mesh_vertices;
// nurbs.MakeClampedUniformKnotVector (0, 1.0);
// nurbs.MakeClampedUniformKnotVector (1, 1.0);
// pcl::visualization::PCLVisualizer viewer ("B-spline surface fitting");
// PointCloudT::Ptr mesh_cloud (new PointCloudT);//初始化一次的点云
// PointCloudT::Ptr mesh_cloud_new (new PointCloudT);//不断更新的初始化的点云

PointCloudT::Ptr cloud_tmp (new PointCloudT);//缓存点云
  PointCloudT::Ptr cloud_nex (new PointCloudT);//最新的点云
    PointCloudT::Ptr cloud_pre (new PointCloudT);//上一帧的点云

pcl::on_nurbs::NurbsDataSurface data_pca;//用于PCA的过程
  pcl::on_nurbs::NurbsDataSurface data_sur;//用于fit的过程
    pcl::on_nurbs::NurbsDataSurface data_pre;//前一帧的ＰＣＡ
      pcl::on_nurbs::NurbsDataSurface data_nex;//后一帧的ＰＣＡ

Eigen::Matrix4f matrix_icp = Eigen::Matrix4f::Identity ();//ICP变换矩阵

void PointCloud2Vector3d (pcl::PointCloud<PointT>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
void visualizeCurve (ON_NurbsCurve &curve, ON_NurbsSurface &surface, pcl::visualization::PCLVisualizer &viewer);
void MeshVertices2Vector3d (std::vector<pcl::Vertices>  mesh_vertices, pcl::on_nurbs::vector_vec3d &data);
void print4x4Matrix (const Eigen::Matrix4f & matrix);

void
mesh_init(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

  PointCloudT::Ptr mesh_cloud (new PointCloudT);//初始化一次的点云
  // PointCloudT::Ptr mesh_cloud (new PointCloudT);//初始化一次的点云
    pcl::fromROSMsg (*cloud_msg, *mesh_cloud);
      printf ("  %lu mesh_cloud points in data set\n", mesh_cloud->size ());

  if (!mesh_cloud->empty()) {

      *cloud_tmp = *mesh_cloud;
        sub2.shutdown();
  }
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

  PointCloudT::Ptr mesh_cloud_new (new PointCloudT);//不断更新的初始化的点云
    pcl::on_nurbs::FittingSurface::Parameter params;
      params.interior_smoothness = 0.3;
        params.interior_weight = 0.4;
          params.boundary_smoothness = 0.01;
            params.boundary_weight = 10;

/*前后帧的点云进行icp*/
    if(!cloud_nex->empty())  {

      *cloud_pre = *cloud_nex;//取得前一帧的点云
    }

    pcl::fromROSMsg (*cloud_msg, *cloud_nex);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setMaximumIterations (iterations_icp);
        icp.setInputSource (cloud_pre);
          icp.setInputTarget (cloud_nex);
            icp.align (*cloud_pre);
          // icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations_icp << " ICP iteration(s) in "<< std::endl;

    if (icp.hasConverged ())    {

    }
    else    {

      PCL_ERROR ("\nICP has not converged.\n");
      return;
    }
/*１．获取前后ＰＣＡ的中心位置，　中心位置的法向量，
  ２．将获取的向量和位移转化为矩阵
  ３．将这个矩阵应用在第一帧的点云上
*/
// cloud_tmp (new PointCloudT);//缓存点云
// PointCloudT::Ptr cloud_nex (new PointCloudT);//最新的点云
// PointCloudT::Ptr cloud_pre (new PointCloudT);//上一帧的点云

    PointCloud2Vector3d (cloud_pre, data_pre.interior);
      nurbs_pre = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_pre);

    PointCloud2Vector3d (cloud_nex, data_nex.interior);
      nurbs_nex = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_nex);

    ON_3dPoint point_pre = nurbs_pre.PointAt(0.5,0.5);
    ON_3dPoint point_nex = nurbs_nex.PointAt(0.5,0.5);

    ON_3dVector normal_pre = nurbs_pre.NormalAt(0.5,0.5);
    ON_3dVector normal_nex = nurbs_nex.NormalAt(0.5,0.5);

    ON_3dVector axis = ON_CrossProduct(normal_nex,normal_pre);//旋转轴
    axis = axis / (sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z));//单位化
    double l_pre=sqrt(normal_pre.x*normal_pre.x+normal_pre.y*normal_pre.y+normal_pre.z*normal_pre.z);
    double l_nex=sqrt(normal_nex.x*normal_nex.x+normal_nex.y*normal_nex.y+normal_nex.z*normal_nex.z);
    double angle = ON_DotProduct(normal_pre,normal_nex)/(l_pre*l_nex);//旋转角度

// Eigen::Matrix4f matrix_icp = Eigen::Matrix4d::Identity ();//ICP变换矩阵
    double c=cos(angle); double s = sin(angle); double x= axis.x;
    double y= axis.y; double z = axis.z;
    matrix_icp(0,0)=c+x*x*(1-c);
    matrix_icp(0,1)=x*y*(1-c)-z*s;
    matrix_icp(0,2)=y*s+x*z*(1-c);
    matrix_icp(1,0)=z*s+x*y*(1-c);
    matrix_icp(1,1)=c+y*y*(1-c);
    matrix_icp(1,2)=-x*s+y*z*(1-c);
    matrix_icp(2,0)=-y*s+x*z*(1-c);
    matrix_icp(2,1)=x*s+y*z*(1-c);
    matrix_icp(2,2)=c+z*z*(1-c);

    matrix_icp(0,3)=point_nex.x-point_pre.x;
    matrix_icp(0,3)=point_nex.y-point_pre.y;
    matrix_icp(0,3)=point_nex.z-point_pre.z;
    // ON_3dPoint point_nex = fit_nex.m_nurbs.PointAt(0.5,0.5);
    //   ON_3dVector normal_nex = fit_nex.m_nurbs.NormalAt(0.5,0.5);
    //
    // if(normal_nex.IsZero() || normal_pre.IsZero() ){}
    //   else{
    //       ON_3dVector axis = ON_CrossProduct(normal_nex,normal_pre);//旋转轴
    //         axis = axis / (sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z));//单位化
    //           double l_pre=sqrt(normal_pre.x*normal_pre.x+normal_pre.y*normal_pre.y+normal_pre.z*normal_pre.z);
    //             double l_nex=sqrt(normal_nex.x*normal_nex.x+normal_nex.y*normal_nex.y+normal_nex.z*normal_nex.z);
    //               double angle = ON_DotProduct(normal_pre,normal_nex)/(l_pre*l_nex);//旋转角度
    //                 double c=cos(angle); double s = sin(angle); double x= axis.x;
    //                   double y= axis.y; double z = axis.z;
    //
    //       matrix_icp_b(0,0)=c+x*x*(1-c);
    //         matrix_icp_b(0,1)=x*y*(1-c)-z*s;
    //           matrix_icp_b(0,2)=y*s+x*z*(1-c);
    //             matrix_icp_b(1,0)=z*s+x*y*(1-c);
    //               matrix_icp_b(1,1)=c+y*y*(1-c);
    //                 matrix_icp_b(1,2)=-x*s+y*z*(1-c);
    //                   matrix_icp_b(2,0)=-y*s+x*z*(1-c);
    //                     matrix_icp_b(2,1)=x*s+y*z*(1-c);
    //                       matrix_icp_b(2,2)=c+z*z*(1-c);
    //                         matrix_icp_b(0,3)=point_nex.x-point_pre.x;
    //                           matrix_icp_b(0,3)=point_nex.y-point_pre.y;
    //                             matrix_icp_b(0,3)=point_nex.z-point_pre.z;
    //
    //       std::cout << "matrix_icp_b: " << '\n';
    //         print4x4Matrix(matrix_icp_b);
    //           matrix_icp_a = matrix_icp_a * matrix_icp_b;
    //             std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    //               std::cout << "matrix_icp_a: " << '\n';
    //                 print4x4Matrix (matrix_icp_a);
    // }//

    print4x4Matrix(matrix_icp);


    pcl::transformPointCloud (*cloud_tmp, *mesh_cloud_new, matrix_icp);
    // *cloud_tmp = *mesh_cloud_new;
    PointCloud2Vector3d (cloud_tmp, data_pca.interior);//问题出在tmp的值并没有更新！！！
    printf ("  %lu cloud_tmp points in data set\n", cloud_tmp->size ());
    nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_pca);
    *cloud_tmp = *mesh_cloud_new;
  /*使用点云进行了曲面拟合*/
  // pcl::on_nurbs::Triangulation::convertSurface2Vertices (nurbs,  cloud, mesh_vertices, mesh_resolution);

  // pcl::on_nurbs::NurbsDataSurface data_sur;//用于fit的过程

    PointCloud2Vector3d (cloud_nex, data_sur.interior);
    // PointCloud2Vector3d (cloud_pre, data_sur.interior);
    pcl::on_nurbs::FittingSurface fit (&data_sur, nurbs);
    std::cout<<"cloud:"<<std::endl;
    // std::cout<<*(nurbs.PointAt(0,0))<<std::endl;
    // pcl::on_nurbs::Triangulation::convertSurface2Vertices (nurbs,  cloud, mesh_vertices, mesh_resolution);
    printf ("  %lu points in data set\n", cloud_tmp->size ());
    printf ("  surface fitting ...\n");
    // std::cout<<*(nurbs.PointAt(0,0))<<std::endl;
      // ros::topic::waitForMessage("/voxel/grid",);
    /*使用mesh的vertices进行曲面拟合*/
    // MeshVertices2Vector3d (mesh_vertices, data.interior);
    // pcl::on_nurbs::FittingSurface fit (&data, nurbs);
    // std::cout<<"mesh:"<<std::endl;
    // std::cout<<*(nurbs.PointAt(0,0))<<std::endl;
    // std::cout<<*(nurbs.PointAt(0,0))<<std::endl;
    // pcl::PolygonMesh mesh;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // std::vector<pcl::Vertices> mesh_vertices;
    // std::string mesh_id = "mesh_nurbs_2";
    // pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);
    // viewer.addPolygonMesh (mesh, mesh_id);
    // std::cout<<"vertices:"<<std::endl;
    // std::cout<<mesh_vertices[100].vertices[0]<<std::endl;
    // std::cout<<mesh_vertices[100].vertices[1]<<std::endl;
    // std::cout<<mesh_vertices[100].vertices[2]<<std::endl;
    // mesh_vertices[i].vertices[0];
  /*Because the refinemrnt is too slow so I annotate this refinemnt section*/
    // for (unsigned i = 0; i < refinement; i++)
    // {
    //   fit.refine (0);
    //   fit.refine (1);
    //   fit.assemble (params);
    //   fit.solve ();
    // //   //pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    // //   //viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
    // //   //viewer.spinOnce ();
    // }
    // pcl::on_nurbs::Triangulation::convertSurface2Vertices (nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    // surface fitting with final refinement level
    for (unsigned i = 0; i < iterations_fit; i++)
    {
      fit.assemble (params);
      fit.solve ();
    }

    double u=0.0;
    double v=0.0;
    int NUB_U =6;//那一共就有7*9=48个点在网格上
    int NUB_V =8;
    float array[600];
    ON_3dPoint mypoint;
    int i =0 , j=0, k=0;

    Matrix4f car2base; //这是把手眼标定的矩阵拿过来用了
    car2base<<-0.0213  , -0.8264 ,  -0.5626   , 1.1210,
     -0.9997  ,  0.0264 ,  -0.0009 ,  -0.1194,
      0.0156   , 0.5624  , -0.8267   , 1.0983,
     -0.0000 ,  -0.0000 ,   0.0000  ,  1.0000;
    Matrix4f parend2car;
    Matrix4f parend2base;

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
    // std::cout<<array[0]<<std::endl;
    // std::cout<<array[1]<<std::endl;
    // std::cout<<array[2]<<std::endl;

    std::vector<float> array1(array,array+600);
    output.data = array1;

    ON_3dPoint par_end_car = fit.m_nurbs.PointAt(0.0,0.5);
    parend2car<<1,0,0,par_end_car.x,
                0,1,0,par_end_car.y,
                0,0,1,par_end_car.z,
                0,0,0,1;
    parend2base=car2base*parend2car;
    // std::cout<<parend2base(0,3)<<std::endl;
    // std::cout<<parend2base(1,3)<<std::endl;
    // std::cout<<parend2base(2,3)<<std::endl;

  // }
  // else{
  //   cout<<"cloud_nex is 空的"<<endl;
  // }
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "visual_bsfitting");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/voxel/grid", 1, cloud_cb);
  sub2 = nh2.subscribe ("/voxel/grid", 1, mesh_init);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<point_msgs::Point> ("visual/bsfitting", 1);

  // Spin
  ros::spin ();
}

void
PointCloud2Vector3d (PointCloudT::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (unsigned i = 0; i < cloud->size (); i++)
  {
    PointT &p = cloud->at (i);
    if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
      // cout<<p.x<<'\t'<<p.y<<'\t'<<p.z<<endl;
  }
}

// std::vector<pcl::Vertices> mesh_vertices;
void
MeshVertices2Vector3d (std::vector<pcl::Vertices>  mesh_vertices, pcl::on_nurbs::vector_vec3d &data)
{
  // for (unsigned i = 0; i < mesh_vertices.size (); i+=3)
  // {
  //   Point &p;
  //   p.x = mesh_vertices[i];
  //   p.y = mesh_vertices[i+1];
  //   p.z = mesh_vertices[i+2];
  //   // p.y = mesh_vertices->at (i+1);
  //   // p.z = mesh_vertices->at (i+2);
  //
  //   if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
  //     data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
  // }
  PointT p;

  for (int i=0; i<mesh_vertices.size(); i++) {
		assert(mesh_vertices[i].vertices.size() == 3);
    // for (int i=0; i<mesh_vertices.size(); i++)
    // {
    //
    // }
    p.x = mesh_vertices[i].vertices[3];
    p.y = mesh_vertices[i].vertices[4];
    p.z = mesh_vertices[i].vertices[5];
    cout<<p.x<<'\t'<<p.y<<'\t'<<p.z<<endl;
	}
  if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
    data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
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
