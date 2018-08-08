#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/from_meshes.h>
#include <string.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
// #include <tf_conversions/tf/eigen.h>
/*ROS自定义的消息类型*/
#include "point_msgs/Point.h"
#include "Kine.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
const int BUF_SIZE=4096;

unsigned order (3);
unsigned iterations_fit (20);


ON_NurbsSurface nurbs (3,false,order,order,order,order);
// ON_NurbsSurface nurbs_2 (3,false,order,order,order,order);
// PointCloudT::Ptr init_cam_cloud (new PointCloudT);
// PointCloudT::Ptr init_tool_cloud (new PointCloudT);

// PointCloudT::Ptr curr_cam_cloud (new PointCloudT);
// PointCloudT::Ptr curr_tool_cloud (new PointCloudT);

class CloudFit
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub1_;//曲面拟合
  ros::Subscriber sub2_;//初始化
  ros::Subscriber sub3_;//接受工具坐标系
  ros::Publisher pub1_;//发布绘图点
  ros::Publisher pub2_;//发布四元数
  ros::Publisher pub3_;//测试发布点云

  Matrix4f cam2tool_mat = Matrix4f::Identity ();
  Matrix4f tool2cam_mat = Matrix4f::Identity ();
  Matrix4f cam2base_mat = Matrix4f::Identity ();
  Matrix4f parper2tool_mat = Matrix4f::Identity ();
  Matrix4f parper2base_mat = Matrix4f::Identity ();
  Matrix4f target2base_mat = Matrix4f::Identity ();
  Matrix4f parper2tool_mat_2 = Matrix4f::Identity ();
  Vector4f tool_point;
  Vector4f cam_point;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
  point_msgs::Point visual_output;
  point_msgs::Point move_output;
  float array[600];
  float tmp_trans[7];
  float quan_trans[7];//要发布的下一个位置的四元数
  ON_3dPoint my_point;
  Kine k_cal;
  ON_3dPoint tra_point[4];//路径点
  ON_3dPoint sor_point;
  ON_3dPoint pub_point[4];//要发布的路径点
  int init_flag = 1;

  // PointCloudT::Ptr init_cam_cloud (new PointCloudT);
  // PointCloudT::Ptr init_tool_cloud (new PointCloudT);

  // PointCloudT::Ptr curr_cam_cloud (new PointCloudT);
  // PointCloudT::Ptr curr_tool_cloud (new PointCloudT);

public:
  CloudFit()
  {
    sub2_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::pcaInit, this);
    sub1_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::cloudCb, this);
    sub3_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::runningPro, this);
    pub1_ = nh_.advertise<point_msgs::Point> ("/visual/bsfitting", 1);
    pub2_ = nh_.advertise<point_msgs::Point> ("/ur/move/tra", 1);//发布的是每一段的位移
    pub3_ = nh_.advertise<sensor_msgs::PointCloud2> ("dianyun", 1);

    cam2base_mat<<-0.0213  , -0.8264  ,  -0.5626  ,  1.1210,//这是把手眼标定的矩阵拿过来用了
                  -0.9997  ,  0.0264  ,  -0.0009  , -0.1194,
                   0.0156  ,  0.5624  ,  -0.8267  ,  1.0983,
                  -0.0000  , -0.0000  ,   0.0000  ,  1.0000;
    tool_point<<0.0,0.0,0.0,1.0;
    cam_point<<0.0,0.0,0.0,1.0;
    // camToTool();
    // sleep(2);
    // runningPro();
    ROS_INFO("CloudFit works/n");
  };
  ~CloudFit(){    };

  void print4x4Matrix (const Eigen::Matrix4f & matrix)
  {
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
    printf ("R = | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
  }

  void camToTool()
  {
      try
      {
        listener.lookupTransform("/tool0_controller"/*target_frame*/, "/camera"/*source_frame*/,\
                                                     ros::Time(0), transform);
        listener.lookupTransform("/tool0_controller"/*目标坐标系*/, "/base"/*source_frame*/,\
                                                     ros::Time(0), transform2);
        //要定义一个中间数数组变量，把transform转化
        tmp_trans[0] = transform.getOrigin().x();
        tmp_trans[1] = transform.getOrigin().y();
        tmp_trans[2] = transform.getOrigin().z();
        tmp_trans[3] = transform.getRotation().x();
        tmp_trans[4] = transform.getRotation().y();
        tmp_trans[5] = transform.getRotation().z();
        tmp_trans[6] = transform.getRotation().w();
        k_cal.quat2Dcm(tmp_trans, cam2tool_mat);


        quan_trans[0] = transform2.getOrigin().x();
        quan_trans[1] = transform2.getOrigin().y();
        quan_trans[2] = transform2.getOrigin().z();
        quan_trans[3] = transform2.getRotation().x();
        quan_trans[4] = transform2.getRotation().y();
        quan_trans[5] = transform2.getRotation().z();
        quan_trans[6] = transform2.getRotation().w();//机械臂目标坐标系的四元数

        // CloudFit::print4x4Matrix(cam2tool_mat);

        tool2cam_mat = cam2tool_mat.inverse();
      }
      catch (tf::TransformException ex)
      {
        // CloudFit::print4x4Matrix(cam2tool_mat);
        ROS_ERROR("%s",ex.what());
      }
      ROS_INFO("camToTool work/n");
 };

  void pointCloud2Vector3d (pcl::PointCloud<PointT>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
  {
    for (unsigned i = 0; i < cloud->size (); i++)
    {
      PointT &p = cloud->at (i);
      if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
          data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
    }
    // ROS_INFO("pointCloud2Vector3dworks/n");
  };

  void pcaInit(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)//接受一次消息，完成初始化
  {
      PointCloudT::Ptr cloud (new PointCloudT);
      pcl::fromROSMsg (*cloud_msg, *cloud);
      int cloud_size=cloud->size();
      // printf ("  %lu cloud points in init_cam_cloud set\n", cloud->size ());
     // *init_tmp_cloud = *init_cam_cloud;
     // printf ("  %lu cloud points in init_tool_cloud set\n", init_tmp_cloud->size ());
      cout<<"cloud_size="<<cloud_size<<endl;
      if(cloud->size()>=cloud_size &&init_flag==1)
      {
        // sub2_.shutdown();//问题不在订阅
        // *init_cam_cloud = *cloud;
        pcl::io::savePCDFileASCII ("init_pcd.pcd", *cloud);
        ROS_INFO(" in pca %lu cloud points in init_cam_cloud set\n", cloud->size ());
        // sleep(1);
        // copyPointCloud(*init_cam_cloud,*init_tmp_cloud);
        // copyPointCloud(init_tmp_cloud, init_cam_cloud->makeShared());
        init_flag=0;
        // ROS_INFO("pcaInit works/n");
      }
//      printf ("  %lu cloud points in init_tmp_cloud set\n", init_tmp_cloud->size ());
//      pcl::transformPointCloud (*init_tmp_cloud/*in*/, *init_tool_cloud/*out*/, cam2tool_mat);//这个函数需要搞清楚
//      printf ("  %lu cloud points in init_tool_cloud set\n", init_tool_cloud->size ());

//      // Convert to ROS data type

//      pcl::PCLPointCloud2 cloud_2_filtered;
//      pcl::toPCLPointCloud2 (*init_tool_cloud, cloud_2_filtered);
//      sensor_msgs::PointCloud2 dianyun;
//      pcl_conversions::fromPCL(cloud_2_filtered, dianyun);

//      // Publish the data
//      pub3_.publish (dianyun);

  };

  void cloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    PointCloudT::Ptr cloud (new PointCloudT);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    int cloud_size=cloud->size();
    ROS_INFO (" in cloudCb %lu cloud points in init_cam_cloud set\n", cloud->size ());
   // *init_tmp_cloud = *init_cam_cloud;
   // printf ("  %lu cloud points in init_tool_cloud set\n", init_tmp_cloud->size ());
    cout<<"cloud_size="<<cloud_size<<endl;
    if(cloud->size()>=cloud_size)
    {
        // *curr_cam_cloud = *cloud;
        pcl::io::savePCDFileASCII ("curr_pcd.pcd", *cloud);
        ROS_INFO(" in pca %lu cloud points in init_cam_cloud set\n", cloud->size ());
    }
  };

  void runningPro (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  // void runningPro ()
  { 
    camToTool();

      PointCloudT::Ptr init_cam_cloud (new PointCloudT);
      PointCloudT::Ptr init_tool_cloud (new PointCloudT);

      PointCloudT::Ptr curr_cam_cloud (new PointCloudT);
      PointCloudT::Ptr curr_tool_cloud (new PointCloudT);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("init_pcd.pcd", *init_cam_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file init_pcd.pcd \n");
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("curr_pcd.pcd", *curr_cam_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file curr_pcd.pcd \n");
  }

      pcl::on_nurbs::NurbsDataSurface data;//pca的数据
      pcl::on_nurbs::NurbsDataSurface pca_data;
      pcl::on_nurbs::FittingSurface::Parameter params;//参数必须放在这个函数的内部
      params.interior_smoothness = 0.2;
      params.interior_weight = 10;
      params.boundary_smoothness = 0.0005;
      params.boundary_weight = 0.01;

    int cloud_1_size=init_cam_cloud->size();
    int cloud_2_size=curr_cam_cloud->size();

    if(init_cam_cloud->size()>=cloud_1_size&&curr_cam_cloud->size()>=cloud_2_size)
    {
      ROS_INFO ("  %lu cloud points in init_cam_cloud set\n", init_cam_cloud->size ());
      pcl::transformPointCloud (*init_cam_cloud/*in*/, *init_tool_cloud/*out*/, cam2tool_mat);//这个函数需要搞清楚
      ROS_INFO ("  %lu cloud points in init_tool_cloud set\n", init_tool_cloud->size ());

      ROS_INFO ("  %lu cloud points in curr_cam_cloud set\n", curr_cam_cloud->size ());
      pcl::transformPointCloud (*curr_cam_cloud/*in*/, *curr_tool_cloud/*out*/, cam2tool_mat);//这个函数需要搞清楚
      ROS_INFO ("  %lu cloud points in curr_tool_cloud set\n", curr_tool_cloud->size ());

      CloudFit::pointCloud2Vector3d (init_tool_cloud, pca_data.interior);
      nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &pca_data);//在这里完成了nurbs的初始化
      pca_data.clear_interior();
      cout<<nurbs.SizeOf()<<endl;
            cout<<"dim:"<<nurbs.m_dim         <<endl;
            cout<<"is_rat:"<<nurbs.m_is_rat      <<endl;
            cout<<"order[0]:"<<nurbs.m_order[0]    <<endl;
            cout<<"order[1]:"<<nurbs.m_order[1]    <<endl;
            cout<<"cv_count[0]:"<<nurbs.m_cv_count[0] <<endl;
            cout<<"cv_count[1]:"<<nurbs.m_cv_count[1] <<endl;
            cout<<"cv_stride[1]:"<<nurbs.m_cv_stride[1]<<endl;
            cout<<"cv_stride[0]:"<<nurbs.m_cv_stride[0]<<endl;
    }
    
      double u=0.0;
      double v=0.0;
      int NUB_U =6;//那一共就有7*9=48个点在网格上
      int NUB_V =8;
      int i =0, j=0, k=0;

      for(i=0;i<=NUB_U;i++)//添加等于号是为了包含边界
      {
        for(j=0;j<=NUB_V;j++)
        {
          // my_point = fit.m_nurbs.PointAt(u,v);
          my_point = nurbs.PointAt(u,v);
          tool_point[0] = my_point.x;
          tool_point[1] = my_point.y;
          tool_point[2] = my_point.z;
          tool_point[3] = 1.0;
          // cout<<tool_point[0]<<endl;
          // cout<<tool_point[1]<<endl;
          // cout<<tool_point[2]<<endl;
          cam_point = tool2cam_mat*tool_point;//转换到相机坐标系下
          array[k]   = cam_point[0];
          array[k+1] = cam_point[1];
          array[k+2] = cam_point[2];
          k += 3;
          v += (double)1/NUB_V;
        }
        u += (double)1/NUB_U;
        v=0.0;
      }
      std::vector<float> array1(array,array+600);

      visual_output.data = array1;

      // nurbs_2.Destroy();//依然不好改善

      // ON_3dPoint par_end = fit.m_nurbs.PointAt(0.5,0.5);
      ON_3dPoint par_end = nurbs.PointAt(0.5,0.5);
      // ON_3dPoint par_end2 = nurbs_2.PointAt(0.5,0.5);
      parper2tool_mat<<1,0,0,par_end.x,
                      0,1,0,par_end.y,
                      0,0,1,par_end.z,
                      0,0,0,1;

      parper2base_mat=cam2base_mat*tool2cam_mat*parper2tool_mat;
      CloudFit::print4x4Matrix(parper2base_mat);
      pub1_.publish (visual_output);
      // pca_data.clear_interior();
      // ROS_INFO("cloudCb works/n");
  
     
      // pcl::fromROSMsg (*cloud_msg, *cloud_cam);
      // //这里要添加一步转化为tool0坐标系下的点云
      // pcl::transformPointCloud (*cloud_cam/*in*/, *cloud_tool/*out*/, cam2tool_mat);
      // CloudFit::pointCloud2Vector3d (cloud_tool, data.interior);
      // pcl::on_nurbs::FittingSurface fit(&data, nurbs);

      // for (unsigned ame = 0; ame < iterations_fit; ++ame)
      // {
      //   fit.assemble (params);
      //   fit.solve ();
      // }

      // tra_point[0].x=0.762; tra_point[0].y=-0.056; tra_point[0].z=0.058;
      // sor_point = fit.m_nurbs.PointAt(0,0.5);
      // double z_distance = sor_point.z-tra_point[0].z;

      // if(z_distance<0)
      // {
      //   quan_trans[2] += (sor_point.z)/50;
      //   std::vector<float> array2(quan_trans,quan_trans+6);
      //   move_output.data = array2;
      //   pub2_.publish (move_output);
      // }

     
  }

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "ur_mesh");
  // ros::NodeHandle nh;
  CloudFit try1;
  // while(ros::ok())
  // {
  //   // try1.camToTool();
  //   try1.runningPro();
  // }

  ros::spin ();
  return 0;
}
