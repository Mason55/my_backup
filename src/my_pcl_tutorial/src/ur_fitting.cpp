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
//同步消息接受
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

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
PointCloudT::Ptr init_cam_cloud (new PointCloudT);
PointCloudT::Ptr init_cloud (new PointCloudT);
PointCloudT::Ptr init_tool_cloud (new PointCloudT);

class CloudFit
{
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
  Matrix4f init_tool2base_mat = Matrix4f::Identity ();
  Matrix4f curr_tool2base_mat = Matrix4f::Identity ();
  Matrix4f init2curr_mat = Matrix4f::Identity ();
  Matrix4f curr2init_mat = Matrix4f::Identity ();
  Matrix4f init_cam2tool_mat = Matrix4f::Identity ();
  Vector4f init_paper_point;
  Vector4f curr_paper_point;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
  tf::StampedTransform init_transform;
  tf::StampedTransform init_transform_cam2tool;

  float cam2tool_quan[7];
  float tool2base_quan[7];//要发布的下一个位置的四元数
  float init_trans[7];//第一个姿态的7个表示位置的xyz,xyzw
  float init_trans_cam2tool[7];//第一个姿态的7个表示位置的xyz,xyzw

  point_msgs::Point visual_output;
  point_msgs::Point move_output;

  float array[600];
  
  ON_3dPoint my_point;
  Kine k_cal;
  ON_3dPoint tra_point[4];//路径点
  ON_3dPoint sor_point;
  ON_3dPoint pub_point[4];//要发布的路径点
  int init_flag = 1;
  
public:
  CloudFit()
  {
    sub2_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::pcaInit, this);
    sub1_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::cloudCb, this);

    pub1_ = nh_.advertise<point_msgs::Point> ("/visual/bsfitting", 1);
    pub2_ = nh_.advertise<point_msgs::Point> ("/ur/move/tra", 1);//发布的是每一段的位移
    pub3_ = nh_.advertise<sensor_msgs::PointCloud2> ("dianyun", 1);

    cam2base_mat<<-0.0213  , -0.8264  ,  -0.5626  ,  1.1210,//这是把手眼标定的矩阵拿过来用了
                  -0.9997  ,  0.0264  ,  -0.0009  , -0.1194,
                   0.0156  ,  0.5624  ,  -0.8267  ,  1.0983,
                  -0.0000  , -0.0000  ,   0.0000  ,  1.0000;
    init_paper_point<<0.0,0.0,0.0,1.0;
    curr_paper_point<<0.0,0.0,0.0,1.0;
    ROS_INFO("CloudFit works/n");
  };
  ~CloudFit(){    };
 
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
      pcl::fromROSMsg (*cloud_msg, *init_cam_cloud);
      int cloud_size=init_cam_cloud->size();
      // printf ("  %lu cloud points in init_cam_cloud set\n", init_cam_cloud->size ());
     // *init_cloud = *init_cam_cloud;
     // printf ("  %lu cloud points in init_tool_cloud set\n", init_cloud->size ());
      // cout<<"cloud_size="<<cloud_size<<endl;
      if(init_cam_cloud->size()>=cloud_size &&init_flag==1) 
      {
        *init_cloud = *init_cam_cloud;
        printf ("  %lu cloud points in init_cloud set\n", init_cloud->size ());
        listener.lookupTransform("/base"/*目标坐标系*/, "/tool0_controller"/*source*/,ros::Time(0), init_transform);
        // sleep(1);
        init_trans[0] = init_transform.getOrigin().x();
        init_trans[1] = init_transform.getOrigin().y();
        init_trans[2] = init_transform.getOrigin().z();
        init_trans[3] = init_transform.getRotation().x();
        init_trans[4] = init_transform.getRotation().y();
        init_trans[5] = init_transform.getRotation().z();
        init_trans[6] = init_transform.getRotation().w();
        k_cal.quat2Dcm(init_trans, init_tool2base_mat);


        listener.lookupTransform("/tool0_controller"/*目标坐标系*/, "/camera"/*source*/,ros::Time(0), init_transform_cam2tool);
        // sleep(1);
        init_trans_cam2tool[0] = init_transform_cam2tool.getOrigin().x();
        init_trans_cam2tool[1] = init_transform_cam2tool.getOrigin().y();
        init_trans_cam2tool[2] = init_transform_cam2tool.getOrigin().z();
        init_trans_cam2tool[3] = init_transform_cam2tool.getRotation().x();
        init_trans_cam2tool[4] = init_transform_cam2tool.getRotation().y();
        init_trans_cam2tool[5] = init_transform_cam2tool.getRotation().z();
        init_trans_cam2tool[6] = init_transform_cam2tool.getRotation().w();
        k_cal.quat2Dcm(init_trans_cam2tool, init_cam2tool_mat);

        cout<<"init_tool2base.mat"<<endl;
        // CloudFit::print4x4Matrix(init_tool2base_mat);
        pcl::on_nurbs::NurbsDataSurface pca_data;
        printf ("  %lu cloud points in init_cloud set\n", init_cloud->size ());
        CloudFit::pointCloud2Vector3d (init_cloud, pca_data.interior);
        nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &pca_data);//在这里完成了nurbs的初始化
        // sleep(1);
        // init_flag=0;
        // ROS_INFO("pcaInit works/n");
      }
     // Convert to ROS data type
      pcl::PCLPointCloud2 cloud_2_filtered;
      pcl::toPCLPointCloud2 (*init_tool_cloud, cloud_2_filtered);
      sensor_msgs::PointCloud2 dianyun;
      pcl_conversions::fromPCL(cloud_2_filtered, dianyun);

      // Publish the data
      pub3_.publish (dianyun);

  };

  void print4x4Matrix (const Eigen::Matrix4f & matrix)
  {
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
    printf ("R = | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
  }
 
  void cloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    PointCloudT::Ptr cloud_cam (new PointCloudT);
    PointCloudT::Ptr cloud_init (new PointCloudT);
    pcl::on_nurbs::NurbsDataSurface data;//pca的数据
    pcl::on_nurbs::FittingSurface::Parameter params;//参数必须放在这个函数的内部
    params.interior_smoothness = 0.2;
    params.interior_weight = 10;
    params.boundary_smoothness = 0.0005;
    params.boundary_weight = 0.01;

     try
      {   
        listener.lookupTransform("/tool0_controller"/*target_frame*/, "/camera"/*source_frame*/,\
                                                     ros::Time(0), transform);
        listener.lookupTransform("/base"/*目标坐标系*/, "/tool0_controller"/*source_frame*/,\
                                                     ros::Time(0), transform2);
        //要定义一个中间数数组变量，把transform转化
        cam2tool_quan[0] = transform.getOrigin().x();
        cam2tool_quan[1] = transform.getOrigin().y();
        cam2tool_quan[2] = transform.getOrigin().z();
        cam2tool_quan[3] = transform.getRotation().x();
        cam2tool_quan[4] = transform.getRotation().y();
        cam2tool_quan[5] = transform.getRotation().z();
        cam2tool_quan[6] = transform.getRotation().w();
        k_cal.quat2Dcm(cam2tool_quan, cam2tool_mat);
        
        tool2base_quan[0] = transform2.getOrigin().x();
        tool2base_quan[1] = transform2.getOrigin().y();
        tool2base_quan[2] = transform2.getOrigin().z();
        tool2base_quan[3] = transform2.getRotation().x();
        tool2base_quan[4] = transform2.getRotation().y();
        tool2base_quan[5] = transform2.getRotation().z();
        tool2base_quan[6] = transform2.getRotation().w();//机械臂目标坐标系的四元数
        k_cal.quat2Dcm(tool2base_quan, curr_tool2base_mat);
        CloudFit::print4x4Matrix(curr_tool2base_mat);

        tool2cam_mat = cam2tool_mat.inverse();
      }
      catch (tf::TransformException ex)
      {
         ROS_ERROR("%s",ex.what());
      }

    // curr2init_mat=init_tool2base_mat*(curr_tool2base_mat.inverse());
    // init2curr_mat=curr2init_mat.inverse();

    // Matrix4f iiii = init_cam2tool_mat*init2curr_mat;
    // cout<<"cam to curr tool"<<endl;
    // // CloudFit::print4x4Matrix(iiii);
    //  cout<<"init cam to tool * init tool to curr tool"<<endl;
    // CloudFit::print4x4Matrix(cam2tool_mat);

    // pcl::on_nurbs::NurbsDataSurface pca_data;
    // printf ("  %lu cloud points in init_cloud set\n", init_cloud->size ());
    // CloudFit::pointCloud2Vector3d (init_cloud, pca_data.interior);
    // ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &pca_data);//在这里完成了nurbs的初始化
  
    pcl::fromROSMsg (*cloud_msg, *cloud_cam);
    //这里要添加一步转化为tool0坐标系下的点云
    pcl::transformPointCloud (*cloud_cam/*in*/, *cloud_init/*out*/, curr2init_mat);
    CloudFit::pointCloud2Vector3d (cloud_init, data.interior);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);
    
    for (unsigned ame = 0; ame < iterations_fit; ++ame)
    {
      fit.assemble (params);
      fit.solve ();
    }

    tra_point[0].x=0.762; tra_point[0].y=-0.056; tra_point[0].z=0.058;
    sor_point = fit.m_nurbs.PointAt(0,0.5);
    double z_distance = sor_point.z-tra_point[0].z;

    if(z_distance<0)
    {cout<<"z_distance: "<<z_distance<<endl;
      tool2base_quan[2] += (sor_point.z)/50;
      std::vector<float> array2(tool2base_quan,tool2base_quan+6);
      move_output.data = array2;
      cout<<array2[0]<<endl;
      cout<<array2[1]<<endl;
      cout<<array2[2]<<endl;
      cout<<array2[3]<<endl;
      cout<<array2[4]<<endl;
      cout<<array2[5]<<endl;
      cout<<array2[6]<<endl;
      pub2_.publish (move_output);
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
        my_point = fit.m_nurbs.PointAt(u,v);
        // my_point = nurbs.PointAt(u,v);
        init_paper_point[0] = my_point.x;
        init_paper_point[1] = my_point.y;
        init_paper_point[2] = my_point.z;
        init_paper_point[3] = 1.0;
        // cout<<init_paper_point[0]<<endl;
        // cout<<init_paper_point[1]<<endl;
        // cout<<init_paper_point[2]<<endl;
        curr_paper_point = curr2init_mat*init_paper_point;//转换到相机坐标系下
        array[k]   = curr_paper_point[0];
        array[k+1] = curr_paper_point[1];
        array[k+2] = curr_paper_point[2];
        k += 3;
        v += (double)1/NUB_V;
      }
      u += (double)1/NUB_U;
      v=0.0;
    }
    std::vector<float> array1(array,array+600);
  
    visual_output.data = array1;

    // nurbs_2.Destroy();//依然不好改善

    ON_3dPoint par_end = fit.m_nurbs.PointAt(0.5,0.5);
    // ON_3dPoint par_end = nurbs.PointAt(0.5,0.5);
    // ON_3dPoint par_end2 = nurbs_2.PointAt(0.5,0.5);
    parper2tool_mat<<1,0,0,par_end.x,
                    0,1,0,par_end.y,
                    0,0,1,par_end.z,
                    0,0,0,1;
    // cout<<"纸片上的0.5/0.5：：：："<<endl;
    // CloudFit::print4x4Matrix(parper2tool_mat);
  
    // parper2base_mat=cam2base_mat*tool2cam_mat*parper2cam_mat;
    // CloudFit::print4x4Matrix(parper2base_mat);
    pub1_.publish (visual_output);
    // pca_data.clear_interior();
    // ROS_INFO("cloudCb works/n");
  };
  
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "ur_mesh");
  // ros::NodeHandle nh;
  CloudFit try1;
  
  ros::spin ();
  return 0;
}