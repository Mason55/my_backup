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
// unsigned refinement (3);
unsigned iterations_fit (20);
// unsigned iterations_icp (30);
// unsigned mesh_resolution (256);

// ON_NurbsSurface nurbs (3,false,order,order,order,order);
// ON_NurbsSurface nurbs_2 (3,false,order,order,order,order);

PointCloudT::Ptr init_cam_cloud (new PointCloudT);
PointCloudT::Ptr init_tmp_cloud (new PointCloudT);
PointCloudT::Ptr init_tool_cloud (new PointCloudT);

class CloudFit
{
  ros::NodeHandle nh_;
  // ros::NodeHandle nh2_;
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
 
public:
  CloudFit()
  {
    sub2_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::pcaInit, this);
    sub1_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::cloudCb, this);
    pub1_ = nh_.advertise<point_msgs::Point> ("/visual/bsfitting", 1);
    pub2_ = nh_.advertise<point_msgs::Point> ("/visual/bsfitting/2", 1);
    pub3_ = nh_.advertise<sensor_msgs::PointCloud2> ("dianyun", 1);
    cam2base_mat<<-0.0213  , -0.8264  ,  -0.5626  ,  1.1210,//这是把手眼标定的矩阵拿过来用了
                  -0.9997  ,  0.0264  ,  -0.0009  , -0.1194,
                   0.0156  ,  0.5624  ,  -0.8267  ,  1.0983,
                  -0.0000  , -0.0000  ,   0.0000  ,  1.0000;
    cam2tool_mat<< 1 , 0 , 0 , 0.2 ,
                   0 , 1 , 0 , 0 ,
                   0 , 0 , 1 , 0 ,
                   0 , 0 , 0 , 1;
    tool_point<<0.0,0.0,0.0,1.0;
    cam_point<<0.0,0.0,0.0,1.0;
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
      printf ("  %lu cloud points in init_cam_cloud set\n", init_cam_cloud->size ());
      cout<<"cloud_size="<<cloud_size<<endl;
      if(init_cam_cloud->size()>=cloud_size &&init_flag==1) 
      {
        // sub2_.shutdown();//问题不在订阅
        *init_tmp_cloud = *init_cam_cloud;
        printf ("  %lu cloud points in init_tmp_cloud set\n", init_tmp_cloud->size ());
        sleep(1);
        // copyPointCloud(*init_cam_cloud,*init_tmp_cloud);
        // copyPointCloud(init_tmp_cloud, init_cam_cloud->makeShared());
        init_flag=0;
        // ROS_INFO("pcaInit works/n");
      }
        // pcl::on_nurbs::NurbsDataSurface data_1;
        // CloudFit::pointCloud2Vector3d (init_tmp_cloud, data_1.interior);
        // ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_1);//在这里完成了nurbs的初始化
    //   printf ("  %lu cloud points in init_tmp_cloud set\n", init_tmp_cloud->size ());
    //   pcl::transformPointCloud (*init_tmp_cloud/*in*/, *init_tool_cloud/*out*/, cam2tool_mat);//这个函数需要搞清楚
    //   printf ("  %lu cloud points in init_tool_cloud set\n", init_tool_cloud->size ());

      // Convert to ROS data type

    //   pcl::PCLPointCloud2 cloud_2_filtered;
    //   pcl::toPCLPointCloud2 (*init_tool_cloud, cloud_2_filtered);
    //   sensor_msgs::PointCloud2 dianyun;
    //   pcl_conversions::fromPCL(cloud_2_filtered, dianyun);

    //   // Publish the data
    //   pub3_.publish (dianyun);

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
    PointCloudT::Ptr cloud_tool (new PointCloudT);
    pcl::on_nurbs::NurbsDataSurface data;//pca的数据
    pcl::on_nurbs::FittingSurface::Parameter params;//参数必须放在这个函数的内部
    params.interior_smoothness = 0.2;
    params.interior_weight = 10;
    params.boundary_smoothness = 0.0005;
    params.boundary_weight = 0.01;

     pcl::on_nurbs::NurbsDataSurface data_1;
        CloudFit::pointCloud2Vector3d (init_tmp_cloud, data_1.interior);
        ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_1);//在这里完成了nurbs的初始化
    
    pcl::transformPointCloud (*init_tmp_cloud/*in*/, *init_tool_cloud/*out*/, cam2tool_mat);//这个函数需要搞清楚
    
    pcl::on_nurbs::NurbsDataSurface data_2;
    CloudFit::pointCloud2Vector3d (init_tool_cloud, data_2.interior);
    ON_NurbsSurface nurbs_2 = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data_2);//在这里完成了nurbs的初始化
    // pca_data.clear_interior();
    cout<<"nurbs.SizeOf()"<<nurbs.SizeOf()<<endl;
    //       cout<<"dim:"<<nurbs.m_dim         <<endl; 
    //       cout<<"is_rat:"<<nurbs.m_is_rat      <<endl;
    //       cout<<"order[0]:"<<nurbs.m_order[0]    <<endl;
    //       cout<<"order[1]:"<<nurbs.m_order[1]    <<endl;
    //       cout<<"cv_count[0]:"<<nurbs.m_cv_count[0] <<endl;
    //       cout<<"cv_count[1]:"<<nurbs.m_cv_count[1] <<endl;
    //       cout<<"cv_stride[1]:"<<nurbs.m_cv_stride[1]<<endl;
    //       cout<<"cv_stride[0]:"<<nurbs.m_cv_stride[0]<<endl;
                {
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
                    // cam_point = tool2cam_mat*tool_point;//转换到相机坐标系下
                    array[k]   = tool_point[0];
                    array[k+1] = tool_point[1];
                    array[k+2] = tool_point[2];
                    k += 3;
                    v += (double)1/NUB_V;
                }
                u += (double)1/NUB_U;
                v=0.0;
                }
                std::vector<float> array1(array,array+600);
            
                visual_output.data = array1;
                pub1_.publish (visual_output);
            }


              {
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
                    my_point = nurbs_2.PointAt(u,v);
                    tool_point[0] = my_point.x;
                    tool_point[1] = my_point.y;
                    tool_point[2] = my_point.z;
                    tool_point[3] = 1.0;
                    // cout<<tool_point[0]<<endl;
                    // cout<<tool_point[1]<<endl;
                    // cout<<tool_point[2]<<endl;
                    // cam_point = tool2cam_mat*tool_point;//转换到相机坐标系下
                    array[k]   = tool_point[0];
                    array[k+1] = tool_point[1];
                    array[k+2] = tool_point[2];
                    k += 3;
                    v += (double)1/NUB_V;
                }
                u += (double)1/NUB_U;
                v=0.0;
                }
                std::vector<float> array1(array,array+600);
            
                visual_output.data = array1;
                pub1_.publish (visual_output);
            }
            nurbs.EmergencyDestroy();
             cout<<"afte destory nurbs.SizeOf()"<<nurbs.SizeOf()<<endl;
            nurbs_2.Destroy();
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