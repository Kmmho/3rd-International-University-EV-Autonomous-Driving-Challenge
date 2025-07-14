#include "lidar_header.h"

// ROI 파라미터
#define ROI_Z 1
#define ROI_X 60
#define ROI_Y 13



ros::Publisher pub1; // voxel
ros::Publisher pub2; // land
ros::Publisher pub3; // obs


// ros::Publisher pub2; // noise
// ros::Publisher pub3; // ransac
// ros::Publisher pub4; // dbscan

pcl::PassThrough<pcl::PointXYZI> pass;

double roll, pitch, yaw;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_orin(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud_orin);


    //IMU 회전
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())); // Pitch
    transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));  // Roll
    pcl::transformPointCloud(*cloud_orin, *cloud, transform);

    // ROS_INFO("Roll: [%f], Pitch: [%f], Yaw: [%f]", roll, pitch, yaw);

    

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_obs(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_land(new pcl::PointCloud<pcl::PointXYZI>);

    // voxel
    // 1. ROI 설정
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-2, 2);
    // pass.setFilterLimits (-0.2, 1.0); //down
    pass.filter (*cloud_out);

    // 앞-뒤
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0, 60);  
    pass.setInputCloud (cloud_out);
    pass.filter (*cloud_out);

    // 좌-우
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-10, 10); 
    pass.setInputCloud (cloud_out);
    pass.filter (*cloud_out);


    // 2. Voxel화
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr cloud_voxel(new pcl::PCLPointCloud2);

    pcl::PCLPointCloud2::Ptr cloud_out_PCL2(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud_out, *cloud_out_PCL2);
    
    sor.setInputCloud (cloud_out_PCL2);
    sor.setLeafSize (0.2f, 0.2f, 0.2f); // voxel 크기 설정, leaf size 0.2cm

    sor.filter(*cloud_voxel);

    sensor_msgs::PointCloud2 output1;
    pcl_conversions::fromPCL(*cloud_voxel, output1);
    output1.header.frame_id = input->header.frame_id;
    output1.header.stamp = ros::Time::now();
    pub1.publish(*cloud_out);
    
    //-----------------------------------------------------------------------------------------
    // land
    // 1. ROI 설정
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.6, -1.2);
    // pass.setFilterLimits (-0.8, -0.6); // down
    pass.filter (*cloud_out_land);

    // 앞-뒤
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-1.0, ROI_X);  
    pass.setInputCloud (cloud_out_land);
    pass.filter (*cloud_out_land);

    // 좌-우
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-ROI_Y, ROI_Y); 
    pass.setInputCloud (cloud_out_land);
    pass.filter (*cloud_out_land);


    // 2. Voxel화
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor2;
    pcl::PCLPointCloud2::Ptr cloud_voxel_land(new pcl::PCLPointCloud2);

    pcl::PCLPointCloud2::Ptr cloud_out_PCL2_land(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud_out_land, *cloud_out_PCL2_land);
    
    sor2.setInputCloud (cloud_out_PCL2_land);
    sor2.setLeafSize (0.1f, 0.1f, 0.1f); // voxel 크기 설정, leaf size 0.2cm

    sor2.filter(*cloud_voxel_land);

    sensor_msgs::PointCloud2 output2;
    pcl_conversions::fromPCL(*cloud_voxel_land, output2);
    output2.header.frame_id = input->header.frame_id;
    output2.header.stamp = ros::Time::now();
    pub2.publish(output2);

    //-----------------------------------------------------------------------------------------
    // obs
    // 1. ROI 설정
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-ROI_Z, 0);
    // pass.setFilterLimits (-0.2, 1.0); //down
    pass.filter (*cloud_out_obs);

    // 앞-뒤
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-2.0, ROI_X);  
    pass.setInputCloud (cloud_out_obs);
    pass.filter (*cloud_out_obs);

    // 좌-우
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-ROI_Y, ROI_Y); 
    pass.setInputCloud (cloud_out_obs);
    pass.filter (*cloud_out_obs);


    // 2. Voxel화
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor3;
    pcl::PCLPointCloud2::Ptr cloud_voxel_obs(new pcl::PCLPointCloud2);

    pcl::PCLPointCloud2::Ptr cloud_out_PCL2_obs(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud_out_obs, *cloud_out_PCL2_obs);
    
    sor.setInputCloud (cloud_out_PCL2_obs);
    sor.setLeafSize (0.2f, 0.2f, 0.2f); // voxel 크기 설정, leaf size 0.2cm

    sor.filter(*cloud_voxel_obs);

    sensor_msgs::PointCloud2 output3;
    pcl_conversions::fromPCL(*cloud_voxel_obs, output3);
    output3.header.frame_id = input->header.frame_id;
    output3.header.stamp = ros::Time::now();
    pub3.publish(*cloud_out_obs);

    
}



void imu_cb (const sensor_msgs::ImuConstPtr& msg)
{
  tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  //ROS_INFO("Roll: [%f], Pitch: [%f], Yaw: [%f]", roll, pitch, yaw);
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar_pre");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
  // ros::Subscriber sub2 = nh.subscribe ("/imu", 1, imu_cb);
  
  // ros::topic::waitForMessage<sensor_msgs::Imu>("/imu", nh);
  ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/velodyne_points", nh);
  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("voxel", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("land", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("obs", 1);
  // pub3 = nh.advertise<sensor_msgs::PointCloud2> ("ransac", 1);


  // Spin
  ros::spin ();
}