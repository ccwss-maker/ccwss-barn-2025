#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include "ellipse.hpp"
#include "DBSCAN.hpp"
#include "KM.hpp"

using namespace grid_map;
using namespace std;

GridMap map_({"local_lidar"});

pcl::PointCloud<pcl::PointXYZ> velodyne_cloud;
pcl::PointCloud<pcl::PointXYZ> velodyne_cloud_filter;
pcl::PointCloud<pcl::PointXYZ> velodyne_cloud_global;

pcl::PassThrough<pcl::PointXYZ> pass_x;
pcl::PassThrough<pcl::PointXYZ> pass_y;
pcl::VoxelGrid<pcl::PointXYZ> sor;

tf::TransformListener *listener_ptr = nullptr;
tf::StampedTransform base_link_transform;
tf::StampedTransform lidar_link_transform;
Eigen::Vector2d robot_position2d;

ros::Subscriber laser_cloud_sub;
ros::Publisher gridmap_pub;
ros::Publisher ellipse_vis_pub;
ros::Publisher local_pcd_pub;
ros::Publisher for_obs_track_pub;

KMAlgorithm KM;
float DBSCAN_R;
int DBSCAN_N;
int block_size;
int block_num;
float _inv_resolution;
float localmap_x_size, localmap_y_size, resolution;

void updateTF()
{
  while (true)
  {
    try
    {
      listener_ptr->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1));
      listener_ptr->lookupTransform("odom", "base_link", ros::Time(0), base_link_transform);
      listener_ptr->waitForTransform("odom", "front_laser", ros::Time(0), ros::Duration(0.1));
      listener_ptr->lookupTransform("odom", "front_laser", ros::Time(0), lidar_link_transform);
      break;
    }
    catch (tf::TransformException &ex)
    {
      ros::Duration(1.0).sleep();
      continue;
    }
  }
}

// void updateTF()
// {
//   while (true)
//   {
//     try
//     {
//       listener_ptr->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1));
//       listener_ptr->lookupTransform("odom", "base_link", ros::Time(0), base_link_transform);
//       listener_ptr->waitForTransform("odom", "front_laser", ros::Time(0), ros::Duration(0.1));
//       listener_ptr->lookupTransform("odom", "front_laser", ros::Time(0), lidar_link_transform);

//       // Print base_link TF
//       tf::Vector3 t1 = base_link_transform.getOrigin();
//       tf::Matrix3x3 rot1(base_link_transform.getRotation());
//       double roll1, pitch1, yaw1;
//       rot1.getRPY(roll1, pitch1, yaw1);
//       ROS_INFO_STREAM("[TF base_link] Translation: [" << t1.x() << ", " << t1.y() << ", " << t1.z() << "]");
//       ROS_INFO_STREAM("[TF base_link] Rotation (RPY): [" << roll1 << ", " << pitch1 << ", " << yaw1 << "]");

//       // Print front_laser TF
//       tf::Vector3 t2 = lidar_link_transform.getOrigin();
//       tf::Matrix3x3 rot2(lidar_link_transform.getRotation());
//       double roll2, pitch2, yaw2;
//       rot2.getRPY(roll2, pitch2, yaw2);
//       ROS_INFO_STREAM("[TF front_laser] Translation: [" << t2.x() << ", " << t2.y() << ", " << t2.z() << "]");
//       ROS_INFO_STREAM("[TF front_laser] Rotation (RPY): [" << roll2 << ", " << pitch2 << ", " << yaw2 << "]");

//       break;
//     }
//     catch (tf::TransformException &ex)
//     {
//       ROS_WARN_STREAM("[TF Error] " << ex.what());
//       ros::Duration(1.0).sleep();
//       continue;
//     }
//   }
// }

void pcd_transform()
{
  pass_x.setInputCloud(velodyne_cloud.makeShared());
  pass_x.filter(velodyne_cloud);

  pass_y.setInputCloud(velodyne_cloud.makeShared());
  pass_y.filter(velodyne_cloud);

  velodyne_cloud_filter.clear();
  sor.setInputCloud(velodyne_cloud.makeShared());
  sor.filter(velodyne_cloud_filter);

  Eigen::Affine3d affine_transform;

  tf::transformTFToEigen(lidar_link_transform, affine_transform);
  pcl::transformPointCloud(velodyne_cloud_filter, velodyne_cloud_global, affine_transform);
}

// void pcd_transform()
// {
//   // Step 1: Filter raw point cloud
//   pass_x.setInputCloud(velodyne_cloud.makeShared());
//   pass_x.filter(velodyne_cloud);

//   pass_y.setInputCloud(velodyne_cloud.makeShared());
//   pass_y.filter(velodyne_cloud);

//   velodyne_cloud_filter.clear();
//   sor.setInputCloud(velodyne_cloud.makeShared());
//   sor.filter(velodyne_cloud_filter);

//   // Step 2: Invert the transform from lidar_link to odom => get odom to lidar_link
//   tf::StampedTransform inverted_tf = lidar_link_transform.inverse();

//   // Step 3: Convert to Eigen and apply transform
//   Eigen::Affine3d affine_transform;
//   tf::transformTFToEigen(inverted_tf, affine_transform);

//   // Step 4: Apply the inverted transform to the filtered point cloud
//   pcl::transformPointCloud(velodyne_cloud_filter, velodyne_cloud_global, affine_transform);
// }

void lidar2gridmap(Eigen::MatrixXf &lidar_data_matrix)
{
  int col = lidar_data_matrix.cols();
  int row = lidar_data_matrix.rows();
  for (const auto &pt : velodyne_cloud_global)
  {
    int j = (pt.x - robot_position2d.x()) * _inv_resolution + col * 0.5;
    j = min(max(j, 0), row - 1);
    int k = (pt.y - robot_position2d.y()) * _inv_resolution + row * 0.5;
    k = min(max(k, 0), col - 1);

    lidar_data_matrix(j,k) = 1.0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_map_pub");
  ros::NodeHandle nh("~");

  gridmap_pub = nh.advertise<grid_map_msgs::GridMap>("gridmap", 1, true);
  local_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("local_pcd", 1);
  ellipse_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("ellipse_vis", 1);
  for_obs_track_pub = nh.advertise<std_msgs::Float32MultiArray>("for_obs_track", 1);

  laser_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/front/laser/cloud", 1, [&](sensor_msgs::PointCloud2::ConstPtr msg)
  {
    pcl::fromROSMsg(*msg, velodyne_cloud);
    updateTF();
  });

  nh.param<float>("localmap_x_size", localmap_x_size, 10);
  nh.param<float>("localmap_y_size", localmap_y_size, 10);
  nh.param<float>("resolution", resolution, 0.1);
  nh.param<float>("DBSCAN_R", DBSCAN_R, 5.0);
  nh.param<int>("DBSCAN_N", DBSCAN_N, 5);
  nh.param<int>("block_size", block_size, localmap_x_size * _inv_resolution * 0.2);
  nh.param<int>("block_num", block_num, 5);

  _inv_resolution = 1 / resolution;
  int map_index_len = localmap_x_size * _inv_resolution;

  tf::TransformListener listener;
  listener_ptr = &listener;

  map_.setFrameId("odom");
  map_.setGeometry(Length(localmap_x_size, localmap_y_size), resolution);
  Eigen::MatrixXf lidar_pcd_matrix(map_index_len, map_index_len);

  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-localmap_x_size / 2, localmap_x_size / 2);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-localmap_y_size / 2, localmap_y_size / 2);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);

  ros::Rate rate(11);
  while (ros::ok())
  {
    robot_position2d << base_link_transform.getOrigin().x(), base_link_transform.getOrigin().y();
    pcd_transform();
    map_.setPosition(robot_position2d);
    map_.clear("local_lidar");
    lidar_pcd_matrix = map_.get("local_lidar");
    lidar2gridmap(lidar_pcd_matrix);
    map_.add("local_lidar", lidar_pcd_matrix);

    vector<DBSCAN::Point> non_clustered_obs;
    for (int i = 0; i < lidar_pcd_matrix.rows(); ++i)
    {
      for (int j = 0; j < lidar_pcd_matrix.cols(); ++j)
      {
        if (lidar_pcd_matrix(i, j) == 1.0)
        {
          DBSCAN::Point pt;
          pt.x = i;
          pt.y = j;
          non_clustered_obs.push_back(pt);
        }
      }
    }

    DBSCAN DS(DBSCAN_R, DBSCAN_N, non_clustered_obs);
    vector<Obstacle> clustered_obs(DS.cluster_num);
    for (const auto &obs : non_clustered_obs)
    {
      if (obs.obsID > 0)
        clustered_obs[obs.obsID - 1].emplace_back(obs.x, obs.y);
    }

    vector<Ellipse> ellipses_array = get_ellipse_array(clustered_obs, map_, localmap_x_size=localmap_x_size,localmap_y_size=localmap_y_size,resolution=resolution,_inv_resolution=_inv_resolution);
    KM.tracking(ellipses_array);
    ab_variance_calculation(ellipses_array);

    std_msgs::Float32MultiArray for_obs_track;
    for (const auto &ellipse : ellipses_array)
    {
      if (ellipse.label == 0) continue;
      for_obs_track.data.push_back(ellipse.cx);
      for_obs_track.data.push_back(ellipse.cy);
      for_obs_track.data.push_back(ellipse.semimajor);
      for_obs_track.data.push_back(ellipse.semiminor);
      for_obs_track.data.push_back(ellipse.theta);
      for_obs_track.data.push_back(ellipse.label);
      for_obs_track.data.push_back(ellipse.variance);
    }
    for_obs_track_pub.publish(for_obs_track);

    sensor_msgs::PointCloud2 local_velodyne_msg;
    pcl::toROSMsg(velodyne_cloud_global, local_velodyne_msg);
    local_velodyne_msg.header.stamp = ros::Time::now();
    local_velodyne_msg.header.frame_id = "odom";
    local_pcd_pub.publish(local_velodyne_msg);

    grid_map_msgs::GridMap gridMapMessage;
    grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
    gridmap_pub.publish(gridMapMessage);

    visEllipse(ellipses_array, ellipse_vis_pub);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
