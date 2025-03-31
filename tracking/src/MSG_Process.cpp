#include "MSG_Process.hpp"
#include "math.h"
MsgProcessNode::MsgProcessNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node) :
    tf_subscriber_node_(tf_subscriber_node){
  // 初始化订阅者
  ros::NodeHandle nh;
  A_Star_Traj_Subscriber_ = nh.subscribe("/A_Star_Planned_Path", 10, &MsgProcessNode::AStarTrajectoryCallback, this);
  // 初始化地图订阅者
  Map_Subscriber_ = nh.subscribe("/A_Star_Map_Relaxed", 10, &MsgProcessNode::MapCallback, this);
  // 初始化发布者
  // Tracking_Traj_Publisher_ = nh.advertise<nav_msgs::Path>("/Tracking_Trajectory", 10);

  config_yaml_path = "/root/PersonalData/Program/jackal_ws/src/tracking/config/config.yaml";
}


void MsgProcessNode::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
  // 保存地图
  grid_map = *map_msg;

  int width = grid_map.info.width;
  int height = grid_map.info.height;

  // 创建二值地图（0=free，255=occupied）
  cv::Mat binary_map(height, width, CV_8UC1);
  for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
          int index = y * width + x;
          int val = grid_map.data[index];
          binary_map.at<uchar>(y, x) = (val > 50) ? 255 : 0;
      }
  }

  // 计算距离图（以像素为单位）
  cv::Mat dist_pixel;
  cv::distanceTransform(255 - binary_map, dist_pixel, cv::DIST_L2, 5);

  // 转换为实际单位（米），乘以地图分辨率
  dist_map = dist_pixel * grid_map.info.resolution;
}


void MsgProcessNode::AStarTrajectoryCallback(const astar_msgs::AStarPathArray::ConstPtr & astar_path)
{
  Eigen::Matrix4Xd control_points_odom = Eigen::Matrix4Xd::Zero(4, astar_path->paths.size());
  // 从A*路径消息中获取路径点
  for (int i = 0; i < astar_path->paths.size(); i++) {
    Eigen::Vector4d position = Eigen::Vector4d(
        astar_path->paths[i].position.x,
        astar_path->paths[i].position.y,
        astar_path->paths[i].position.z,
        1.0
    );
    control_points_odom.col(i) = position;
  }
  control_points = control_points_odom.block(0, 0, 3, control_points_odom.cols());
  rush_sign = astar_path->rush_sign;
  // nav_msgs::Path tracking_path;
  // tracking_path.header.frame_id = "odom";
  // tracking_path.header.stamp = astar_path->header.stamp;
  // for (int i = 0; i < control_points.cols(); i++) {
  //   geometry_msgs::PoseStamped pose;
  //   pose.pose.position.x = control_points(0, i);
  //   pose.pose.position.y = control_points(1, i);
  //   pose.pose.position.z = 0;

  //   double yaw = control_points(2, i);
  //   tf::Quaternion q;
  //   q.setRPY(0, 0, yaw);
  //   pose.pose.orientation.x = q.x();
  //   pose.pose.orientation.y = q.y();
  //   pose.pose.orientation.z = q.z();
  //   pose.pose.orientation.w = q.w();
  //   tracking_path.poses.push_back(pose);
  // }
  // Tracking_Traj_Publisher_.publish(tracking_path);

}