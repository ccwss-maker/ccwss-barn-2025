#include "MSG_Process.hpp"
#include "math.h"
MsgProcessNode::MsgProcessNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node) :
    tf_subscriber_node_(tf_subscriber_node){
  // 初始化订阅者
  ros::NodeHandle nh;
//   A_Star_Traj_Subscriber_ = nh.subscribe("/A_Star_Planned_Path", 10, &MsgProcessNode::AStarTrajectoryCallback, this);
  First_Opti_Traj_Subscriber_ = nh.subscribe("/First_Optimized_Trajectory", 10, &MsgProcessNode::FirstOptiTrajectoryCallback, this);
  // 初始化地图订阅者
  Map_Subscriber_ = nh.subscribe("/A_Star_Map_Origin", 10, &MsgProcessNode::MapCallback, this);
  // 初始化发布者
  // Tracking_Traj_Publisher_ = nh.advertise<nav_msgs::Path>("/Tracking_Trajectory", 10);

  Tracking_Traj_Maker_Publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/MPC/Origin_Reference_Traj", 1);
  config_yaml_path = "/jackal_ws/src/tracking/config/config.yaml";

  rush_sign = false;
}

// -------------------- 工具函数 -------------------- //
template <typename T>
T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
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
  YAML::Node config = YAML::LoadFile(config_yaml_path);
  Eigen::Matrix4Xd control_points_odom = Eigen::Matrix4Xd::Zero(4, astar_path->paths.size());

  // 从A*路径消息中获取路径点 + theta
  Eigen::MatrixXd raw_points(3, astar_path->paths.size());
  for (int i = 0; i < astar_path->paths.size(); i++) {
      raw_points(0, i) = astar_path->paths[i].position.x;
      raw_points(1, i) = astar_path->paths[i].position.y;
      raw_points(2, i) = astar_path->paths[i].position.z;  // 直接用你传入的 theta
  }

  // 曲率密度调节
  std::vector<Eigen::Vector3d> new_points;
  double min_spacing = config["resample"]["min_spacing"].as<double>();
  double max_spacing = config["resample"]["max_spacing"].as<double>();

  for (int i = 1; i < raw_points.cols() - 1; ++i) {
    double dtheta1 = std::atan2(std::sin(raw_points(2, i) - raw_points(2, i - 1)),
                                std::cos(raw_points(2, i) - raw_points(2, i - 1)));
    double dtheta2 = std::atan2(std::sin(raw_points(2, i + 1) - raw_points(2, i)),
                                std::cos(raw_points(2, i + 1) - raw_points(2, i)));
    double curvature = std::abs(dtheta2 - dtheta1);
    double spacing = max_spacing - curvature * (max_spacing - min_spacing);
    spacing = clamp(spacing, min_spacing, max_spacing);

    if (new_points.empty()) {
        new_points.push_back(raw_points.col(i));
    } else {
        Eigen::Vector2d last_xy(new_points.back()(0), new_points.back()(1));
        Eigen::Vector2d curr_xy(raw_points(0, i), raw_points(1, i));
        if ((curr_xy - last_xy).norm() >= spacing) {
            new_points.push_back(raw_points.col(i));
        }
    }
  }
  new_points.push_back(raw_points.col(raw_points.cols() - 1));

  control_points = Eigen::MatrixXd(3, new_points.size());
  for (int i = 0; i < new_points.size(); ++i) {
      control_points.col(i) = new_points[i];
  }

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

  // ------------------- 参考轨迹发布 ------------------- //
  visualization_msgs::MarkerArray marker_array;
  int ID = 0;
  for (int i = 0; i < control_points.cols(); ++i) {
      visualization_msgs::Marker point_marker;
      point_marker.header.frame_id = "odom";
      point_marker.header.stamp = ros::Time::now();
      point_marker.ns = "origin_trajectory";
      point_marker.type = visualization_msgs::Marker::SPHERE;
      point_marker.action = visualization_msgs::Marker::ADD;

      // 为每个点设置唯一的ID
      point_marker.id = ID++;  // 为每个点分配唯一的ID

      // 设置Marker的比例
      point_marker.scale.x = 0.03;  // 点的大小
      point_marker.scale.y = 0.03;
      point_marker.scale.z = 0.03;

      // 设置Marker的颜色
      point_marker.color.r = 0.0;  // 红色分量
      point_marker.color.g = 0.0;  // 绿色分量
      point_marker.color.b = 1.0;  // 蓝色分量
      point_marker.color.a = 1.0;  // 透明度（alpha）

      // 设置点的坐标
      point_marker.pose.position.x = control_points(0, i);
      point_marker.pose.position.y = control_points(1, i);
      point_marker.pose.position.z = 0.3;
      point_marker.pose.orientation.x = 0.0;
      point_marker.pose.orientation.y = 0.0;
      point_marker.pose.orientation.z = 0.0;
      point_marker.pose.orientation.w = 1.0;
      // 将Marker添加到MarkerArray
      marker_array.markers.push_back(point_marker);
  }
  Tracking_Traj_Maker_Publisher_.publish(marker_array);


}


void MsgProcessNode::FirstOptiTrajectoryCallback(const initial_optimized_msgs::InitialOptimizedTrajectory::ConstPtr& first_opti_path)
{

    rush_sign = first_opti_path->rush_sign;
    if(first_opti_path->emergency_braking_sign)
    {
        control_points = Eigen::MatrixXd::Zero(3, 0);
        return;
    }
    YAML::Node config = YAML::LoadFile(config_yaml_path);
    minco::MINCO_S3NU minco;
    double pieceN = first_opti_path->position.size() - 2;
    Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();
    initState.col(0) << first_opti_path->position.front().x,
                        first_opti_path->position.front().y,    
                        first_opti_path->position.front().z;
    finalState.col(0) << first_opti_path->position.back().x,
                         first_opti_path->position.back().y,
                         first_opti_path->position.back().z;
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, first_opti_path->position.size() - 2);
    for (int i = 1; i < first_opti_path->position.size() - 2; i++) {
        points.col(i) << first_opti_path->position[i].x,
                         first_opti_path->position[i].y,
                         first_opti_path->position[i].z;
    }
    Eigen::VectorXd times = Eigen::VectorXd::Zero(first_opti_path->times.size());
    for (int i = 0; i < first_opti_path->times.size(); i++) {
        times(i) = first_opti_path->times[i];
    }
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);

    double T_total = times.sum();
    double T = config["T"].as<double>();
    int step = (int)(T_total / T);

    int skip_front_points = config["skip_front_points"].as<int>();
    int start_index = 0;
    double front_sum = 0;
    while (start_index < step && front_sum < times.head(skip_front_points).sum()) {
        front_sum = start_index * T;
        start_index++;
    }
    control_points = Eigen::MatrixXd::Zero(3, step - start_index);
    for(int i=start_index; i<step; i++)
    {
        int current_segment;
        double local_t;
        double t = i * T;
        findCurrentSegmentAndLocalTime(t, current_segment, local_t, pieceN, times);
        double t0 = 1;
        double t1 = local_t;
        double t2 = t1 * t1;
        double t3 = t1 * t2;
        double t4 = t1 * t3;
        double t5 = t1 * t4;
        Eigen::VectorXd T_Position(6);
        T_Position << t0, t1, t2, t3, t4, t5;
        Eigen::VectorXd T_Velocity(6);
        T_Velocity << 0, 1, 2 * t1, 3 * t2, 4 * t3, 5 * t4;
        Eigen::MatrixXd b = minco.b.block(current_segment * 6, 0, 6, 3).transpose();
        Eigen::Vector3d V = minco.b.block(current_segment * 6, 0, 6, 3).transpose() * T_Velocity;  // 速度向量
        Eigen::Vector3d point = b * T_Position;
        point(2) = std::atan2(V(1), V(0));
        control_points.col(i - start_index) = point;
    }

    // ------------------- 参考轨迹发布 ------------------- //
    visualization_msgs::MarkerArray marker_array;
    // 清除之前的Marker
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    int ID = 0;
    for (int i = 0; i < control_points.cols(); i++) {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "odom";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "origin_trajectory";
        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;

        // 为每个点设置唯一的ID
        point_marker.id = ID++;  // 为每个点分配唯一的ID

        // 设置Marker的比例
        point_marker.scale.x = 0.03;  // 点的大小
        point_marker.scale.y = 0.03;
        point_marker.scale.z = 0.03;

        // 设置Marker的颜色
        point_marker.color.r = 0.0;  // 红色分量
        point_marker.color.g = 0.0;  // 绿色分量
        point_marker.color.b = 1.0;  // 蓝色分量
        point_marker.color.a = 1.0;  // 透明度（alpha）

        // 设置点的坐标
        point_marker.pose.position.x = control_points(0, i);
        point_marker.pose.position.y = control_points(1, i);
        point_marker.pose.position.z = 0;
        point_marker.pose.orientation.x = 0.0;
        point_marker.pose.orientation.y = 0.0;
        point_marker.pose.orientation.z = 0.0;
        point_marker.pose.orientation.w = 1.0;
        // 将Marker添加到MarkerArray
        marker_array.markers.push_back(point_marker);
    }
    Tracking_Traj_Maker_Publisher_.publish(marker_array);
}



void MsgProcessNode::findCurrentSegmentAndLocalTime(double t, int & segment, double & local_t, int pieceN, Eigen::VectorXd times)
{
    int left = 0;
    int right = pieceN - 1;
    double total_time = 0.0;

    // 二分查找
    while (left <= right) {
        int mid = left + (right - left) / 2;
        double segment_end_time = 0.0;

        // 计算到mid段为止的总时间
        for (int i = 0; i <= mid; i++) {
            segment_end_time += times[i];
        }

        if (t < segment_end_time - times[mid]) {
            right = mid - 1;
        } else if (t > segment_end_time) {
            left = mid + 1;
        } else {
            segment = mid;
            total_time = segment_end_time - times[mid];
            break;
        }
    }

    // 如果时间超过了所有轨迹段的总时间，则返回最后一个段
    if (left > right) {
        segment = pieceN - 1;
        total_time = 0.0;
        for (int i = 0; i < pieceN - 1; i++) {
            total_time += times[i];
        }
    }
    // 计算局部时间（相对于当前轨迹段的时间）
    local_t = t - total_time;
}