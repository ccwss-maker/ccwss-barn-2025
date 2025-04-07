#include "Mapping.hpp"
#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>

// MappingNode implementation
MappingNode::MappingNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node) :
    tf_subscriber_node_(tf_subscriber_node),
    // 使用更小的初始大小初始化地图
    origin_map(0.1, 100),
    relaxed_grid_map(0.1, 100),
    grid_map(0.1, 100)
{
    ros::NodeHandle nh;
    config_yaml_path = "/jackal_ws/src/navigation/config/config.yaml";

    // 初始化ROS订阅器和发布器
    scan_sub_ = nh.subscribe("/front/scan", 1, &MappingNode::scanCallback, this);
    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MappingNode::goalCallback, this);

    // 初始化动作服务器
    action_server_ = std::make_unique<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>>(
        nh,
        "move_base",
        boost::bind(&MappingNode::executeGoalCallback, this, _1),
        false
    );
    action_server_->start();
    ROS_INFO("MoveBase ActionServer started");

    // 初始化发布器
    goal_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker_goal", 10);
    map_origin_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map_Origin", 10);
    map_relaxed_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map_Relaxed", 10);
    map_safety_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map", 10);
    ROS_INFO("Mapping node initialized and started");

    // 初始化标志
    ready = false;

    // 初始化地图大小
    map_size.xmin = vehicle_pose.x;
    map_size.xmax = vehicle_pose.x;
    map_size.ymin = vehicle_pose.y;
    map_size.ymax = vehicle_pose.y;

    // 记录上一个路径原点
    last_path_origin_xmin_ = map_size.xmin;
    last_path_origin_ymin_ = map_size.ymin;

    // 加载配置
    config = YAML::LoadFile(config_yaml_path);
    
    // 设置所有地图的分辨率和原点 - 原点固定为(0,0)
    double initial_resolution = config["resolution_meters"].as<double>();
    origin_map.setResolution(initial_resolution);
    origin_map.setOrigin(0.0, 0.0);
    
    relaxed_grid_map.setResolution(initial_resolution);
    relaxed_grid_map.setOrigin(0.0, 0.0);
    
    grid_map.setResolution(initial_resolution);
    grid_map.setOrigin(0.0, 0.0);
    
    // ROS_INFO("Map origin fixed at (0,0), initial map size is 100x100");
}

void MappingNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    if (!goal_state_.have_goal_) {
        ROS_DEBUG("No goal set, ignoring scan");
        return;
    }

    Matrix Laser_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "front_laser");
    Matrix Base_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "base_link");
    Eigen::Vector3d Base_To_Odom_Translation = Base_To_Odom_Matrix.Translation_Read();
    double roll, pitch, yaw;
    tf::Quaternion transform_q = Base_To_Odom_Matrix.Quaternion_Read();
    tf::Matrix3x3(transform_q).getRPY(roll, pitch, yaw);

    // 设置车辆初始位姿
    vehicle_pose.x = Base_To_Odom_Translation.x();
    vehicle_pose.y = Base_To_Odom_Translation.y();
    vehicle_pose.yaw = yaw;

    config = YAML::LoadFile(config_yaml_path);

    // 收集所有有效的障碍点并且检查是否需要"rush"
    bool rush_sign_find = true;
    std::vector<Eigen::Vector2d> scan_obstacles;
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        if (std::isfinite(range)) {
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            Eigen::Vector4d p_laser(x, y, 0.0, 1.0);
            if(x > config["rush_distance"].as<double>()) rush_sign_find = false;
            Eigen::Vector4d p_odom = Laser_To_Odom_Matrix.Rotation_Translation_Read() * p_laser;
            scan_obstacles.emplace_back(p_odom.x(), p_odom.y());
        }
    }

    if (scan_obstacles.empty()) {
        ROS_WARN_THROTTLE(5.0, "No valid laser points found");
        return;
    }

    if(rush_sign_find) {
        rush_sign = true;
        ROS_WARN_THROTTLE(5.0, "Rush sign found, ignoring scan");
        return;
    }

    // 获取起点和终点
    Eigen::Vector2d start_point(vehicle_pose.x, vehicle_pose.y);
    Eigen::Vector2d goal_point(goal_state_.current_goal_.pose.position.x, 
                              goal_state_.current_goal_.pose.position.y);
                              
    // 获取所有点的网格坐标边界
    int min_grid_x = std::numeric_limits<int>::max();
    int min_grid_y = std::numeric_limits<int>::max();
    int max_grid_x = std::numeric_limits<int>::min();
    int max_grid_y = std::numeric_limits<int>::min();
    
    // 检查起点和终点
    auto [start_grid_x, start_grid_y] = origin_map.worldToGrid(start_point.x(), start_point.y());
    auto [goal_grid_x, goal_grid_y] = origin_map.worldToGrid(goal_point.x(), goal_point.y());
    
    min_grid_x = std::min({min_grid_x, start_grid_x, goal_grid_x});
    min_grid_y = std::min({min_grid_y, start_grid_y, goal_grid_y});
    max_grid_x = std::max({max_grid_x, start_grid_x, goal_grid_x});
    max_grid_y = std::max({max_grid_y, start_grid_y, goal_grid_y});
    
    // 检查所有障碍点
    for (const auto& pt : scan_obstacles) {
        auto [grid_x, grid_y] = origin_map.worldToGrid(pt.x(), pt.y());
        min_grid_x = std::min(min_grid_x, grid_x);
        min_grid_y = std::min(min_grid_y, grid_y);
        max_grid_x = std::max(max_grid_x, grid_x);
        max_grid_y = std::max(max_grid_y, grid_y);
    }
    
    // 获取当前地图边界
    int map_min_x, map_min_y, map_max_x, map_max_y;
    origin_map.getMapBounds(map_min_x, map_min_y, map_max_x, map_max_y);
    
    // 检查地图是否需要调整大小
    if (min_grid_x < map_min_x || min_grid_y < map_min_y || 
        max_grid_x > map_max_x || max_grid_y > map_max_y) {
        // 需要调整地图大小
        // ROS_INFO("Map resize needed: Current (%d,%d)-(%d,%d), Required (%d,%d)-(%d,%d)",
        //          map_min_x, map_min_y, map_max_x, map_max_y,
        //          min_grid_x, min_grid_y, max_grid_x, max_grid_y);
        
        // 调整地图大小，确保包含所有点
        origin_map.resizeMap(min_grid_x, min_grid_y, max_grid_x, max_grid_y);
    }
    
    // 准备障碍物矩阵
    Eigen::MatrixXd obstacle_matrix(2, scan_obstacles.size());
    for (size_t i = 0; i < scan_obstacles.size(); ++i) {
        obstacle_matrix.col(i) = scan_obstacles[i];
    }
    
    // 更新原始地图，标记障碍物
    origin_map.updateMap(obstacle_matrix);
    
    // 清除起点和终点
    origin_map.setCellValue(vehicle_pose.x, vehicle_pose.y, 0);
    origin_map.setCellValue(goal_state_.current_goal_.pose.position.x, 
                           goal_state_.current_goal_.pose.position.y, 0);
    
    // 计算起点和终点网格坐标（可能因为地图调整而改变）
    std::pair<int, int> start = origin_map.worldToGrid(vehicle_pose.x, vehicle_pose.y);
    std::pair<int, int> goal = origin_map.worldToGrid(
        goal_state_.current_goal_.pose.position.x,
        goal_state_.current_goal_.pose.position.y);
    
    // 准备膨胀地图参数
    grid_resolution_meters = config["resolution_meters"].as<double>();
    car_length = config["Car_Length"].as<double>();
    car_width = config["Car_Width"].as<double>();
    safety_hor = config["Safety_Hor"].as<double>();

    // 创建新的膨胀地图，保持与原始地图相同的参数
    relaxed_grid_map = origin_map;  // 复制原始地图
    grid_map = origin_map;          // 复制原始地图
    
    // 计算膨胀半径
    double relaxed_radius = std::max(car_length / 2.0, car_width / 2.0) + 0.05;
    double full_radius = std::max(car_length / 2.0, car_width / 2.0) + safety_hor;

    // 执行膨胀 - 直接在各自的地图上操作，不需要传入原始地图
    relaxed_grid_map.boundedInflation(relaxed_radius);
    grid_map.boundedInflation(full_radius);

    // 清除膨胀地图上的起点和终点
    relaxed_grid_map.setCellValue(vehicle_pose.x, vehicle_pose.y, 0);
    relaxed_grid_map.setCellValue(goal_state_.current_goal_.pose.position.x, 
                                 goal_state_.current_goal_.pose.position.y, 0);

    grid_map.setCellValue(vehicle_pose.x, vehicle_pose.y, 0);
    grid_map.setCellValue(goal_state_.current_goal_.pose.position.x, 
                         goal_state_.current_goal_.pose.position.y, 0);
    
    // 可选：发布地图
    if (config["Publish_Map"].as<bool>()) {
        origin_map.publishMap(map_origin_pub, "odom", grid_resolution_meters, start, goal);
        relaxed_grid_map.publishMap(map_relaxed_pub, "odom", grid_resolution_meters, start, goal);
        grid_map.publishMap(map_safety_pub, "odom", grid_resolution_meters, start, goal);
    }

    ready = true;
}


void MappingNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
    goal_state_.current_goal_.pose.position = goal_msg->pose.position;
    goal_state_.current_goal_.pose.orientation = goal_msg->pose.orientation;
    goal_state_.have_goal_ = true;
    
    // ROS_INFO("Received goal at (%.2f, %.2f)", 
    //          goal_msg->pose.position.x, goal_msg->pose.position.y);
             
    publishGoalMarker();
}

void MappingNode::executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
    goal_state_.current_goal_.pose.position = goal->target_pose.pose.position;
    goal_state_.current_goal_.pose.orientation = goal->target_pose.pose.orientation;
    goal_state_.have_goal_ = true;
    
    ROS_INFO("Received action goal at (%.2f, %.2f)", 
             goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
             
    publishGoalMarker();
}

void MappingNode::publishGoalMarker() {
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose = goal_state_.current_goal_.pose;
    marker.pose.orientation = goal_state_.current_goal_.pose.orientation;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker.lifetime = ros::Duration(0);
    
    visualization_msgs::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "goal";
    text_marker.id = 1;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    
    text_marker.pose = goal_state_.current_goal_.pose;
    text_marker.pose.position.z += 0.7;
    text_marker.pose.orientation = goal_state_.current_goal_.pose.orientation;
    
    text_marker.scale.z = 0.3;
    
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.text = "Goal";
    
    text_marker.lifetime = ros::Duration(0);
    
    goal_marker_pub_.publish(marker);
    goal_marker_pub_.publish(text_marker);
}