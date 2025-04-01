#include "Mapping.hpp"
#include <algorithm>
MappingNode::MappingNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node) :
    tf_subscriber_node_(tf_subscriber_node)
{
    ros::NodeHandle nh;
    
    config_yaml_path = "/jackal_ws/src/navigation/config/config.yaml";
    
    // 初始化同步订阅者
    scan_sub_ = nh.subscribe("/map", 1, &MappingNode::scanCallback, this);
    
    // 初始化目标位置订阅者
    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MappingNode::goalCallback, this);
    
    // 初始化 IMU 订阅者
    imu_sub_ = nh.subscribe("/imu/data", 1, &MappingNode::imuCallback, this);

    // 初始化 move_base ActionServer
    action_server_ = std::make_unique<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>>(
        nh, 
        "move_base", 
        boost::bind(&MappingNode::executeGoalCallback, this, _1), 
        false // 不自动开始
    );
    action_server_->start();
    ROS_INFO("MoveBase ActionServer started");

    // 初始化发布者
    goal_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker_goal", 10);
    map_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/Map_Grid", 10);
    ROS_INFO("Mapping node initialized and started");


    angular_velocity_z_last_base = 0.0;
    time_last = ros::Time::now();
}


void MappingNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // 计算角速度
    double angular_velocity_z_base = imu_msg->angular_velocity.z;
    double dt = (imu_msg->header.stamp - time_last).toSec();
    linear_acceleration_x_base = imu_msg->linear_acceleration.x;
    linear_acceleration_y_base = imu_msg->linear_acceleration.y;
    angular_acceleration_z_base = (angular_velocity_z_base - angular_velocity_z_last_base) / dt;
    angular_velocity_z_last_base = angular_velocity_z_base;
    time_last = imu_msg->header.stamp;
}


void MappingNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // 车辆速度
    linear_velocity_x_odom = odom_msg->twist.twist.linear.x;
    linear_velocity_y_odom = odom_msg->twist.twist.linear.y;
    angular_velocity_z_odom = odom_msg->twist.twist.angular.z;
}

void MappingNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    Matrix Base_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "base_link");
    tf::Quaternion Base_To_Odom_Quaternion = Base_To_Odom_Matrix.Quaternion_Read();
    Eigen::Vector3d Base_To_Odom_Translation = Base_To_Odom_Matrix.Translation_Read();
    Eigen::Matrix3d Base_To_Odom_Rotation = Base_To_Odom_Matrix.Rotation_Read();

    double roll, pitch, yaw;
    tf::Matrix3x3(Base_To_Odom_Quaternion).getRPY(roll, pitch, yaw);
    vehicle_state_.x = Base_To_Odom_Translation.x();
    vehicle_state_.y = Base_To_Odom_Translation.y();
    vehicle_state_.yaw = yaw;
    Eigen::Vector3d vel_odom(linear_velocity_x_odom, linear_velocity_y_odom, 0.0);
    vehicle_state_.vx = vel_odom(0);
    vehicle_state_.vy = vel_odom(1);
    vehicle_state_.w = angular_velocity_z_odom;

    Eigen::Vector3d acc_base(linear_acceleration_x_base, linear_acceleration_y_base, 0.0);
    Eigen::Vector3d acc_odom = Base_To_Odom_Rotation * acc_base;
    vehicle_state_.ax = acc_odom(0);
    vehicle_state_.ay = acc_odom(1);
    vehicle_state_.aw = angular_acceleration_z_base;

    if (!goal_state_.have_goal_) {
        ROS_WARN_THROTTLE(5.0, "No goal received yet");
        return;
    }

    config = YAML::LoadFile(config_yaml_path);
    double a_star_resolution = config["resolution_meters"].as<double>();
    double car_length = config["Car_Length"].as<double>();
    double car_width  = config["Car_Width"].as<double>();
    double safety_hor = config["Safety_Hor"].as<double>();
    double inflate_radius = std::max(car_length, car_width) / 2.0 + safety_hor;

}

void MappingNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
    // 更新目标
    goal_state_.current_goal_.pose.position.x = goal_msg->pose.position.y;
    goal_state_.current_goal_.pose.position.y = goal_msg->pose.position.x;
    goal_state_.current_goal_.pose.orientation = goal_msg->pose.orientation;
    goal_state_.have_goal_ = true;
    // 每次收到目标点时都发布目标点标记
    publishGoalMarker();
}

void MappingNode::executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
    // 更新目标点
    goal_state_.current_goal_.pose.position.x = goal->target_pose.pose.position.y;
    goal_state_.current_goal_.pose.position.y = goal->target_pose.pose.position.x;
    goal_state_.current_goal_.pose.orientation = goal->target_pose.pose.orientation;
    goal_state_.have_goal_ = true;
    // 每次收到目标点时都发布目标点标记
    publishGoalMarker();
}

void MappingNode::publishGoalMarker() {
    visualization_msgs::Marker marker;
    
    // 设置基本属性
    marker.header.frame_id = "odom";  // 使用里程计坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal";  // 使用简单的命名空间
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置位置和姿态
    marker.pose = goal_state_.current_goal_.pose;
    marker.pose.orientation = goal_state_.current_goal_.pose.orientation;
    // 设置大小
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    
    // 设置颜色 (红色，半透明)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    // 设置持续时间 (0表示一直存在)
    marker.lifetime = ros::Duration(0);
    
    // 添加文本标签
    visualization_msgs::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "goal";  // 使用相同的命名空间
    text_marker.id = 1;  // 使用不同的ID
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    
    // 文本位置 (在目标点上方)
    text_marker.pose = goal_state_.current_goal_.pose;
    text_marker.pose.position.z += 0.7;  // 在球体上方显示文本
    text_marker.pose.orientation = goal_state_.current_goal_.pose.orientation;
    
    // 文本大小
    text_marker.scale.z = 0.3;  // 文本高度
    
    // 文本颜色 (白色)
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    // 文本内容
    text_marker.text = "Goal";
    
    // 设置持续时间 (0表示一直存在)
    text_marker.lifetime = ros::Duration(0);
    
    // 发布标记
    goal_marker_pub_.publish(marker);
    goal_marker_pub_.publish(text_marker);
}

void MappingNode::publishMapVisualizations(const nav_msgs::OccupancyGrid& inflated_map)
{
    if (!config["Publish_A_Star_Map"].as<bool>()) return;

    nav_msgs::OccupancyGrid grid = inflated_map;  // 直接复制地图
    grid.header.stamp = ros::Time::now();         // 更新时间戳

    // 可选：设置目标点高亮显示
    int width = grid.info.width;
    int height = grid.info.height;
    double resolution = grid.info.resolution;
    geometry_msgs::Pose origin = grid.info.origin;

    // 将目标点坐标映射为格子索引
    int gx = static_cast<int>((goal_state_.current_goal_.pose.position.x - origin.position.x) / resolution);
    int gy = static_cast<int>((goal_state_.current_goal_.pose.position.y - origin.position.y) / resolution);

    if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
        int goal_idx = gy * width + gx;
        grid.data[goal_idx] = 50;  // 目标点可视化为灰色
    }

    map_grid_pub_.publish(grid);
}