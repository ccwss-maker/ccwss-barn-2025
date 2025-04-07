#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "TFSubscriberNode.hpp"
#include "DenseGridMap.hpp"
#include "yaml-cpp/yaml.h"
#include <std_msgs/ColorRGBA.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

class MappingNode {
public:
    // 构造函数：接收TF订阅节点的共享指针
    MappingNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node);
    
    // 是否需要快速移动
    bool rush_sign;

    // 目标状态结构体
    struct GoalState {
        bool have_goal_ = false;
        geometry_msgs::PoseStamped current_goal_;
    } goal_state_;

    // 车辆位姿结构体
    struct VehiclePose {
        double x = 0;
        double y = 0;
        double yaw = 0;
    } vehicle_pose;

    // 地图大小结构体
    struct MapSize {
        double xmin = 0;
        double xmax = 0;
        double ymin = 0;
        double ymax = 0;
    } map_size;

    // 地图对象
    DenseGridMap origin_map;        // 原始障碍物地图
    DenseGridMap relaxed_grid_map;  // 松弛膨胀地图
    DenseGridMap grid_map;          // 安全膨胀地图

    // 地图参数
    double grid_resolution_meters = 0.1;
    double safety_hor = 0.0;
    double car_length = 0.0;
    double car_width = 0.0;

    // 地图是否准备就绪
    bool ready = false;

private:
    // 目标回调函数
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
    void executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    
    // 激光扫描回调函数
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    
    // 发布目标标记
    void publishGoalMarker();

    // TF订阅节点
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;

    // ROS发布器
    ros::Publisher goal_marker_pub_;
    ros::Publisher map_origin_pub;
    ros::Publisher map_relaxed_pub;
    ros::Publisher map_safety_pub;

    // ROS订阅器
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    
    // 动作服务器
    std::unique_ptr<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>> action_server_;

    // 变换矩阵
    Matrix Map_To_Odom_Matrix;

    // 配置文件路径
    std::string config_yaml_path;
    YAML::Node config;

    // 上一个路径原点
    double last_path_origin_xmin_ = 0;
    double last_path_origin_ymin_ = 0;
};

#endif // MAPPING_HPP