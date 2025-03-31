#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "TFSubscriberNode.hpp"
#include "yaml-cpp/yaml.h"

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

class MappingNode {
public:
    // Constructor
    MappingNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node);
    typedef struct 
    {
        bool have_goal_;
        geometry_msgs::PoseStamped current_goal_;
    }GoalState;
    GoalState goal_state_;
    nav_msgs::OccupancyGrid inflated_map;
    typedef struct 
    {
        double x;
        double y;
        double yaw;
        double vx;
        double vy;
        double w;
        double ax;
        double ay;
        double aw;
    }VehicleState;
    VehicleState vehicle_state_;
    
private:
    // Callback functions
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
    void executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void publishGoalMarker();
    void publishMapVisualizations(const nav_msgs::OccupancyGrid& inflated_map);
    void inflateObstacleGrid(const nav_msgs::OccupancyGrid& source_map, nav_msgs::OccupancyGrid& target_map, double inflate_radius, double target_resolution);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    // Planning functions
    
    // Publish goal marker
    
    // TF listener node
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;

    // Publishers
    ros::Publisher goal_marker_pub_;
    ros::Publisher wall_marker_pub_;
    ros::Publisher map_grid_pub_ ;      // Publish A* grid map
    // Subscribers
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    std::unique_ptr<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>> action_server_;
    
    // Data storage
    Matrix Map_To_Odom_Matrix;    // Map -> Odom 变换矩阵
    typedef struct 
    {
        double xmin;
        double xmax;
        double ymin;
        double ymax;
    }MapSize;
    MapSize map_size;
    
    // Configuration path
    std::string config_yaml_path;
    YAML::Node config;

    double angular_velocity_z_odom;     // angular velocity z
    double linear_velocity_x_odom;      // linear velocity x
    double linear_velocity_y_odom;      // linear velocity y
    double linear_acceleration_x_base; // linear acceleration x
    double linear_acceleration_y_base; // linear acceleration y
    double angular_acceleration_z_base;      // angular acceleration z
    double angular_velocity_z_last_base;      // Last angular velocity z
    ros::Time time_last;                    // Last time
};

#endif // MAPPING_HPP