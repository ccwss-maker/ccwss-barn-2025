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

    std::vector<std::vector<int>> generateInflatedGridMap(double xmin, double xmax, double ymin, double ymax,
                                                            double resolution,
                                                            double car_length, double car_width,
                                                            double safety_margin);  

    bool rush_sign;
    typedef struct 
    {
        bool have_goal_;
        geometry_msgs::PoseStamped current_goal_;
    }GoalState;
    GoalState goal_state_;
    bool mapOriginChanged;

    typedef struct 
    {
        double x;
        double y;
        double yaw;
    } VehiclePose;
    VehiclePose vehicle_pose;


    typedef struct 
    {
        double xmin;
        double xmax;
        double ymin;
        double ymax;
    }MapSize;
    MapSize map_size;

    std::vector<std::vector<int>> origin_map;
    std::vector<std::vector<int>> relaxed_grid_map;
    std::vector<std::vector<int>> grid_map;
    double grid_resolution_meters;
    double safety_hor;
    double car_length;
    double car_width;

    bool ready;
    
private:
    // Callback functions
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
    void executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void publishGoalMarker();
    // Visualization function
    void publishGridMapRviz(const std::vector<std::vector<int>>& grid_map,
                            double x_min, double y_min,
                            double grid_resolution_meters,
                            const std::pair<int, int>& start,
                            const std::pair<int, int>& goal,
                            ros::Publisher& grid_pub_);
    // Planning functions
    
    // Publish goal marker
    
    // TF listener node
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;

    // Publishers
    ros::Publisher goal_marker_pub_;
    ros::Publisher map_origin_pub ;      // Publish A* map 
    ros::Publisher map_relaxed_pub ;      // Publish A* map 
    ros::Publisher map_safety_pub ;      // Publish A* map 
    // Subscribers
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    std::unique_ptr<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>> action_server_;
    
    // Data storage
    Matrix Map_To_Odom_Matrix;    // Map -> Odom 变换矩阵
    
    // Configuration path
    std::string config_yaml_path;
    YAML::Node config;


    Eigen::MatrixXd obstacle_points_matrix_; // Obstacle points in base_link frame
    std::vector<std::pair<Eigen::Vector2d, ros::Time>> historical_obstacle_points_;


    double last_path_origin_xmin_;
    double last_path_origin_ymin_;


};

#endif // MAPPING_HPP