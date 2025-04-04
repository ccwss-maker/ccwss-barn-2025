#ifndef A_STAR_PLANNING_HPP
#define A_STAR_PLANNING_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <queue> // For std::priority_queue
#include <tf/transform_datatypes.h> // For tf::Quaternion
#include "TFSubscriberNode.hpp"
#include "yaml-cpp/yaml.h"
#include "astar_msgs/AStarPathArray.h"


#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

// A* Search Node structure
struct AStarNode {
    int x, y;
    float g_cost, h_cost;
    int parent_x, parent_y;

    AStarNode(int x, int y, float g, float h, int px, int py) 
        : x(x), y(y), g_cost(g), h_cost(h), parent_x(px), parent_y(py) {}

    float f_cost() const {
        return g_cost + h_cost;
    }

    bool operator>(const AStarNode& other) const {
        return f_cost() > other.f_cost();
    }
};

// A* Path structure to store path points with position and yaw
typedef struct {
    Eigen::Vector2d position;
    double yaw;
} A_Star_Path_;


class AStarPlanningNode {
public:
    // Constructor
    AStarPlanningNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node);
private:
    // Callback functions
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
    void executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    // Planning functions
    void planTrajectory();
    
    // Publish goal marker
    void publishGoalMarker();
    
    // A* Path planning function
    std::vector<A_Star_Path_> AStarSearch(const std::vector<std::vector<int>>& grid_map, 
                                         const std::pair<int, int>& start, 
                                         const std::pair<int, int>& goal, 
                                         double grid_resolution_meters);
    
    // Visualization function
    void publishAStarGridMapRviz(const std::vector<std::vector<int>>& grid_map,
        double x_min, double y_min,
        double grid_resolution_meters,
        const std::pair<int, int>& start,
        const std::pair<int, int>& goal,
        ros::Publisher& grid_pub_);
    void publishAStarPathRviz(const std::vector<A_Star_Path_>& path,
                        double x_min, double y_min,
                        double grid_resolution_meters);
    bool isPathColliding(const std::vector<A_Star_Path_>& path,
        const std::vector<std::vector<int>>& grid_map,
        double xmin, double ymin, double resolution);
    std::vector<std::vector<int>> generateInflatedGridMap(const Eigen::MatrixXd& obstacle_points,
                                                        double xmin, double xmax, double ymin, double ymax,
                                                        double resolution,
                                                        double car_length, double car_width,
                                                        double safety_margin);       
    std::vector<A_Star_Path_> remapPathToNewMapOrigin(const std::vector<A_Star_Path_>& old_path,
                                                        double old_xmin, double old_ymin,
                                                        double new_xmin, double new_ymin,
                                                        double resolution);                                     
    void publishLastAStarPath();
    // TF listener node
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;

    // Publishers
    ros::Publisher goal_marker_pub_;
    ros::Publisher a_star_path_pub_;       // Publish A* path
    ros::Publisher a_star_path_rviz_pub_;  // Publish A* path for RVIZ visualization
    ros::Publisher a_star_map_pub ;      // Publish A* map 
    ros::Publisher a_star_map_relaxed_pub ;      // Publish A* map 
    ros::Publisher a_start_map_origin_pub ;      // Publish A* map 

    // Subscribers
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    std::unique_ptr<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>> action_server_;
    
    // Data storage
    Matrix Laser_To_Odom_Matrix;    // 雷达 -> 世界 变换矩阵
    geometry_msgs::PoseStamped current_goal_;
    std::vector<A_Star_Path_> a_star_path_; // A* path with position and yaw
    Eigen::MatrixXd obstacle_points_matrix_; // Obstacle points in base_link frame
    std::vector<std::pair<Eigen::Vector2d, ros::Time>> historical_obstacle_points_;

    
    // Status flags
    bool have_goal_;
    
    // Configuration path
    std::string config_yaml_path;
    YAML::Node config;

    typedef struct 
    {
        double xmin;
        double xmax;
        double ymin;
        double ymax;
    }MapSize;
    MapSize map_size;
    
    std::vector<A_Star_Path_> last_path_;
    double last_path_origin_xmin_;
    double last_path_origin_ymin_;

    typedef struct 
    {
        double x;
        double y;
        double yaw;
    } VehiclePose;
    VehiclePose vehicle_pose;

    bool rush_sign;
};

#endif // A_STAR_PLANNING_HPP