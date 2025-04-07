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
#include "Mapping.hpp"
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
    // double yaw;
} A_Star_Path_;

class AStarPlanningNode {
public:
    // Constructor
    AStarPlanningNode(std::shared_ptr<MappingNode> mapping_node, std::shared_ptr<TFSubscriberNode> tf_subscriber_node);

private:
    // Callback functions
    void TimerCallback(const ros::TimerEvent& event);
    
    double findClosestPathDistance(std::vector<A_Star_Path_> path, double vehicle_x, double vehicle_y);
    // A* Path planning function
    std::vector<A_Star_Path_> AStarSearch(DenseGridMap& grid_map,
                                          const std::pair<int, int>& start,
                                          const std::pair<int, int>& goal,
                                          double grid_resolution_meters);
    
    // Visualization function
    void publishAStarPathRviz(const std::vector<A_Star_Path_>& path,
                             double grid_resolution_meters);
    
    bool isPathColliding(const std::vector<A_Star_Path_>& path,
        DenseGridMap& grid_map);
    
    void publishLastAStarPath(std::vector<A_Star_Path_>& path, bool rush_sign);
    
    // TF listener node
    std::shared_ptr<MappingNode> mapping_node_;
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;
    
    // Publishers
    ros::Publisher a_star_path_pub_; // Publish A* path
    ros::Publisher a_star_path_rviz_pub_; // Publish A* path for RVIZ visualization
    
    // Subscribers
    ros::Timer timer_;
    
    // Data storage
    std::vector<A_Star_Path_> a_star_path_; // A* path with position and yaw
    
    // Configuration path
    std::string config_yaml_path;
    YAML::Node config;
    std::vector<A_Star_Path_> last_path_;
    
    MappingNode::MapSize map_size;
    MappingNode::VehiclePose vehicle_pose;
};

#endif // A_STAR_PLANNING_HPP