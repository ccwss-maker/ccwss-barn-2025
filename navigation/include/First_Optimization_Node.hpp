#ifndef FIRST_OPTIMIZATION_NODE_HPP
#define FIRST_OPTIMIZATION_NODE_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"
#include "astar_msgs/AStarPathArray.h"
#include "initial_optimized_msgs/InitialOptimizedTrajectory.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "utils/minco.hpp"
#include <LBFGS.h>

class FirstOptimizationNode {
public:
    // Constructor
    FirstOptimizationNode();
private:
    typedef struct{
        Eigen::Vector3d position;
    }A_Star_Path_;

    typedef struct{
        Eigen::MatrixX2d b;      
        Eigen::VectorXd times;
    }Initial_Optimized_Trajectory_;

    // Callback functions
    void TimerCallback(const ros::TimerEvent& event);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & map);
    void pathCallback(const astar_msgs::AStarPathArray::ConstPtr & astar_path);
    double ComputeCostAndGradient(const Eigen::VectorXd& params, Eigen::VectorXd& grad);
    double ComputeCostAndGradient_Position(Eigen::MatrixX3d& GradByPoints, Eigen::VectorXd& GradByTimes);
    void visualizeTrajectory(initial_optimized_msgs::InitialOptimizedTrajectory InitialOptimizedTrajectory);
    int FindClosestAStarPointIndex(const Eigen::Vector3d& query_point) const;
    void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T);
    template <typename EIGENVEC>
    void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau);
    void forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P);
    template <typename EIGENVEC>
    void backwardP(const Eigen::Matrix3Xd &P, EIGENVEC &xi);
    template <typename EIGENVEC>
    void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT, EIGENVEC &gradTau);
    template <typename EIGENVEC>
    void backwardGradP(const Eigen::VectorXd &xi, const Eigen::Matrix3Xd &gradP, EIGENVEC &gradXi);
    void Emergency_Brake_Publish();
    // Publishers
    ros::Publisher First_Optimized_Trajectory_Publisher_;
    ros::Publisher First_Opimization_Marker_Publisher_;
    // Subscribers
    ros::Subscriber a_star_sub_;
    ros::Subscriber map_sub_;
    ros::Timer timer_;


    std::string config_yaml_path;
    YAML::Node config;


    cv::Mat dist_map; 
    nav_msgs::OccupancyGrid grid_map;
    astar_msgs::AStarPathArray astar_path;
    std::vector<A_Star_Path_> A_Star_Path, A_Star_Path_Last;
    minco::MINCO_S3NU minco;
    int pieceN;
    int spatialDim;
    int temporalDim;
    Eigen::Matrix3Xd points;        
    Eigen::VectorXd times;
    
    double weight_time;
    double weight_energy_x;
    double weight_energy_y;
    double weight_energy_w;
    double weight_position_x;
    double weight_position_y;
    double weight_position_w;
    double weight_linear_velocity;
    double weight_angular_velocity;

    double max_linear_velocity;
    double max_angular_velocity;

    double last_cost = std::numeric_limits<double>::max();
    double last_cost_Yaw = std::numeric_limits<double>::max();
    initial_optimized_msgs::InitialOptimizedTrajectory InitialOptimizedTrajectory_pub;

    bool sloved;
};

#endif // FIRST_OPTIMIZATION_NODE_HPP