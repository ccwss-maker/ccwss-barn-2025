#ifndef MSG_PROCESS_HPP
#define MSG_PROCESS_HPP

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include "astar_msgs/AStarPathArray.h"
#include "initial_optimized_msgs/InitialOptimizedTrajectory.h"
#include "Eigen/Dense"
#include "../../navigation/include/TFSubscriberNode.hpp"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "../../navigation/include/utils/minco.hpp"
class MsgProcessNode {
public:
    MsgProcessNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node);
    Eigen::Matrix3Xd control_points;   // 控制点
    cv::Mat dist_map; 
    nav_msgs::OccupancyGrid grid_map;
    bool rush_sign;

private:
    void AStarTrajectoryCallback(const astar_msgs::AStarPathArray::ConstPtr& astar_path);
    void FirstOptiTrajectoryCallback(const initial_optimized_msgs::InitialOptimizedTrajectory::ConstPtr& first_opti_path);

    
    void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void findCurrentSegmentAndLocalTime(double t, int & segment, double & local_t, int pieceN, Eigen::VectorXd times);
    //话题订阅
    ros::Subscriber A_Star_Traj_Subscriber_;
    //话题订阅
    ros::Subscriber Map_Subscriber_;
    ros::Subscriber First_Opti_Traj_Subscriber_;
    
    //话题发布
    ros::Publisher Tracking_Traj_Maker_Publisher_;
    // TF listener node
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;

    Matrix Odom_To_Base_Matrix;      // 世界 -> 车辆 变换矩阵


    std::string config_yaml_path;
};

#endif