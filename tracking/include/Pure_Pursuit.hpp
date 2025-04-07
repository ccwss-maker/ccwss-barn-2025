#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "ros/ros.h"
#include "MSG_Process.hpp"
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"
#include <Eigen/Sparse>
#include "../../navigation/include/TFSubscriberNode.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>
#include <algorithm>

class PurePursuitNode {
public:
    PurePursuitNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node, std::shared_ptr<MsgProcessNode> msg_process_node);
private:
    void TimerCallback(const ros::TimerEvent& event);
    // 共享指针
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;
    std::shared_ptr<MsgProcessNode> msg_process_node_;  // control_node共享指针

    Eigen::Matrix3Xd control_points;
    ros::Timer timer_;

    std::string config_yaml_path;


    ros::Publisher twist_cmd_pub_;
    ros::Publisher predict_path_pub_;
    ros::Publisher ref_path_pub_;
    ros::Publisher marker_pub_;

    //MPC参数
    int Np = 10;                   //预测步长
    int Nc = 5;                    //控制步长
    double T = 0.05;             //控制周期0.05s
    
    // 上一个控制输入
    double last_v_;
    double last_omega_;

    // 控制限制
    double v_min_, v_max_; // 线速度限制(m/s)
    double omega_min_, omega_max_; // 角速度限制(rad/s)
    
    // 权重矩阵参数
    double w_x_, w_y_, w_phi_; // 状态跟踪权重
    double w_v_, w_omega_;     // 控制输入权重
    double w_dv_, w_domega_;   // 控制变化率权重

    // Matrix Odom_To_Base_Matrix;
    bool rush_sign;
};

#endif