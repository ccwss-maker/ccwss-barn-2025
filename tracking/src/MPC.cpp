#include "MPC.hpp"
#include "math.h"
#include <OsqpEigen/OsqpEigen.h>
#include <algorithm>
#include <numeric>
// -------------------- 工具函数 -------------------- //
template <typename T>
T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

// -------------------- 构造函数 -------------------- //
MPCNode::MPCNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node, std::shared_ptr<MsgProcessNode> msg_process_node) :
    tf_subscriber_node_(tf_subscriber_node), msg_process_node_(msg_process_node)
{
    ros::NodeHandle nh;
    config_yaml_path = "/jackal_ws/src/tracking/config/config.yaml";
    YAML::Node config = YAML::LoadFile(config_yaml_path);
    double T = config["T"].as<double>();

    // 创建定时器
    timer_ = nh.createTimer(ros::Duration(T), &MPCNode::TimerCallback, this);

    // 发布器初始化
    twist_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 10);
    predict_path_pub_ = nh.advertise<nav_msgs::Path>("/mpc/predict_path", 1);
    ref_path_pub_ = nh.advertise<nav_msgs::Path>("/mpc/reference_path", 1);
    // marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mpc/reference_path", 1);

    rush_sign = false;
}

// -------------------- MPC 核心控制回调函数 -------------------- //
void MPCNode::TimerCallback(const ros::TimerEvent& event)
{   
    // ---------------- 获取TF与基础配置 ---------------- //
    Matrix Base_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "base_link");
    Matrix Odom_To_Base_Matrix = tf_subscriber_node_->Matrix_Read("base_link", "odom");
    YAML::Node config = YAML::LoadFile(config_yaml_path);

    Eigen::MatrixXd control_points = msg_process_node_->control_points;
    cv::Mat dist_map = msg_process_node_->dist_map;
    
    // 控制参数读取
    T = config["T"].as<double>();
    v_min_ = config["control_limits"]["v_min"].as<double>();
    v_max_ = config["control_limits"]["v_max"].as<double>();
    omega_min_ = config["control_limits"]["omega_min"].as<double>();
    omega_max_ = config["control_limits"]["omega_max"].as<double>();
    
    // 纯跟踪参数
    double look_ahead_distance = config["pure_pursuit"]["look_ahead_distance"].as<double>();
    double min_look_ahead = config["pure_pursuit"]["min_look_ahead"].as<double>();
    double max_look_ahead = config["pure_pursuit"]["max_look_ahead"].as<double>();
    double k_look_ahead = config["pure_pursuit"]["k_look_ahead"].as<double>();
    
    // 速度控制参数
    double alpha = config["speed_control"]["alpha"].as<double>();
    int curvature_window = config["speed_control"]["curvature_window"].as<int>();
    double min_curvature_radius = config["speed_control"]["min_radius"].as<double>();

    // 当前位姿提取
    Eigen::Vector3d x0;
    x0(0) = Base_To_Odom_Matrix.Translation_Read()(0);
    x0(1) = Base_To_Odom_Matrix.Translation_Read()(1);
    double roll, pitch, yaw;
    tf::Quaternion transform_q = Base_To_Odom_Matrix.Quaternion_Read();
    tf::Matrix3x3(transform_q).getRPY(roll, pitch, yaw);
    x0(2) = yaw;

    // ---------------- 加速模式（Rush）控制 ---------------- //
    if (msg_process_node_->rush_sign) {
        std::cout << "rush" << std::endl;
        Eigen::Vector3d goal = control_points.col(control_points.cols() - 1);
        Eigen::Vector4d goal_odom;
        goal_odom << goal(0), goal(1), 0.0, 1.0;
        Eigen::Matrix4d T_odom_to_base = Odom_To_Base_Matrix.Rotation_Translation_Read();
        Eigen::Vector4d goal_base_homogeneous = T_odom_to_base * goal_odom;
        Eigen::Vector3d goal_base = goal_base_homogeneous.head<3>();

        double dx_body = goal_base(0);
        double dy_body = goal_base(1);

        double k_v = config["rush"]["k_v"].as<double>();
        double k_w = config["rush"]["k_w"].as<double>();
        double v_cmd = k_v * dx_body;
        double omega_cmd = k_w * std::atan2(dy_body, dx_body);

        geometry_msgs::Twist twist_cmd;
        twist_cmd.linear.x = v_cmd;
        twist_cmd.angular.z = omega_cmd;
        twist_cmd_pub_.publish(twist_cmd);
        return;
    }

    static ros::Time emergency_brake_expire_time_ = ros::Time::now();
    geometry_msgs::Twist twist_cmd;

    if (control_points.cols() == 0) {
        double emergency_brake_duration = 0.3;
        if (config["emergency_brake_duration"]) {
            emergency_brake_duration = config["emergency_brake_duration"].as<double>();
        }
        emergency_brake_expire_time_ = ros::Time::now() + ros::Duration(emergency_brake_duration);
    }

    if (ros::Time::now() <= emergency_brake_expire_time_) 
    {
        // 计算从紧急制动开始到现在经过的时间
        double elapsed_time = (ros::Time::now() - (emergency_brake_expire_time_ - ros::Duration(config["emergency_brake_duration"].as<double>()))).toSec();
        double total_time = config["emergency_brake_duration"].as<double>();
        
        // 计算当前应该的减速比例 (0到1之间)
        double deceleration_ratio = std::min(1.0, elapsed_time / (total_time * 0.8));
        
        // 计算目标速度 (从当前速度平滑过渡到0)
        static double last_cmd_v = 0.0;  // 用于记录上次发布的速度命令
        double target_v = last_cmd_v * (1.0 - deceleration_ratio);
        
        // 限制减速度
        double max_deceleration = config["control_limits"]["max_deceleration"].as<double>();
        double min_velocity_change = -max_deceleration * T;
        double velocity_change = target_v - last_cmd_v;
        
        if (velocity_change < min_velocity_change) {
            target_v = last_cmd_v + min_velocity_change;
        }
        
        // 更新并发布命令
        twist_cmd.linear.x = target_v;
        twist_cmd.angular.z = 0.0;  // 角速度可以直接设为0
        twist_cmd_pub_.publish(twist_cmd);
        
        // 记录本次发布的速度
        last_cmd_v = target_v;
        
        ROS_INFO("Emergency brake: %.2f seconds remaining, current velocity: %.2f", 
                (emergency_brake_expire_time_ - ros::Time::now()).toSec(), target_v);
        return;
    }

    // ---------------- 寻找最近轨迹点 ---------------- //
    Eigen::Vector2d current_xy(x0(0), x0(1));
    int nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < control_points.cols(); ++i) {
        Eigen::Vector2d pt_xy = control_points.block<2,1>(0, i);
        double dist = (pt_xy - current_xy).norm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    // ---------------- 计算横向误差和航向角误差（用于调试） ---------------- //
    double lateral_error = 0.0;
    double heading_error = 0.0;
    if (nearest_idx < control_points.cols() - 1) {
        // 计算轨迹线段的方向向量
        Eigen::Vector2d p1 = control_points.block<2,1>(0, nearest_idx);
        Eigen::Vector2d p2 = control_points.block<2,1>(0, nearest_idx + 1);
        Eigen::Vector2d path_vector = p2 - p1;
        
        if (path_vector.norm() > 1e-6) {
            Eigen::Vector2d unit_path = path_vector.normalized();
            Eigen::Vector2d normal(-unit_path(1), unit_path(0));  // 法向量
            
            // 横向误差（车辆到轨迹线段的垂直距离）
            lateral_error = (current_xy - p1).dot(normal);
            
            // 计算轨迹的航向角
            double path_heading = std::atan2(unit_path(1), unit_path(0));
            
            // 航向角误差
            heading_error = std::atan2(std::sin(x0(2) - path_heading), 
                                      std::cos(x0(2) - path_heading));
        }
    }
    
    // ---------------- 计算曲率和曲率半径 ---------------- //
    std::vector<double> curvatures;
    
    for (int i = std::max(1, nearest_idx - curvature_window/2); 
         i < std::min((int)control_points.cols() - 1, nearest_idx + curvature_window/2); 
         ++i) {
        
        // 使用三点法计算曲率
        if (i > 0 && i < control_points.cols() - 1) {
            Eigen::Vector2d p0 = control_points.block<2,1>(0, i-1);
            Eigen::Vector2d p1 = control_points.block<2,1>(0, i);
            Eigen::Vector2d p2 = control_points.block<2,1>(0, i+1);
            
            // 计算三点之间的距离
            double a = (p0 - p1).norm();
            double b = (p1 - p2).norm();
            double c = (p2 - p0).norm();
            
            // 避免数值问题
            if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
                continue;
            }
            
            // 计算半周长
            double s = (a + b + c) / 2.0;
            
            // 计算三角形面积
            double area = std::sqrt(std::max(0.0, s * (s - a) * (s - b) * (s - c)));
            
            // 计算曲率（曲率 = 4 * 面积 / (a * b * c)）
            double curvature = 4.0 * area / (a * b * c);
            curvatures.push_back(curvature);
        }
    }
    
    // 计算平均曲率
    double avg_curvature = 0.0;
    if (!curvatures.empty()) {
        avg_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
    }
    
    // 计算曲率半径（避免除零）
    double curvature_radius = (avg_curvature > 1e-6) ? (1.0 / avg_curvature) : 1000.0;
    curvature_radius = std::max(curvature_radius, min_curvature_radius);

    // ---------------- 基于速度调整前向距离 ---------------- //
    // 动态调整前向距离：车速越高，前向距离越大
    double current_speed = std::max(0.1, std::abs(x0(0)));  // 使用当前速度（避免零速度）
    look_ahead_distance = min_look_ahead + k_look_ahead * current_speed;
    look_ahead_distance = clamp(look_ahead_distance, min_look_ahead, max_look_ahead);

    // ---------------- 寻找目标点（纯跟踪） ---------------- //
    int target_idx = nearest_idx;
    double accumulated_distance = 0.0;
    
    // 从最近点开始搜索满足前向距离的点
    for (int i = nearest_idx; i < control_points.cols() - 1; ++i) {
        Eigen::Vector2d p1 = control_points.block<2,1>(0, i);
        Eigen::Vector2d p2 = control_points.block<2,1>(0, i+1);
        
        double segment_length = (p2 - p1).norm();
        accumulated_distance += segment_length;
        
        if (accumulated_distance >= look_ahead_distance) {
            target_idx = i + 1;
            break;
        }
    }
    
    // 如果找不到满足条件的点，使用最远点
    if (target_idx == nearest_idx && nearest_idx < control_points.cols() - 1) {
        target_idx = std::min(nearest_idx + 5, (int)control_points.cols() - 1);
    }
    
    // ---------------- 计算目标点在车身坐标系中的位置 ---------------- //
    Eigen::Vector3d target_point = control_points.col(target_idx);
    Eigen::Vector4d target_odom;
    target_odom << target_point(0), target_point(1), 0.0, 1.0;
    
    Eigen::Matrix4d T_odom_to_base = Odom_To_Base_Matrix.Rotation_Translation_Read();
    Eigen::Vector4d target_base_homogeneous = T_odom_to_base * target_odom;
    Eigen::Vector3d target_base = target_base_homogeneous.head<3>();
    
    double target_x = target_base(0);
    double target_y = target_base(1);
    
    // ---------------- 计算曲率自适应速度 ---------------- //
    // 速度与曲率成反比：曲率大，速度小
    double cmd_v = v_max_ * (1.0 - std::exp(-alpha * curvature_radius));
    cmd_v = clamp(cmd_v, v_min_, v_max_);
    
    // ---------------- 计算纯跟踪角速度 ---------------- //
    // 纯跟踪控制律：角速度 = 2*v*sin(alpha)/L，其中L是前视距离，alpha是目标点与车头的夹角
    double L = std::sqrt(target_x*target_x + target_y*target_y);
    double cmd_omega = 0.0;
    
    if (L > 0.1) {  // 避免除以零或很小的值
        double alpha = std::atan2(target_y, target_x);
        cmd_omega = 2.0 * cmd_v * std::sin(alpha) / L;
    } else {
        // 如果目标点太近，使用简单的方向控制
        cmd_omega = std::atan2(target_y, target_x) * 2.0;
    }
    
    cmd_omega = clamp(cmd_omega, omega_min_, omega_max_);
    
    // ---------------- 控制指令发布 ---------------- //
    if (config["control"].as<bool>()) {
        geometry_msgs::Twist twist_cmd;
        twist_cmd.linear.x = cmd_v;
        twist_cmd.angular.z = cmd_omega;
        twist_cmd_pub_.publish(twist_cmd);
    }
    
    // ---------------- 发布预测轨迹（可视化） ---------------- //
    nav_msgs::Path predict_path;
    predict_path.header.stamp = ros::Time::now();
    predict_path.header.frame_id = "odom";
    
    // 简单模拟车辆轨迹
    int predict_steps = 20;
    Eigen::Vector3d x_pred = x0;
    
    for (int i = 0; i < predict_steps; ++i) {
        double phi = x_pred(2);
        x_pred(0) += T * cmd_v * std::cos(phi);
        x_pred(1) += T * cmd_v * std::sin(phi);
        x_pred(2) += T * cmd_omega;
        
        geometry_msgs::PoseStamped pose;
        pose.header = predict_path.header;
        pose.pose.position.x = x_pred(0);
        pose.pose.position.y = x_pred(1);
        pose.pose.position.z = 0.0;
        
        tf::Quaternion q;
        q.setRPY(0, 0, x_pred(2));
        tf::quaternionTFToMsg(q, pose.pose.orientation);
        predict_path.poses.push_back(pose);
    }
    predict_path_pub_.publish(predict_path);
    
    // ---------------- 发布参考轨迹（可视化） ---------------- //
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = ros::Time::now();
    
    for (int i = nearest_idx; i < std::min(nearest_idx + 30, (int)control_points.cols()); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = control_points(0, i);
        pose.pose.position.y = control_points(1, i);
        pose.pose.position.z = 0.2;
        
        double yaw = control_points(2, i);
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        path_msg.poses.push_back(pose);
    }
    ref_path_pub_.publish(path_msg);
    
    // 调试输出
    ROS_INFO("Pure Pursuit | Look ahead: %.2f m, Target idx: %d, Lateral error: %.3f m, Curvature radius: %.2f m, Cmd v: %.2f, Cmd w: %.2f", 
             look_ahead_distance, target_idx - nearest_idx, lateral_error, curvature_radius, cmd_v, cmd_omega);
}