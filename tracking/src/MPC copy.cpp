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
    double a_min = config["control_limits"]["a_min"].as<double>();
    double a_max = config["control_limits"]["a_max"].as<double>();
    double alpha_min = config["control_limits"]["alpha_min"].as<double>();
    double alpha_max = config["control_limits"]["alpha_max"].as<double>();

    // 读取跟踪权重
    double w_lat = config["weights"]["lateral"].as<double>();  // 横向误差权重
    double w_yaw = config["weights"]["yaw"].as<double>();  // 航向角误差权重
    double w_v = config["weights"]["velocity"].as<double>();  // 速度跟踪权重
    double w_lon = config["weights"]["longitudinal"].as<double>();  // 纵向误差权重
    int lookahead = config["lookahead"].as<int>();

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
        emergency_brake_expire_time_ = ros::Time::now() + ros::Duration(config["emergency_brake_duration"].as<double>());
    }

    if (ros::Time::now() <= emergency_brake_expire_time_) {
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.0;
        twist_cmd_pub_.publish(twist_cmd);
        std::cout << "stop" << std::endl;
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

    // ---------------- 计算横向误差和航向角误差 ---------------- //
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
    int curvature_window = config["speed_control"]["curvature_window"].as<int>();
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
            
            // 计算半周长
            double s = (a + b + c) / 2.0;
            
            // 计算三角形面积
            double area = std::sqrt(s * (s - a) * (s - b) * (s - c));
            
            // 计算曲率（曲率 = 4 * 面积 / (a * b * c)）
            if (a > 1e-6 && b > 1e-6 && c > 1e-6) {
                double curvature = 4.0 * area / (a * b * c);
                curvatures.push_back(curvature);
            }
        }
    }
    
    // 计算平均曲率
    double avg_curvature = 0.0;
    if (!curvatures.empty()) {
        avg_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
    }
    
    // 计算曲率半径（避免除零）
    double curvature_radius = (avg_curvature > 1e-6) ? (1.0 / avg_curvature) : 1000.0;
    
    // ------------------- 预测步长调整 ------------------- //
    int Np_max = config["Np_max"].as<int>();
    int Np_min = config["Np_min"].as<int>();
    
    // 曲率越大，预测步长越小
    double curvature_sensitivity = config["curvature_sensitivity"].as<double>();
    double normalized_curvature = std::min(1.0, avg_curvature * curvature_sensitivity);
    Np = static_cast<int>(Np_max * (1.0 - normalized_curvature) + Np_min * normalized_curvature);
    Np = clamp(Np, Np_min, Np_max);

    // ------------------- 截取预测与控制点 ------------------- //
    int N_start = std::min(nearest_idx + lookahead, (int)control_points.cols() - 1);
    Np = std::min(Np, (int)control_points.cols() - N_start);
    Nc = std::min(config["Nc"].as<int>(), Np);
    
    // ------------------- 状态空间离散模型 ------------------- //
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
    double phi0 = x0(2);
    Eigen::MatrixXd B(3, 2);
    B << T * cos(phi0), 0,
         T * sin(phi0), 0,
         0,             T;

    Eigen::MatrixXd A_bar = Eigen::MatrixXd::Zero(3 * Np, 3);
    Eigen::MatrixXd B_bar = Eigen::MatrixXd::Zero(3 * Np, 2 * Nc);
    for (int i = 0; i < Np; ++i) {
        A_bar.block(3 * i, 0, 3, 3) = A;
        for (int j = 0; j <= std::min(i, Nc - 1); ++j) {
            B_bar.block(3 * i, 2 * j, 3, 2) = B;
        }
    }

    // ------------------- 构造参考轨迹 ------------------- //
    Eigen::VectorXd ref(3 * Np);
    for (int i = 0; i < Np; i++) {
        ref.segment<3>(3 * i) = control_points.col(N_start + i);
    }
    
    // ------------------- 构造参考速度（基于曲率） ------------------- //
    Eigen::VectorXd v_ref = Eigen::VectorXd::Zero(Nc);
    double alpha = config["speed_control"]["alpha"].as<double>();  // 曲率对速度的影响因子
    double min_curvature_radius = config["speed_control"]["min_radius"].as<double>();  // 最小曲率半径
    
    for (int i = 0; i < Nc; ++i) {
        // 计算当前位置的曲率
        int idx = N_start + i;
        double local_curvature_radius = curvature_radius;
        
        // 如果有必要，可以为每个预测点计算单独的曲率
        if (idx > 0 && idx < control_points.cols() - 1) {
            Eigen::Vector2d p0 = control_points.block<2,1>(0, idx-1);
            Eigen::Vector2d p1 = control_points.block<2,1>(0, idx);
            Eigen::Vector2d p2 = control_points.block<2,1>(0, idx+1);
            
            double a = (p0 - p1).norm();
            double b = (p1 - p2).norm();
            double c = (p2 - p0).norm();
            double s = (a + b + c) / 2.0;
            double area = std::sqrt(s * (s - a) * (s - b) * (s - c));
            
            if (a > 1e-6 && b > 1e-6 && c > 1e-6) {
                double local_curvature = 4.0 * area / (a * b * c);
                local_curvature_radius = (local_curvature > 1e-6) ? (1.0 / local_curvature) : 1000.0;
            }
        }
        
        // 限制最小曲率半径，避免速度过低
        local_curvature_radius = std::max(local_curvature_radius, min_curvature_radius);
        
        // 基于曲率半径映射到速度：曲率半径越小，速度越低
        double v = v_max_ * (1.0 - std::exp(-alpha * local_curvature_radius));
        v_ref(i) = clamp(v, v_min_, v_max_);
    }

    // ------------------- 构建跟踪误差成本矩阵 ------------------- //
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3 * Np, 3 * Np);
    
    // 根据横向误差和航向角误差调整权重
    double lat_scale = 1.0;
    double yaw_scale = 1.0;
    double lateral_threshold = config["lateral_control"]["error_threshold"].as<double>();
    
    if (std::abs(lateral_error) > lateral_threshold) {
        double gain = config["lateral_control"]["error_gain"].as<double>();
        lat_scale = 1.0 + gain * std::abs(lateral_error);
    }
    
    if (std::abs(heading_error) > config["lateral_control"]["heading_threshold"].as<double>()) {
        double gain = config["lateral_control"]["heading_gain"].as<double>();
        yaw_scale = 1.0 + gain * std::abs(heading_error);
    }
    
    // 设置跟踪误差代价矩阵
    for (int i = 0; i < Np; ++i) {
        // 使用横向误差计算修正
        double ref_yaw = ref(3 * i + 2);  // 参考轨迹在该点的朝向
        
        // 构建旋转矩阵，将世界坐标系误差转换为轨迹坐标系误差
        double cos_yaw = cos(ref_yaw);
        double sin_yaw = sin(ref_yaw);
        
        // 在跟踪点的局部坐标系下设置权重
        // x'表示沿轨迹方向的纵向分量，y'表示垂直于轨迹的横向分量
        double wx = w_lon;  // 纵向权重（可以很小）
        double wy = w_lat * lat_scale;  // 横向权重
        double wphi = w_yaw * yaw_scale;  // 航向权重
        
        // 构建这一点的旋转权重矩阵
        Eigen::Matrix2d Rot;
        Rot << cos_yaw, sin_yaw, 
              -sin_yaw, cos_yaw;
        
        Eigen::Matrix2d W;
        W << wx, 0,
             0, wy;
        
        // 计算旋转后的权重矩阵
        Eigen::Matrix2d W_rot = Rot.transpose() * W * Rot;
        
        // 应用到状态权重矩阵
        Q(3*i, 3*i) = W_rot(0,0);
        Q(3*i, 3*i+1) = W_rot(0,1);
        Q(3*i+1, 3*i) = W_rot(1,0);
        Q(3*i+1, 3*i+1) = W_rot(1,1);
        Q(3*i+2, 3*i+2) = wphi;  // 偏航角权重
    }

    // ------------------- 构建速度跟踪成本矩阵 ------------------- //
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2 * Nc, 2 * Nc);
    for (int i = 0; i < Nc; ++i) {
        R(2*i, 2*i) = w_v;  // 线速度v的跟踪权重
    }
    
    // 构建速度参考向量
    Eigen::VectorXd u_ref = Eigen::VectorXd::Zero(2 * Nc);
    for (int i = 0; i < Nc; ++i) {
        u_ref(2*i) = v_ref(i);  // 线速度参考
    }

    // ------------------- 优化目标函数构建 ------------------- //
    Eigen::MatrixXd H_dense = B_bar.transpose() * Q * B_bar + R;
    Eigen::VectorXd f = B_bar.transpose() * Q * (A_bar * x0 - ref) - R * u_ref;

    // ------------------- 控制输入约束构建 ------------------- //
    int n_variables = 2 * Nc;
    int n_constraints = 2 * Nc + 2 * (Nc - 1);
    Eigen::SparseMatrix<double> constraintMatrix(n_constraints, n_variables);

    // 控制输入范围（v/omega）
    for (int i = 0; i < 2 * Nc; ++i) {
        constraintMatrix.insert(i, i) = 1.0;
    }

    // 加速度约束 v(i+1) - v(i)
    for (int i = 0; i < Nc - 1; ++i) {
        constraintMatrix.insert(2 * Nc + i, 2 * (i + 1)) = 1.0;
        constraintMatrix.insert(2 * Nc + i, 2 * i) = -1.0;
    }

    // 角加速度约束 omega(i+1) - omega(i)
    int offset_ang = 2 * Nc + (Nc - 1);
    for (int i = 0; i < Nc - 1; ++i) {
        constraintMatrix.insert(offset_ang + i, 2 * (i + 1) + 1) = 1.0;
        constraintMatrix.insert(offset_ang + i, 2 * i + 1) = -1.0;
    }

    // ------------------- 约束上下界设置 ------------------- //
    Eigen::VectorXd lowerBound = Eigen::VectorXd::Constant(n_constraints, -OsqpEigen::INFTY);
    Eigen::VectorXd upperBound = Eigen::VectorXd::Constant(n_constraints, OsqpEigen::INFTY);

    for (int i = 0; i < Nc; ++i) {
        lowerBound[2 * i] = v_min_;
        upperBound[2 * i] = v_max_;
        lowerBound[2 * i + 1] = omega_min_;
        upperBound[2 * i + 1] = omega_max_;
    }

    double delta_v_min = a_min * T;
    double delta_v_max = a_max * T;
    for (int i = 0; i < Nc - 1; ++i) {
        lowerBound[2 * Nc + i] = delta_v_min;
        upperBound[2 * Nc + i] = delta_v_max;
    }

    double delta_omega_min = alpha_min * T;
    double delta_omega_max = alpha_max * T;
    for (int i = 0; i < Nc - 1; ++i) {
        lowerBound[offset_ang + i] = delta_omega_min;
        upperBound[offset_ang + i] = delta_omega_max;
    }

    // ------------------- 求解器设置与调用 ------------------- //
    OsqpEigen::Solver solver;
    solver.data()->setNumberOfVariables(n_variables);
    solver.data()->setNumberOfConstraints(n_constraints);

    Eigen::SparseMatrix<double> hessianMatrix = H_dense.sparseView();
    if (!solver.data()->setHessianMatrix(hessianMatrix)) {
        ROS_ERROR("Failed to set Hessian matrix");
        return;
    }
    if (!solver.data()->setGradient(f)) {
        ROS_ERROR("Failed to set gradient");
        return;
    }
    if (!solver.data()->setLinearConstraintsMatrix(constraintMatrix)) {
        ROS_ERROR("Failed to set constraint matrix");
        return;
    }
    if (!solver.data()->setLowerBound(lowerBound)) {
        ROS_ERROR("Failed to set lower bound");
        return;
    }
    if (!solver.data()->setUpperBound(upperBound)) {
        ROS_ERROR("Failed to set upper bound");
        return;
    }

    solver.settings()->setVerbosity(false);
    solver.settings()->setAlpha(1.0);
    solver.settings()->setWarmStart(true);

    if (!solver.initSolver()) {
        ROS_ERROR("Failed to initialize OSQP solver");
        return;
    }
    if (!solver.solve()) {
        ROS_WARN("OSQP solver failed to find an optimal solution");
        return;
    }

    // ------------------- 控制指令发布 ------------------- //
    Eigen::VectorXd solution = solver.getSolution();
    double cmd_v = solution[0];
    double cmd_omega = solution[1];

    if (config["control"].as<bool>()) {
        geometry_msgs::Twist twist_cmd;
        twist_cmd.linear.x = clamp(cmd_v, v_min_, v_max_);
        twist_cmd.angular.z = clamp(cmd_omega, omega_min_, omega_max_);
        twist_cmd_pub_.publish(twist_cmd);
    }

    // ------------------- 预测轨迹发布 ------------------- //
    nav_msgs::Path predict_path;
    predict_path.header.stamp = ros::Time::now();
    predict_path.header.frame_id = "odom";

    Eigen::Vector3d x_pred = x0;
    for (int i = 0; i < Nc; ++i) {
        double v = solution[2 * i];
        double omega = solution[2 * i + 1];

        double phi = x_pred(2);
        x_pred(0) += T * v * cos(phi);
        x_pred(1) += T * v * sin(phi);
        x_pred(2) += T * omega;

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

    // ------------------- 参考轨迹发布 ------------------- //
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = ros::Time::now();

    for (int i = 0; i < Np; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = ref(3 * i + 0);
        pose.pose.position.y = ref(3 * i + 1);
        pose.pose.position.z = 0.2;

        double yaw = ref(3 * i + 2);
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
    ROS_INFO("Lateral error: %.3f m, Heading error: %.3f rad, Curvature radius: %.2f m, Ref velocity: %.2f m/s", 
             lateral_error, heading_error, curvature_radius, v_ref(0));
}