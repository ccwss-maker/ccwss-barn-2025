#include "MPC.hpp"
#include "math.h"
#include <OsqpEigen/OsqpEigen.h>

#include <algorithm>

template <typename T>
T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}


MPCNode::MPCNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node, std::shared_ptr<MsgProcessNode> msg_process_node) :
    tf_subscriber_node_(tf_subscriber_node), msg_process_node_(msg_process_node)
{
    // 初始化订阅者
    ros::NodeHandle nh;
    config_yaml_path = "/jackal_ws/src/tracking/config/config.yaml";
    YAML::Node config = YAML::LoadFile(config_yaml_path);
    double T = config["T"].as<double>();
    timer_ = nh.createTimer(ros::Duration(T), &MPCNode::TimerCallback, this);

    // 初始化发布者
    twist_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 10);
    predict_path_pub_ = nh.advertise<nav_msgs::Path>("/mpc/predict_path", 1);
    ref_path_pub_ = nh.advertise<nav_msgs::Path>("/mpc/reference_path", 1);

}

double MPCNode::computeObstacleCost(const Eigen::Vector2d& pos_world,
    const nav_msgs::OccupancyGrid& grid_map,
    const cv::Mat& dist_map,
    double w_obs,
    double sigma)
{
    YAML::Node config = YAML::LoadFile(config_yaml_path);
    double origin_x = grid_map.info.origin.position.x;
    double origin_y = grid_map.info.origin.position.y;
    double resolution = grid_map.info.resolution;

    int x = static_cast<int>((pos_world.x() - origin_x) / resolution);
    int y = static_cast<int>((pos_world.y() - origin_y) / resolution);

    if (x < 0 || x >= dist_map.cols || y < 0 || y >= dist_map.rows)
    return 0.0;

    float d = dist_map.at<float>(y, x);

    double ret = w_obs * std::exp(-d / sigma);

    ret = std::min(ret, config["max_obs"].as<double>());
    // std::cout << "ret: " << ret << std::endl;

    
    return ret;  // 越靠近障碍，惩罚越大
}


void MPCNode::TimerCallback(const ros::TimerEvent& event)
{
    Matrix Base_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "base_link");
    Matrix Odom_To_Base_Matrix = tf_subscriber_node_->Matrix_Read("base_link", "odom");

    YAML::Node config = YAML::LoadFile(config_yaml_path);
    Eigen::MatrixXd control_points = msg_process_node_->control_points;
    cv::Mat dist_map = msg_process_node_->dist_map;
    if (control_points.cols() == 0) {
        geometry_msgs::Twist twist_cmd;
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.0;
        twist_cmd_pub_.publish(twist_cmd);
        return;
    }

    T = config["T"].as<double>();
    v_min_ = config["control_limits"]["v_min"].as<double>();
    v_max_ = config["control_limits"]["v_max"].as<double>();
    omega_min_ = config["control_limits"]["omega_min"].as<double>();
    omega_max_ = config["control_limits"]["omega_max"].as<double>();
    double a_min = config["control_limits"]["a_min"].as<double>();
    double a_max = config["control_limits"]["a_max"].as<double>();
    double alpha_min = config["control_limits"]["alpha_min"].as<double>();
    double alpha_max = config["control_limits"]["alpha_max"].as<double>();

    w_x_ = config["weights"]["x"].as<double>();
    w_y_ = config["weights"]["y"].as<double>();
    w_phi_ = config["weights"]["phi"].as<double>();
    w_v_ = config["weights"]["v"].as<double>();
    w_omega_ = config["weights"]["omega"].as<double>();
    double w_vref = config["weights"]["v_ref"].as<double>();
    double w_obs = config["weights"]["obs"].as<double>();
    double sigma = config["sigma"].as<double>();
    int lookahead = config["lookahead"].as<int>();

    Eigen::Vector3d x0;
    x0(0) = Base_To_Odom_Matrix.Translation_Read()(0);
    x0(1) = Base_To_Odom_Matrix.Translation_Read()(1);
    double roll, pitch, yaw;
    tf::Quaternion transform_q = Base_To_Odom_Matrix.Quaternion_Read();
    tf::Matrix3x3(transform_q).getRPY(roll, pitch, yaw);
    x0(2) = yaw;
    // std::cout << "x0: " << x0.transpose() << std::endl<< std::endl;


    if(msg_process_node_->rush_sign)    rush_sign = true;
    // rush_sign = msg_process_node_->rush_sign;
    // rush_sign = true;
    if (rush_sign) {
        // std::cout << "rush!!!!" << std::endl;
        Eigen::Vector3d goal = control_points.col(control_points.cols() - 1);
        // 构造目标点齐次坐标（只有位置）
        Eigen::Vector4d goal_odom;
        goal_odom << goal(0), goal(1), 0.0, 1.0;

        Eigen::Matrix4d T_odom_to_base = Odom_To_Base_Matrix.Rotation_Translation_Read();
        Eigen::Vector4d goal_base_homogeneous = T_odom_to_base * goal_odom;
        Eigen::Vector3d goal_base = goal_base_homogeneous.head<3>();
        double dx_body = goal_base(0);
        double dy_body = goal_base(1);

        // 控制
        double k_v = config["rush"]["k_v"].as<double>();
        double k_w = config["rush"]["k_w"].as<double>();
        double v_cmd = k_v * dx_body;
        double omega_cmd = k_w * std::atan2(dy_body, dx_body);

        v_cmd = clamp(v_cmd, v_min_, v_max_);
        omega_cmd = clamp(omega_cmd, omega_min_, omega_max_);

        geometry_msgs::Twist twist_cmd;
        twist_cmd.linear.x = v_cmd;
        twist_cmd.angular.z = omega_cmd;
        twist_cmd_pub_.publish(twist_cmd);
        return;
    }

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

    
    double alpha = config["alpha"].as<double>();
    int offset = config["offset"].as<int>();
    int Np_max = config["Np"].as<int>();
    int Np_min = config["Np_min"].as<int>();
    int window_size = config["window_size"].as<int>();  // 例如20
    
    int end_idx = std::min(nearest_idx + window_size, (int)control_points.cols() - 1);
    
    double curvature_sum = 0.0;
    double max_curvature = 0.0;
    int curvature_count = 0;
    
    for (int i = nearest_idx + 1; i <= end_idx; ++i) {
        double phi1 = control_points(2, i);
        double phi0 = control_points(2, i - 1);
        double dphi = std::atan2(std::sin(phi1 - phi0), std::cos(phi1 - phi0));
        double abs_dphi = std::abs(dphi);
    
        curvature_sum += abs_dphi;
        max_curvature = std::max(max_curvature, abs_dphi);
        curvature_count++;
    }
    
    double avg_curvature = (curvature_count > 0) ? (curvature_sum / curvature_count) : 0.0;
    
    double hybrid_curvature = 0.5 * avg_curvature + 0.5 * max_curvature;
    
    double curvature_sensitivity = config["curvature_sensitivity"].as<double>();
    double curvature_factor = clamp(hybrid_curvature * curvature_sensitivity, 0.0, 1.0);
    
    Np = static_cast<int>(Np_max * (1.0 - curvature_factor) + Np_min * curvature_factor);
    Np = clamp(Np, Np_min, Np_max);
    
    
    // -----------------------------

    int N_start = std::min(nearest_idx + lookahead, (int)control_points.cols() - 1);
    Np = std::min(Np, (int)control_points.cols() - N_start);
    Nc = std::min(config["Nc"].as<int>(), Np);


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

    // Eigen::VectorXd ref(3 * Np);
    // for (int i = 0; i < Np; ++i) {
    //     ref.segment<3>(3 * i) = control_points.col(N_start + i);
    // }

    Eigen::VectorXd ref(3 * Np);

    // 地图信息
    double origin_x = msg_process_node_->grid_map.info.origin.position.x;
    double origin_y = msg_process_node_->grid_map.info.origin.position.y;
    double resolution = msg_process_node_->grid_map.info.resolution;

    // 参数
    double distance_threshold = config["distance_threshold"].as<double>();  // 小于这个距离就推开（单位：米）
    double push_strength = config["push_strength"].as<double>();       // 推开幅度（越大偏得越多）

    for (int i = 0; i < Np; ++i) {
        Eigen::Vector3d pt = control_points.col(N_start + i);
        Eigen::Vector2d pos_world(pt.x(), pt.y());

        int x = static_cast<int>((pos_world.x() - origin_x) / resolution);
        int y = static_cast<int>((pos_world.y() - origin_y) / resolution);

        // 边界检查
        if (x >= 1 && x < dist_map.cols - 1 && y >= 1 && y < dist_map.rows - 1) {
            float dist = dist_map.at<float>(y, x);
            if (dist < distance_threshold) {
                // 计算梯度（Sobel近似）
                float dx = (dist_map.at<float>(y, x + 1) - dist_map.at<float>(y, x - 1)) / (2 * resolution);
                float dy = (dist_map.at<float>(y + 1, x) - dist_map.at<float>(y - 1, x)) / (2 * resolution);
                Eigen::Vector2d grad(dx, dy);

                // 如果梯度不为零，归一化并推开
                if (grad.norm() > 1e-3) {
                    grad.normalize();
                    pos_world += push_strength * grad;
                }
            }
        }

        ref.segment<3>(3 * i) << pos_world.x(), pos_world.y(), pt.z();  // 保留原来的 yaw
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3 * Np, 3 * Np);
    for (int i = 0; i < Np; ++i) {
        Q.block<3, 3>(3 * i, 3 * i) = Eigen::DiagonalMatrix<double, 3>(w_x_, w_y_, w_phi_);
    }

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2 * Nc, 2 * Nc);
    for (int i = 0; i < Nc; ++i) {
        R(2 * i, 2 * i) = w_v_;
        R(2 * i + 1, 2 * i + 1) = w_omega_;
    }

    Eigen::VectorXd v_ref = Eigen::VectorXd::Zero(Nc);
    for (int i = 0; i < Nc; ++i) {
        int idx0 = N_start + i;
        int idx_back = std::max(idx0 - offset, 0);
        int idx_forward = std::min(idx0 + offset, (int)control_points.cols() - 1);
        double phi_back = control_points(2, idx_back);
        double phi_forward = control_points(2, idx_forward);
        double dphi = std::atan2(std::sin(phi_forward - phi_back), std::cos(phi_forward - phi_back));
        double v = v_max_ * std::exp(-alpha * std::abs(dphi));
        v_ref(i) = clamp(v, v_min_, v_max_);
    }

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(Nc, 2 * Nc);
    for (int i = 0; i < Nc; ++i) {
        M(i, 2 * i) = 1;
    }
    Eigen::MatrixXd W_vref = Eigen::MatrixXd::Identity(Nc, Nc) * w_vref;

    Eigen::MatrixXd H_dense = B_bar.transpose() * Q * B_bar + R + M.transpose() * W_vref * M;
    Eigen::VectorXd f = B_bar.transpose() * Q * (A_bar * x0 - ref) - M.transpose() * W_vref * v_ref;

    for (int i = 0; i < Np; ++i) {
        Eigen::Vector3d x_est = A_bar.block(3 * i, 0, 3, 3) * x0;
        Eigen::Vector2d pos_xy(x_est(0), x_est(1));
        double obs_cost = computeObstacleCost(pos_xy, msg_process_node_->grid_map, dist_map, w_obs, sigma);
        for (int j = 0; j < Nc; ++j) {
            if (j <= i) {
                f(2 * j) += obs_cost;
            }
        }
    }

    int n_variables = 2 * Nc;
    int n_constraints = 2 * Nc + 2 * (Nc - 1); // 新增角加速度限制
    Eigen::SparseMatrix<double> constraintMatrix(n_constraints, n_variables);

    for (int i = 0; i < 2 * Nc; ++i) {
        constraintMatrix.insert(i, i) = 1.0;
    }

    for (int i = 0; i < Nc - 1; ++i) {
        constraintMatrix.insert(2 * Nc + i, 2 * (i + 1)) = 1.0;
        constraintMatrix.insert(2 * Nc + i, 2 * i) = -1.0;
    }

    int offset_ang = 2 * Nc + (Nc - 1);
    for (int i = 0; i < Nc - 1; ++i) {
        constraintMatrix.insert(offset_ang + i, 2 * (i + 1) + 1) = 1.0;
        constraintMatrix.insert(offset_ang + i, 2 * i + 1) = -1.0;
    }

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

    Eigen::VectorXd solution = solver.getSolution();
    double cmd_v = solution[0];
    double cmd_omega = solution[1];

    if (config["control"].as<bool>()) {
        geometry_msgs::Twist twist_cmd;
        twist_cmd.linear.x = clamp(cmd_v, v_min_, v_max_);
        twist_cmd.angular.z = clamp(cmd_omega, omega_min_, omega_max_);
        twist_cmd_pub_.publish(twist_cmd);
    }

    // std::cout << "cmd_v: " << cmd_v << ", cmd_omega: " << cmd_omega << std::endl;
    nav_msgs::Path predict_path;
    predict_path.header.stamp = ros::Time::now();
    predict_path.header.frame_id = "odom";  // 或 "map"，视你的TF配置而定

    Eigen::Vector3d x_pred = x0;
    for (int i = 0; i < Nc; ++i) {
        double v = solution[2 * i];
        double omega = solution[2 * i + 1];

        // 简单非完整车辆模型预测
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

    nav_msgs::Path ref_path;
    ref_path.header.stamp = ros::Time::now();
    ref_path.header.frame_id = "odom";  // or "map"

    for (int i = 0; i < Np; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = ref_path.header;
        pose.pose.position.x = control_points(0, N_start + i);
        pose.pose.position.y = control_points(1, N_start + i);
        pose.pose.position.z = 0.0;

        double phi = control_points(2, N_start + i);
        tf::Quaternion q;
        q.setRPY(0, 0, phi);
        tf::quaternionTFToMsg(q, pose.pose.orientation);

        ref_path.poses.push_back(pose);
    }

    ref_path_pub_.publish(ref_path);
}
