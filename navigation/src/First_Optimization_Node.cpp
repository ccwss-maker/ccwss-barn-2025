
#include "First_Optimization_Node.hpp"
#include <chrono>
FirstOptimizationNode::FirstOptimizationNode()
{
    ros::NodeHandle nh;
    
    config_yaml_path = "/jackal_ws/src/navigation/config/config.yaml";
    
    // 初始化目标位置订阅者
    a_star_sub_ = nh.subscribe("/A_Star_Planned_Path", 1, &FirstOptimizationNode::pathCallback, this);
    map_sub_ = nh.subscribe("/A_Star_Map_Relaxed", 1, &FirstOptimizationNode::mapCallback, this);
    
    // 创建定时器
    timer_ = nh.createTimer(ros::Duration(0.02), &FirstOptimizationNode::TimerCallback, this);

    First_Optimized_Trajectory_Publisher_ = nh.advertise<initial_optimized_msgs::InitialOptimizedTrajectory>("/First_Optimized_Trajectory", 1);
    First_Opimization_Marker_Publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/First_Optimization_Marker_Array", 1);
    ROS_INFO("First Optimization node initialized and started");

    sloved = false;
}

void FirstOptimizationNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & map)
{
    // 保存地图
    grid_map = *map;

    int width = grid_map.info.width;
    int height = grid_map.info.height;

    // 创建二值地图（0=free，255=occupied）
    cv::Mat binary_map(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            int val = grid_map.data[index];
            binary_map.at<uchar>(y, x) = (val > 50) ? 255 : 0;
        }
    }

    // 计算距离图（以像素为单位）
    cv::Mat dist_pixel;
    cv::distanceTransform(255 - binary_map, dist_pixel, cv::DIST_L2, 5);

    // 转换为实际单位（米），乘以地图分辨率
    dist_map = dist_pixel * grid_map.info.resolution;
}

void FirstOptimizationNode::pathCallback(const astar_msgs::AStarPathArray::ConstPtr & astar_path_)
{
    if(astar_path_->paths.empty()) {
        ROS_ERROR("Received empty A* path");
        return;
    }
    astar_path = *astar_path_;
}

void FirstOptimizationNode::TimerCallback(const ros::TimerEvent& event)
{
    if (astar_path.paths.empty()) {
        // ROS_ERROR("A* path not ready");
        return;
    }

    config = YAML::LoadFile(config_yaml_path);

    Eigen::Vector3d A_Star_Path_front(astar_path.paths.front().position.x,
                                      astar_path.paths.front().position.y,
                                      astar_path.paths.front().position.z);
    Eigen::Vector3d A_Star_Path_back(astar_path.paths.back().position.x,
                                     astar_path.paths.back().position.y,
                                     astar_path.paths.back().position.z);

    A_Star_Path.clear();
    int point_interval = config["A_Star_Point_interval"].as<int>();
    for (int i = 1; i < astar_path.paths.size() - 2; i += (point_interval + 1)) {
        A_Star_Path_ path_point;
        path_point.position = Eigen::Vector3d(astar_path.paths[i].position.x,
                                              astar_path.paths[i].position.y,
                                              astar_path.paths[i].position.z);
        A_Star_Path.push_back(path_point);
    }

    // 判断路径是否改变
    bool path_changed = false;
    if (A_Star_Path_Last.size() != A_Star_Path.size()) {
        path_changed = true;
    } else {
        for (int i = 0; i < A_Star_Path.size(); ++i) {
            if ((A_Star_Path[i].position - A_Star_Path_Last[i].position).norm() > 0.1) {
                path_changed = true;
                break;
            }
        }
    }

    if (path_changed) {
        last_cost = std::numeric_limits<double>::max();  // 重置上次代价
        last_cost_Yaw = std::numeric_limits<double>::max();  // 重置上次代价
        sloved = false;
        std::cout << "Path changed, re-optimizing..." << std::endl;
        Emergency_Brake_Publish();
    }
    A_Star_Path_Last = A_Star_Path;

    // 初始化优化变量
    pieceN = A_Star_Path.size() + 1;
    temporalDim = pieceN;
    spatialDim = 3 * (pieceN - 1);

    Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();
    initState.col(0) = A_Star_Path_front;
    finalState.col(0) = A_Star_Path_back;
    minco.setConditions(initState, finalState, pieceN);

    Eigen::VectorXd x(temporalDim + spatialDim);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

    Eigen::VectorXd T = config["init_time"].as<double>() * Eigen::VectorXd::Ones(pieceN);

    for (int i = 0; i < pieceN - 1; ++i)
        xi.segment(3 * i, 3) = A_Star_Path[i].position;
    backwardT(T, tau);

    weight_time = config["traj_opimiz_weight_time"].as<double>();
    weight_position_x = config["traj_opimiz_weight_position_x"].as<double>();
    weight_position_y = config["traj_opimiz_weight_position_y"].as<double>();
    weight_position_w = config["traj_opimiz_weight_position_w"].as<double>();
    weight_energy_x = config["traj_opimiz_weight_energy_x"].as<double>();
    weight_energy_y = config["traj_opimiz_weight_energy_y"].as<double>();
    weight_energy_w = config["traj_opimiz_weight_energy_w"].as<double>();
    weight_linear_velocity = config["traj_opimiz_weight_linear_velocity"].as<double>();
    weight_angular_velocity = config["traj_opimiz_weight_angular_velocity"].as<double>();
    max_linear_velocity = config["max_linear_velocity"].as<double>();
    max_angular_velocity = config["max_angular_velocity"].as<double>();
    // 优化设置
    LBFGSpp::LBFGSParam<double> param;
    param.epsilon = config["first_optimum_epsilon"].as<double>();
    param.max_iterations = config["first_optimum_max_iterations"].as<int>();
    param.min_step = config["first_optimum_min_step"].as<double>();

    LBFGSpp::LBFGSSolver<double> solver(param);

    std::function<double(const Eigen::VectorXd&, Eigen::VectorXd&)> cost_function =
        [this](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
            return this->ComputeCostAndGradient(x, grad);
        };

    double min_cost = 0.0;
    int niter = 0;
    double cost_Yaw = 999;

    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        niter = solver.minimize(cost_function, x, min_cost);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // ROS_INFO("Initial Optimization Finished in %ld ms, iterations: %d, cost: %f", duration.count(), niter, min_cost);

        forwardT(tau, times);
        forwardP(xi, points);
        minco.setParameters(points, times);
        Eigen::MatrixX3d GradByPoints_Position = Eigen::MatrixX3d::Zero(pieceN - 1, 3);
        Eigen::VectorXd GradByTimes_Position = Eigen::VectorXd::Zero(pieceN);
        cost_Yaw = ComputeCostAndGradient_Position(GradByPoints_Position, GradByTimes_Position);
    } catch (const std::exception& e) {
        ROS_WARN("Optimization failed: %s", e.what());
        return;
    }

    // 判断是否更新轨迹
    if (cost_Yaw < 5 && min_cost < 40)
    {
        if(min_cost < last_cost && cost_Yaw < last_cost_Yaw)
        {
            sloved = true;
            last_cost = min_cost;
            last_cost_Yaw = cost_Yaw;

            InitialOptimizedTrajectory_pub.position.clear();
            InitialOptimizedTrajectory_pub.times.clear();

            geometry_msgs::Vector3 p;
            for (int i = 0; i < pieceN; i++)
            {
                InitialOptimizedTrajectory_pub.times.push_back(times[i]);
            }
            p.x = A_Star_Path_front(0); p.y = A_Star_Path_front(1); p.z = A_Star_Path_front(2);
            InitialOptimizedTrajectory_pub.position.push_back(p);
            for (int i = 0; i < points.cols(); i++) {
                p.x = points(0, i); p.y = points(1, i); p.z = points(2, i);
                InitialOptimizedTrajectory_pub.position.push_back(p);
            }
            p.x = A_Star_Path_back(0); p.y = A_Star_Path_back(1); p.z = A_Star_Path_back(2);
            InitialOptimizedTrajectory_pub.position.push_back(p);

            ROS_INFO("Updated trajectory with better cost_Yaw: %f, cost: %f", cost_Yaw, min_cost);
        }
    }
    else if (last_cost == std::numeric_limits<double>::max()) {
        // 还没成功优化过，fallback 发布原始路径
        ROS_WARN("Fallback: No valid optimized trajectory. Publishing raw path.");
        InitialOptimizedTrajectory_pub.position.clear();
        InitialOptimizedTrajectory_pub.times.clear();

        geometry_msgs::Vector3 p;
        p.x = A_Star_Path_front(0); p.y = A_Star_Path_front(1); p.z = A_Star_Path_front(2);
        InitialOptimizedTrajectory_pub.position.push_back(p);
        for (const auto& pt : A_Star_Path) {
            p.x = pt.position(0); p.y = pt.position(1); p.z = pt.position(2);
            InitialOptimizedTrajectory_pub.position.push_back(p);
        }
        p.x = A_Star_Path_back(0); p.y = A_Star_Path_back(1); p.z = A_Star_Path_back(2);
        InitialOptimizedTrajectory_pub.position.push_back(p);

        for(int i = 0; i < pieceN; ++i)
        {
            InitialOptimizedTrajectory_pub.times.push_back(T(i));
        }

    }

    // 发布轨迹
    // if(sloved) {
        initial_optimized_msgs::InitialOptimizedTrajectory msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = ros::Time::now();
        msg.position = InitialOptimizedTrajectory_pub.position;
        msg.times = InitialOptimizedTrajectory_pub.times;
        msg.rush_sign = astar_path.rush_sign;
        msg.emergency_braking_sign = false;
        First_Optimized_Trajectory_Publisher_.publish(msg);
        if (config["Publish_First_Optimum_Marker"].as<bool>(true)) {
            visualizeTrajectory(msg);
        }
    // }
}


// 代价函数及其梯度计算
double FirstOptimizationNode::ComputeCostAndGradient(const Eigen::VectorXd& params, Eigen::VectorXd& grad) {    
    double cost = 0.0;
    grad.setZero();
    int dimTau = temporalDim; // 时间维度
    int dimXi = spatialDim;   // 空间维度
    Eigen::Map<const Eigen::VectorXd> tau(params.data(), dimTau); // 将参数映射为tau
    Eigen::Map<const Eigen::VectorXd> xi(params.data() + dimTau, dimXi); // 将参数映射为xi
    Eigen::Map<Eigen::VectorXd> gradTau(grad.data(), dimTau); // 将梯度映射为gradTau
    Eigen::Map<Eigen::VectorXd> gradXi(grad.data() + dimTau, dimXi); // 将梯度映射为gradXi
    
    Eigen::MatrixX3d GradByCoeffs = Eigen::MatrixX3d::Zero(6 * pieceN, 3);
    Eigen::VectorXd GradByTimes = Eigen::VectorXd::Zero(pieceN);
    Eigen::Matrix3Xd gradByPoints = Eigen::Matrix3Xd::Zero(3, pieceN - 1);
    Eigen::VectorXd gradByTimes = Eigen::VectorXd::Zero(pieceN);
    forwardT(tau, times); // 计算时间参数
    forwardP(xi, points); // 计算空间参数
    for (int i = 0; i < times.size(); ++i)
    {
        if (times[i] < 1e-3 || !std::isfinite(times[i])) {
            ROS_ERROR("Invalid T[%d] = %f", i, times[i]);
            return 1e10;  // 返回大cost，丢弃这次优化
        }
    }

    minco.setParameters(points, times); // 设置最小曲线参数
    minco.getEnergy(cost, weight_energy_x, weight_energy_y, weight_energy_w);
    minco.getEnergyPartialGradByCoeffs(GradByCoeffs, weight_energy_x, weight_energy_y, weight_energy_w); // ∂E/∂c
    minco.getEnergyPartialGradByTimes(GradByTimes, weight_energy_x, weight_energy_y, weight_energy_w);   // ∂E/∂T
    minco.propogateGrad(GradByCoeffs, GradByTimes, gradByPoints, gradByTimes);
    cost += weight_time * times.sum();
    gradByTimes.array() += weight_time;

    Eigen::MatrixX3d GradByPoints_Position = Eigen::MatrixX3d::Zero(pieceN - 1, 3);
    Eigen::VectorXd GradByTimes_Position = Eigen::VectorXd::Zero(pieceN);
    double cost_Position;
    cost_Position += ComputeCostAndGradient_Position(GradByPoints_Position, GradByTimes_Position);
    gradByPoints += GradByPoints_Position.transpose();
    GradByTimes += GradByTimes_Position;
    cost += cost_Position;

    // 加速度惩罚（软约束）
    for (int i = 0; i < pieceN - 1; i++) {
        Eigen::Vector3d V = minco.b.block((i + 1) * 6 + 1, 0, 1, 3).transpose();
        double linear_v = std::sqrt(V(0) * V(0) + V(1) * V(1));
        double angular_v = std::abs(V(2));
    
        if (linear_v > max_linear_velocity) {
            double diff = linear_v - max_linear_velocity;
            cost += weight_linear_velocity * diff * diff;
            GradByTimes[i] += 2 * weight_linear_velocity * diff;
        }

        // if (angular_v > max_angular_velocity) {
        //     double diff = angular_v - max_angular_velocity;
        //     cost += weight_angular_velocity * diff * diff;
        //     GradByTimes[i] += 2 * weight_angular_velocity * diff;
        // }
    }
    
    backwardGradT(tau, gradByTimes, gradTau);
    backwardGradP(xi, gradByPoints, gradXi);

    
    return cost; // 返回总代价
}

int FirstOptimizationNode::FindClosestAStarPointIndex(const Eigen::Vector3d& query_point) const {
    int closest_index = -1;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < A_Star_Path.size(); ++i) {
        double dist = (query_point - A_Star_Path[i].position).squaredNorm();
        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }

    return closest_index;
}

// 代价函数及其梯度计算
double FirstOptimizationNode::ComputeCostAndGradient_Position(Eigen::MatrixX3d& GradByPoints, Eigen::VectorXd& GradByTimes) {
    double cost = 0.0;
    
    for (int i = 0; i < A_Star_Path.size(); i++) {
        Eigen::Vector3d P = minco.b.block((i + 1) * 6 + 0, 0, 1, 3).transpose();
        Eigen::Vector3d V = minco.b.block((i + 1) * 6 + 1, 0, 1, 3).transpose();
        // int closest_idx = FindClosestAStarPointIndex(P);
        // Eigen::Vector3d closest_point = A_Star_Path[closest_idx].position;
        // Eigen::Vector3d delta = P - closest_point;
        Eigen::Vector3d delta = P - A_Star_Path[i].position;
        Eigen::Vector3d delta_2 = Eigen::Vector3d(delta(0) * delta(0), delta(1) * delta(1), delta(2) * delta(2));
        Eigen::Vector3d C = Eigen::Vector3d(weight_position_x, weight_position_y, weight_position_w);
        GradByPoints.row(i) = 2 * delta.cwiseProduct(C).transpose();
        GradByTimes(i) = GradByPoints.row(i).dot(V);
        cost += delta_2.dot(C);
    }
    return cost;
}

// tau -> T 的安全映射
void FirstOptimizationNode::forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
{
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    
    // 使用双曲正切(tanh)函数来构造一个更平滑、安全的映射
    for (int i = 0; i < sizeTau; i++)
    {
        // 使用 exp(tau) 替代原来的有理函数
        // 这样可以保证 T 总是正数，并且没有除以零的风险
        T(i) = 0.1 + 0.9 * std::exp(0.5 * tau(i));
        
        // 确保 T 有一个最小值
        if (T(i) < 0.1 || !std::isfinite(T(i))) {
            T(i) = 0.1;
        }
        
        // 限制最大值，防止数值溢出
        if (T(i) > 100.0) {
            T(i) = 100.0;
        }
    }
    return;
}

// T -> tau 的安全映射
template <typename EIGENVEC>
void FirstOptimizationNode::backwardT(const Eigen::VectorXd &T, EIGENVEC &tau)
{
    const int sizeT = T.size();
    tau.resize(sizeT);
    
    for (int i = 0; i < sizeT; i++)
    {
        // 确保 T 是正数且大于最小阈值
        double safe_T = std::max(0.1, T(i));
        double log_arg = (safe_T - 0.1) / 0.9;
        if (log_arg <= 0.0) {
            log_arg = 0.001;  // 一个小的正数，避免对数参数为零或负数
        }
        // 使用 log 函数，这是 exp 的逆函数
        tau(i) = 2.0 * std::log(log_arg);
        
        // 限制 tau 的范围，防止极端值
        if (tau(i) > 10.0) {
            tau(i) = 10.0;
        } else if (tau(i) < -10.0) {
            tau(i) = -10.0;
        }
    }
    return;
}

// 梯度的安全映射
template <typename EIGENVEC>
void FirstOptimizationNode::backwardGradT(const Eigen::VectorXd &tau,
                                    const Eigen::VectorXd &gradT,
                                    EIGENVEC &gradTau)
{
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    
    for (int i = 0; i < sizeTau; i++)
    {
        // 使用我们新的映射函数的导数计算梯度变换
        // 对于 T = 0.1 + 0.9 * exp(0.5 * tau)
        // dT/dtau = 0.45 * exp(0.5 * tau)
        // dtau/dT = 1 / (0.45 * exp(0.5 * tau)) = 1 / (0.5 * (T - 0.1))
        
        double T_val = 0.1 + 0.9 * std::exp(0.5 * tau(i));
        double dT_dtau = 0.45 * std::exp(0.5 * tau(i));
        
        // 防止除以零或小值
        if (dT_dtau < 1e-6) {
            dT_dtau = 1e-6;
        }
        
        // 梯度转换
        gradTau(i) = gradT(i) * dT_dtau;
        
        // 限制梯度大小，防止数值爆炸
        double max_grad = 100.0;
        if (std::abs(gradTau(i)) > max_grad) {
            gradTau(i) = (gradTau(i) > 0) ? max_grad : -max_grad;
        }
    }
    return;
}

template <typename EIGENVEC>
void FirstOptimizationNode::backwardGradP(const Eigen::VectorXd &xi,
                                    const Eigen::Matrix3Xd &gradP,
                                    EIGENVEC &gradXi)
{
    const int sizeP = gradP.cols();
    for (int i = 0; i < sizeP; ++i)
    {
        gradXi.segment(3 * i, 3) = gradP.col(i);
    }
    return;
}

void FirstOptimizationNode::forwardP(const Eigen::VectorXd &xi, Eigen::Matrix3Xd &P)
{
    const int sizeP = xi.size() / 3;
    P.resize(3, sizeP);
    for (int i = 0; i < sizeP; i++)
    {
        P.col(i) = xi.segment(3 * i, 3);
    }
    return;
}

template <typename EIGENVEC>
void FirstOptimizationNode::backwardP(const Eigen::Matrix3Xd &P, EIGENVEC &xi)
{
    const int sizeP = P.cols();
    for (int i = 0; i < sizeP; ++i)
    {
        xi.segment(3 * i, 3) = P.col(i);
    }
    return;
}

void FirstOptimizationNode::visualizeTrajectory(initial_optimized_msgs::InitialOptimizedTrajectory InitialOptimizedTrajectory)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "odom";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    First_Opimization_Marker_Publisher_.publish(marker_array);
    marker_array.markers.clear(); // 清空数组以便添加新标记

    // 生成新的轨迹标记
    int ID = 0;

    for (int i = 0; i < InitialOptimizedTrajectory.position.size(); i++) 
    {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "odom";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "trajectory";
        point_marker.type = visualization_msgs::Marker::SPHERE;  // 使用SPHERE类型表示单个点
        point_marker.action = visualization_msgs::Marker::ADD;

        // 为每个点设置唯一的ID
        point_marker.id = ID++;  // 为每个点分配唯一的ID

        // 设置Marker的比例
        point_marker.scale.x = 0.05;  // 点的大小
        point_marker.scale.y = 0.05;
        point_marker.scale.z = 0.05;

        // 设置Marker的颜色
        point_marker.color.r = 0.0;  // 红色分量
        point_marker.color.g = 0.0;  // 绿色分量
        point_marker.color.b = 1.0;  // 蓝色分量
        point_marker.color.a = 1.0;  // 透明度（alpha）

        // 设置点的坐标
        point_marker.pose.position.x = InitialOptimizedTrajectory.position[i].x;
        point_marker.pose.position.y = InitialOptimizedTrajectory.position[i].y;
        point_marker.pose.position.z = 0;
        point_marker.pose.orientation.x = 0.0;
        point_marker.pose.orientation.y = 0.0;
        point_marker.pose.orientation.z = 0.0;
        point_marker.pose.orientation.w = 1.0;

        // 将Marker添加到MarkerArray
        marker_array.markers.push_back(point_marker);
    }

    // 发布新的轨迹标记
    First_Opimization_Marker_Publisher_.publish(marker_array);
}

void FirstOptimizationNode::Emergency_Brake_Publish()
{
    initial_optimized_msgs::InitialOptimizedTrajectory msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = ros::Time::now();
    msg.emergency_braking_sign = true;
    First_Optimized_Trajectory_Publisher_.publish(msg);
}