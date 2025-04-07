#include "Mapping.hpp"
#include <algorithm>
MappingNode::MappingNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node) :
    tf_subscriber_node_(tf_subscriber_node)
{
    ros::NodeHandle nh;
    
    config_yaml_path = "/jackal_ws/src/navigation/config/config.yaml";
    
    // 初始化同步订阅者
    scan_sub_ = nh.subscribe("/front/scan", 1, &MappingNode::scanCallback, this);
    
    // 初始化目标位置订阅者
    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MappingNode::goalCallback, this);

    // 初始化 move_base ActionServer
    action_server_ = std::make_unique<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>>(
        nh, 
        "move_base", 
        boost::bind(&MappingNode::executeGoalCallback, this, _1), 
        false // 不自动开始
    );
    action_server_->start();
    ROS_INFO("MoveBase ActionServer started");

    // 初始化发布者
    goal_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker_goal", 10);
    map_origin_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map_Origin", 10);
    map_relaxed_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map_Relaxed", 10);
    map_safety_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map", 10);
    ROS_INFO("Mapping node initialized and started");

    mapOriginChanged = false;
    ready = false;


    Matrix Base_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "base_link");
    Eigen::Vector3d Base_To_Odom_Translation = Base_To_Odom_Matrix.Translation_Read();
    double roll, pitch, yaw;
    tf::Quaternion transform_q = Base_To_Odom_Matrix.Quaternion_Read();
    tf::Matrix3x3(transform_q).getRPY(roll, pitch, yaw);
    vehicle_pose.x = Base_To_Odom_Translation.x();
    vehicle_pose.y = Base_To_Odom_Translation.y();
    vehicle_pose.yaw = yaw;
}

void MappingNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    if (!goal_state_.have_goal_)  {
        return;
    }
    
    Matrix Laser_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "front_laser");     
    // 获取配置参数
    config = YAML::LoadFile(config_yaml_path);

    // 存储激光点
    std::vector<Eigen::Vector3d> raw_points;

    // 计算有效激光点并转换到激光坐标系下
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        if (std::isfinite(range)) {
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            raw_points.emplace_back(x, y, 0.0);
        }
    }

    if (raw_points.empty()) {
        ROS_WARN_THROTTLE(5.0, "No valid laser points found");
        return;
    }

    // 构造齐次变换矩阵（4x4）
    Eigen::Matrix4d transform_matrix = Laser_To_Odom_Matrix.Rotation_Translation_Read();

    rush_sign = true;
    for (const auto& pt : raw_points) {
        if (pt.x() > config["rush_distance"].as<double>()) {
            rush_sign = false;
            break;
        }
    }
    
    ros::Time now = ros::Time::now();
    // === 1. 将当前帧点云转换为 odom 坐标下的点，并加入历史障碍点 ===
    for (size_t i = 0; i < raw_points.size(); ++i) {
        Eigen::Vector4d p_laser;
        p_laser << raw_points[i], 1.0;  // 齐次坐标
        Eigen::Vector4d p_odom = transform_matrix * p_laser;
        Eigen::Vector2d obs_xy = p_odom.head<2>();
        historical_obstacle_points_.emplace_back(obs_xy, now);
    }
    
    // === 2. 移除过期障碍点（如60秒未被再次看到）===
    ros::Duration decay(10.0);
    auto it = historical_obstacle_points_.begin();
    while (it != historical_obstacle_points_.end()) {
        if ((now - it->second) > decay) {
            it = historical_obstacle_points_.erase(it);
        } else {
            ++it;
        }
    }
    
    // === 3. 构造 obstacle_points_matrix_ 供地图构建 ===
    obstacle_points_matrix_ = Eigen::MatrixXd(2, historical_obstacle_points_.size());
    for (size_t i = 0; i < historical_obstacle_points_.size(); ++i) {
        obstacle_points_matrix_.col(i) = historical_obstacle_points_[i].first;
    }


    // 获取目标点在odom坐标系中的位置
    Eigen::Vector2d goal_position(goal_state_.current_goal_.pose.position.x, goal_state_.current_goal_.pose.position.y);

    Eigen::Vector4d goal_in_odom;
    goal_in_odom << goal_position.x(), goal_position.y(), 0.0, 1.0;

    grid_resolution_meters = config["resolution_meters"].as<double>();
    map_size.xmin = fmin(fmin(obstacle_points_matrix_.row(0).minCoeff(), goal_state_.current_goal_.pose.position.x),map_size.xmin);
    map_size.xmax = fmax(fmax(obstacle_points_matrix_.row(0).maxCoeff(), goal_state_.current_goal_.pose.position.x),map_size.xmax);
    map_size.ymin = fmin(fmin(obstacle_points_matrix_.row(1).minCoeff(), goal_state_.current_goal_.pose.position.y),map_size.ymin);
    map_size.ymax = fmax(fmax(obstacle_points_matrix_.row(1).maxCoeff(), goal_state_.current_goal_.pose.position.y),map_size.ymax);

    mapOriginChanged = std::abs(map_size.xmin - last_path_origin_xmin_) > 1e-6 ||
                       std::abs(map_size.ymin - last_path_origin_ymin_) > 1e-6;

    // 获取车辆尺寸和安全边距参数
    car_length = config["Car_Length"].as<double>();
    car_width = config["Car_Width"].as<double>();
    safety_hor = config["Safety_Hor"].as<double>();
    
    // 计算起点和终点的栅格坐标
    std::pair<int, int> start(
        static_cast<int>(std::round((vehicle_pose.x - map_size.xmin) / grid_resolution_meters)),
        static_cast<int>(std::round((vehicle_pose.y - map_size.ymin) / grid_resolution_meters))
    );
    
    std::pair<int, int> goal(
        static_cast<int>(std::round((goal_in_odom.x() - map_size.xmin) / grid_resolution_meters)),
        static_cast<int>(std::round((goal_in_odom.y() - map_size.ymin) / grid_resolution_meters))
    );

    origin_map = generateInflatedGridMap(map_size.xmin, map_size.xmax,
                                        map_size.ymin, map_size.ymax,
                                        grid_resolution_meters,
                                        0, 0, 0);
                       
    relaxed_grid_map = generateInflatedGridMap(map_size.xmin, map_size.xmax,
                                            map_size.ymin, map_size.ymax,
                                            grid_resolution_meters,
                                            car_length, car_width, 0.05);

    grid_map = generateInflatedGridMap(map_size.xmin, map_size.xmax,
                                        map_size.ymin, map_size.ymax,
                                        grid_resolution_meters,
                                        car_length, car_width, safety_hor);
    // 确保起点和终点是可通行的
    origin_map[start.second][start.first] = 0;
    origin_map[goal.second][goal.first] = 0;
    
    relaxed_grid_map[start.second][start.first] = 0;
    relaxed_grid_map[goal.second][goal.first] = 0;

    grid_map[start.second][start.first] = 0;
    grid_map[goal.second][goal.first] = 0;

    publishGridMapRviz(origin_map, map_size.xmin, map_size.ymin, grid_resolution_meters, start, goal, map_origin_pub);
    publishGridMapRviz(relaxed_grid_map, map_size.xmin, map_size.ymin, grid_resolution_meters, start, goal, map_relaxed_pub);
    publishGridMapRviz(grid_map, map_size.xmin, map_size.ymin, grid_resolution_meters, start, goal, map_safety_pub);

    ready = true;
}

void MappingNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
    // 更新目标
    goal_state_.current_goal_.pose.position = goal_msg->pose.position;
    goal_state_.current_goal_.pose.orientation = goal_msg->pose.orientation;
    goal_state_.have_goal_ = true;
    // 每次收到目标点时都发布目标点标记
    publishGoalMarker();
}

void MappingNode::executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
    // 更新目标点
    goal_state_.current_goal_.pose.position = goal->target_pose.pose.position;
    goal_state_.current_goal_.pose.orientation = goal->target_pose.pose.orientation;
    goal_state_.have_goal_ = true;
    // 每次收到目标点时都发布目标点标记
    publishGoalMarker();
}

void MappingNode::publishGoalMarker() {
    visualization_msgs::Marker marker;
    
    // 设置基本属性
    marker.header.frame_id = "odom";  // 使用里程计坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal";  // 使用简单的命名空间
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置位置和姿态
    marker.pose = goal_state_.current_goal_.pose;
    marker.pose.orientation = goal_state_.current_goal_.pose.orientation;
    // 设置大小
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    
    // 设置颜色 (红色，半透明)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    // 设置持续时间 (0表示一直存在)
    marker.lifetime = ros::Duration(0);
    
    // 添加文本标签
    visualization_msgs::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "goal";  // 使用相同的命名空间
    text_marker.id = 1;  // 使用不同的ID
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    
    // 文本位置 (在目标点上方)
    text_marker.pose = goal_state_.current_goal_.pose;
    text_marker.pose.position.z += 0.7;  // 在球体上方显示文本
    text_marker.pose.orientation = goal_state_.current_goal_.pose.orientation;
    
    // 文本大小
    text_marker.scale.z = 0.3;  // 文本高度
    
    // 文本颜色 (白色)
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    // 文本内容
    text_marker.text = "Goal";
    
    // 设置持续时间 (0表示一直存在)
    text_marker.lifetime = ros::Duration(0);
    
    // 发布标记
    goal_marker_pub_.publish(marker);
    goal_marker_pub_.publish(text_marker);
}

std::vector<std::vector<int>> MappingNode::generateInflatedGridMap(
    double xmin, double xmax, double ymin, double ymax,
    double resolution,
    double car_length, double car_width,
    double safety_margin) {

    int grid_width = static_cast<int>(std::round((xmax - xmin) / resolution)) + 1;
    int grid_height = static_cast<int>(std::round((ymax - ymin) / resolution)) + 1;

    std::vector<std::vector<int>> inflated_map(grid_height, std::vector<int>(grid_width, 0));

    double inflation_radius = std::max(car_length / 2.0, car_width / 2.0) + safety_margin;
    int inflation_radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution));

    for (int i = 0; i < obstacle_points_matrix_.cols(); ++i) {
        double x = obstacle_points_matrix_(0, i);
        double y = obstacle_points_matrix_(1, i);

        if (x >= xmin && x < xmax && y >= ymin && y < ymax) {
            int grid_x = static_cast<int>(std::round((x - xmin) / resolution));
            int grid_y = static_cast<int>(std::round((y - ymin) / resolution));

            for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
                    double dist = std::sqrt(dx * dx + dy * dy) * resolution;
                    if (dist <= inflation_radius) {
                        int nx = grid_x + dx;
                        int ny = grid_y + dy;
                        if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height) {
                            inflated_map[ny][nx] = 1;
                        }
                    }
                }
            }
        }
    }

    return inflated_map;
}


void MappingNode::publishGridMapRviz(const std::vector<std::vector<int>>& grid_map,
                                    double x_min, double y_min,
                                    double grid_resolution_meters,
                                    const std::pair<int, int>& start,
                                    const std::pair<int, int>& goal,
                                    ros::Publisher& grid_pub_) {
    if (!config["Publish_Map"].as<bool>()) return;

    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "odom";
    grid.header.stamp = ros::Time::now();

    grid.info.resolution = grid_resolution_meters;
    grid.info.width = grid_map[0].size();
    grid.info.height = grid_map.size();
    grid.info.origin.position.x = x_min;
    grid.info.origin.position.y = y_min;
    grid.info.origin.position.z = -0.1;

    grid.data.resize(grid.info.width * grid.info.height, 0);

    for (unsigned int y = 0; y < grid_map.size(); ++y) {
        for (unsigned int x = 0; x < grid_map[y].size(); ++x) {
            unsigned int index = y * grid.info.width + x;

            if (x == start.first && y == start.second) {
                grid.data[index] = 25;
            } else if (x == goal.first && y == goal.second) {
                grid.data[index] = 50;
            } else if (grid_map[y][x] == 1) {
                grid.data[index] = 100;
            } else {
                grid.data[index] = 0;
            }
        }
    }

    grid_pub_.publish(grid);
}