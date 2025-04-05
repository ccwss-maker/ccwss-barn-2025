#include "A_Star_Planning_Node.hpp"
#include <cmath>

AStarPlanningNode::AStarPlanningNode(std::shared_ptr<TFSubscriberNode> tf_subscriber_node) :
    tf_subscriber_node_(tf_subscriber_node),
    have_goal_(false)
{
    ros::NodeHandle nh;
    
    config_yaml_path = "/jackal_ws/src/navigation/config/config.yaml";
    
    // 初始化目标位置订阅者
    scan_sub_ = nh.subscribe("/front/scan", 1, &AStarPlanningNode::scanCallback, this);
    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &AStarPlanningNode::goalCallback, this);
    
    // 初始化 move_base ActionServer
    action_server_ = std::make_unique<actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>>(
        nh, 
        "move_base", 
        boost::bind(&AStarPlanningNode::executeGoalCallback, this, _1), 
        false // 不自动开始
    );
    action_server_->start();
    ROS_INFO("MoveBase ActionServer started");

    // 初始化发布者
    goal_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker_goal", 10);
    a_star_path_pub_ = nh.advertise<astar_msgs::AStarPathArray>("/A_Star_Planned_Path", 10);
    a_star_path_rviz_pub_ = nh.advertise<nav_msgs::Path>("/A_Star_Planned_Path_Rviz", 10);
    a_star_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map", 10);
    a_star_map_relaxed_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map_Relaxed", 10);
    a_start_map_origin_pub = nh.advertise<nav_msgs::OccupancyGrid>("/A_Star_Map_Origin", 10);


    Matrix Base_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "base_link");
    Eigen::Vector3d Base_To_Odom_Translation = Base_To_Odom_Matrix.Translation_Read();
    double roll, pitch, yaw;
    tf::Quaternion transform_q = Base_To_Odom_Matrix.Quaternion_Read();
    tf::Matrix3x3(transform_q).getRPY(roll, pitch, yaw);
    vehicle_pose.x = Base_To_Odom_Translation.x();
    vehicle_pose.y = Base_To_Odom_Translation.y();
    vehicle_pose.yaw = yaw;

    ROS_INFO("Trajectory planning node initialized and started");
}

void AStarPlanningNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

    if (!have_goal_)  {
        return;
    }
    Laser_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "front_laser");     
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

    double car_length = config["Car_Length"].as<double>();
    double car_width = config["Car_Width"].as<double>();

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
    
    // === 2. 移除过期障碍点（如30秒未被再次看到）===
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

    planTrajectory();
}

void AStarPlanningNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
    // 更新目标
    current_goal_.pose.position = goal_msg->pose.position;
    current_goal_.pose.orientation = goal_msg->pose.orientation;
    have_goal_ = true;
    // 每次收到目标点时都发布目标点标记
    publishGoalMarker();
}

void AStarPlanningNode::executeGoalCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
    // 更新目标点
    current_goal_.pose.position = goal->target_pose.pose.position;
    current_goal_.pose.orientation = goal->target_pose.pose.orientation;
    have_goal_ = true;
    // 每次收到目标点时都发布目标点标记
    publishGoalMarker();
}


void AStarPlanningNode::planTrajectory() {
    // 如果没有目标点，无法进行规划
    if (!have_goal_) {
        return;
    }
    
    // 获取目标点在odom坐标系中的位置
    Eigen::Vector2d goal_position(current_goal_.pose.position.x, current_goal_.pose.position.y);
    Eigen::Vector2d vehicle_xy(vehicle_pose.x, vehicle_pose.y);
    
    // 计算到目标的当前距离
    double distance = (goal_position - vehicle_xy).norm();
    
    Eigen::Vector4d goal_in_odom;
    goal_in_odom << goal_position.x(), goal_position.y(), 0.0, 1.0;

    double grid_resolution_meters = config["resolution_meters"].as<double>();
    map_size.xmin = fmin(fmin(obstacle_points_matrix_.row(0).minCoeff(), current_goal_.pose.position.x),map_size.xmin);
    map_size.xmax = fmax(fmax(obstacle_points_matrix_.row(0).maxCoeff(), current_goal_.pose.position.x),map_size.xmax);
    map_size.ymin = fmin(fmin(obstacle_points_matrix_.row(1).minCoeff(), current_goal_.pose.position.y),map_size.ymin);
    map_size.ymax = fmax(fmax(obstacle_points_matrix_.row(1).maxCoeff(), current_goal_.pose.position.y),map_size.ymax);

    bool mapOriginChanged = std::abs(map_size.xmin - last_path_origin_xmin_) > 1e-6 ||
                            std::abs(map_size.ymin - last_path_origin_ymin_) > 1e-6;

    if (!last_path_.empty() && mapOriginChanged) {
        last_path_ = remapPathToNewMapOrigin(
            last_path_,
            last_path_origin_xmin_, last_path_origin_ymin_,
            map_size.xmin, map_size.ymin, grid_resolution_meters);
    
        last_path_origin_xmin_ = map_size.xmin;
        last_path_origin_ymin_ = map_size.ymin;
    }
    
    // 获取车辆尺寸和安全边距参数
    double car_length = config["Car_Length"].as<double>();
    double car_width = config["Car_Width"].as<double>();
    double safety_hor = config["Safety_Hor"].as<double>();
    
    // 计算起点和终点的栅格坐标
    std::pair<int, int> start(
        static_cast<int>(std::round((vehicle_pose.x - map_size.xmin) / grid_resolution_meters)),
        static_cast<int>(std::round((vehicle_pose.y - map_size.ymin) / grid_resolution_meters))
    );
    
    std::pair<int, int> goal(
        static_cast<int>(std::round((goal_in_odom.x() - map_size.xmin) / grid_resolution_meters)),
        static_cast<int>(std::round((goal_in_odom.y() - map_size.ymin) / grid_resolution_meters))
    );

    auto origin_map = generateInflatedGridMap(obstacle_points_matrix_,
                                                map_size.xmin, map_size.xmax,
                                                map_size.ymin, map_size.ymax,
                                                grid_resolution_meters,
                                                0, 0, 0);
                       
    auto relaxed_grid_map = generateInflatedGridMap(obstacle_points_matrix_,
        map_size.xmin, map_size.xmax,
        map_size.ymin, map_size.ymax,
        grid_resolution_meters,
        car_length, car_width, 0.05);

    auto grid_map = generateInflatedGridMap(obstacle_points_matrix_,
        map_size.xmin, map_size.xmax,
        map_size.ymin, map_size.ymax,
        grid_resolution_meters,
        car_length, car_width, safety_hor);
    // 确保起点和终点是可通行的
    grid_map[start.second][start.first] = 0;
    grid_map[goal.second][goal.first] = 0;

    relaxed_grid_map[start.second][start.first] = 0;
    relaxed_grid_map[goal.second][goal.first] = 0;
    
    ;
    // 尝试复用 last_path_：使用“放宽版”地图，只考虑车身本体，无安全冗余
    bool path_found = false;

    if (!last_path_.empty() &&
        !isPathColliding(last_path_, relaxed_grid_map, map_size.xmin, map_size.ymin, grid_resolution_meters)) {
        // ROS_INFO("Reusing previous path under relaxed check (safety_hor = 0.0)");
        a_star_path_ = last_path_;
        path_found = true;
    } else 
    {
        double safety_step = 0.02;
        double current_safety = safety_hor;
        double best_safety = -1.0;
        bool path_found = false;

        while (current_safety >= 0.0) {
            grid_map = generateInflatedGridMap(obstacle_points_matrix_,
                                                map_size.xmin, map_size.xmax,
                                                map_size.ymin, map_size.ymax,
                                                grid_resolution_meters,
                                                car_length, car_width, current_safety);

            grid_map[start.second][start.first] = 0;
            grid_map[goal.second][goal.first] = 0;

            a_star_path_ = AStarSearch(grid_map, start, goal, grid_resolution_meters);

            if (!a_star_path_.empty()) {
                // ROS_INFO("A* succeeded with safety_hor = %.2f", current_safety);
                best_safety = current_safety;
                last_path_ = a_star_path_;
                path_found = true;
                break;
            } else {
                // ROS_WARN("A* failed with safety_hor = %.2f, trying smaller", current_safety);
                current_safety -= safety_step;
            }
        }

        if (path_found) {
            // ROS_INFO("Final path chosen with safety_hor = %.2f", best_safety);
        } else {
            // ROS_ERROR("A* failed under all safety margins.");
        }
    }
    
    // 检查路径是否为空
    if (last_path_.empty()) {
        // ROS_WARN("Path planning failed: no valid path found.");
    } else {
        publishLastAStarPath(); 
        publishAStarPathRviz(last_path_, map_size.xmin, map_size.ymin, grid_resolution_meters);
    }

    
    publishAStarGridMapRviz(origin_map, map_size.xmin, map_size.ymin, grid_resolution_meters, start, goal, a_start_map_origin_pub);
    publishAStarGridMapRviz(relaxed_grid_map, map_size.xmin, map_size.ymin, grid_resolution_meters, start, goal, a_star_map_relaxed_pub);
    publishAStarGridMapRviz(grid_map, map_size.xmin, map_size.ymin, grid_resolution_meters, start, goal, a_star_map_pub);
}

std::vector<A_Star_Path_> AStarPlanningNode::remapPathToNewMapOrigin(
    const std::vector<A_Star_Path_>& old_path,
    double old_xmin, double old_ymin,
    double new_xmin, double new_ymin,
    double resolution)
{
    std::vector<A_Star_Path_> new_path;

    if (old_path.empty()) return new_path;

    // === 构造 3x3 坐标变换矩阵 ===
    Eigen::Matrix3d old_to_world = Eigen::Matrix3d::Identity();
    old_to_world(0, 0) = resolution;
    old_to_world(1, 1) = resolution;
    old_to_world(0, 2) = old_xmin;
    old_to_world(1, 2) = old_ymin;

    Eigen::Matrix3d world_to_new = Eigen::Matrix3d::Identity();
    world_to_new(0, 0) = 1.0 / resolution;
    world_to_new(1, 1) = 1.0 / resolution;
    world_to_new(0, 2) = -new_xmin / resolution;
    world_to_new(1, 2) = -new_ymin / resolution;

    Eigen::Matrix3d transform = world_to_new * old_to_world;

    // === 构造旧路径矩阵 ===
    Eigen::MatrixXd old_mat(3, old_path.size());  // 每列是一个点的齐次坐标
    for (size_t i = 0; i < old_path.size(); ++i) {
        old_mat(0, i) = old_path[i].position.x();
        old_mat(1, i) = old_path[i].position.y();
        old_mat(2, i) = 1.0;
    }

    // === 计算新路径矩阵 ===
    Eigen::MatrixXd new_mat = transform * old_mat;

    // === 构造新路径 ===
    for (size_t i = 0; i < old_path.size(); ++i) {
        A_Star_Path_ pt = old_path[i];
        int new_x = static_cast<int>(std::round(new_mat(0, i)));
        int new_y = static_cast<int>(std::round(new_mat(1, i)));
        pt.position = Eigen::Vector2d(new_x, new_y);
        new_path.push_back(pt);
    }

    return new_path;
}



void AStarPlanningNode::publishLastAStarPath() {
    if (last_path_.empty()) {
        ROS_WARN("No path to publish.");
        return;
    }

    astar_msgs::AStarPathArray path_array;
    path_array.header.stamp = ros::Time::now();
    path_array.header.frame_id = "odom";

    double resolution = config["resolution_meters"].as<double>();

    for (const auto& pt : last_path_) {
        astar_msgs::AStarPath path_msg;

        // 将 grid 坐标转换为实际世界坐标（meter）
        path_msg.position.x = map_size.xmin + pt.position.x() * resolution;
        path_msg.position.y = map_size.ymin + pt.position.y() * resolution;
        path_msg.position.z = pt.yaw;

        path_array.paths.push_back(path_msg);
    }
    path_array.rush_sign = rush_sign;
    a_star_path_pub_.publish(path_array);
}


std::vector<std::vector<int>> AStarPlanningNode::generateInflatedGridMap(
    const Eigen::MatrixXd& obstacle_points,
    double xmin, double xmax, double ymin, double ymax,
    double resolution,
    double car_length, double car_width,
    double safety_margin) {

    int grid_width = static_cast<int>(std::round((xmax - xmin) / resolution)) + 1;
    int grid_height = static_cast<int>(std::round((ymax - ymin) / resolution)) + 1;

    std::vector<std::vector<int>> inflated_map(grid_height, std::vector<int>(grid_width, 0));

    double inflation_radius = std::max(car_length / 2.0, car_width / 2.0) + safety_margin;
    int inflation_radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution));

    for (int i = 0; i < obstacle_points.cols(); ++i) {
        double x = obstacle_points(0, i);
        double y = obstacle_points(1, i);

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

bool AStarPlanningNode::isPathColliding(
    const std::vector<A_Star_Path_>& path,
    const std::vector<std::vector<int>>& grid_map,
    double xmin, double ymin, double resolution)
{
    int height = grid_map.size();
    int width = grid_map[0].size();

    for (const auto& point : path) {
        int grid_x = static_cast<int>(point.position.x());
        int grid_y = static_cast<int>(point.position.y());

        // 越界视为碰撞
        if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
            return true;
        }

        // 栅格值为1表示障碍，认为路径发生碰撞
        if (grid_map[grid_y][grid_x] != 0) {
            return true;
        }
    }

    return false;
}

std::vector<A_Star_Path_> AStarPlanningNode::AStarSearch(const std::vector<std::vector<int>>& grid_map, 
                                                         const std::pair<int, int>& start, 
                                                         const std::pair<int, int>& goal, 
                                                         double grid_resolution_meters) 
{
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::vector<std::vector<bool>> closed_list(grid_map.size(), std::vector<bool>(grid_map[0].size(), false));
    std::vector<A_Star_Path_> a_star_path;

    // === 从配置读取参数 ===
    bool enable_penalty = false;
    double penalty_weight = 0.2;
    int penalty_radius = 2;
    bool enable_debug = false;

    enable_penalty = config["Enable_Obstacle_Penalty"].as<bool>();
    penalty_weight = config["Penalty_Weight"].as<double>();
    penalty_radius = config["Penalty_Radius"].as<int>();

    open_list.emplace(start.first, start.second, 0.0f, 
                      std::hypot(goal.first - start.first, goal.second - start.second), -1, -1);

    // std::vector<std::pair<int, int>> directions = {
    //     {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    //     {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    // };
    std::vector<std::pair<int, int>> directions = {
        {1, 0},   // 向右（东）
        {-1, 0},  // 向左（西）
        {0, 1},   // 向上（北）
        // {0, -1},  // 向下（南）
        {1, 1},   // 右上（东北）
        // {1, -1},  // 右下（东南）
        {-1, 1},  // 左上（西北）
        // {-1, -1}  // 左下（西南）
    };

    std::map<std::pair<int, int>, std::pair<int, int>> came_from;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        if (current.x == goal.first && current.y == goal.second) {
            std::pair<int, int> current_pos = {current.x, current.y};
            std::vector<std::pair<int, int>> path;
            while (current_pos != start) {
                path.push_back(current_pos);
                current_pos = came_from[current_pos];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            for (size_t i = 0; i < path.size(); ++i) {
                A_Star_Path_ a_star_node;
                a_star_node.position = Eigen::Vector2d(path[i].first, path[i].second);
                a_star_node.yaw = (i == 0) ? vehicle_pose.yaw :
                                  atan2(path[i].second - path[i-1].second, path[i].first - path[i-1].first);
                a_star_path.push_back(a_star_node);
            }

            return a_star_path;
        }

        closed_list[current.y][current.x] = true;

        for (const auto& dir : directions) {
            int new_x = current.x + dir.first;
            int new_y = current.y + dir.second;

            if (new_x < 0 || new_x >= grid_map[0].size() || 
                new_y < 0 || new_y >= grid_map.size() ||
                closed_list[new_y][new_x]) {
                continue;
            }

            if (grid_map[new_y][new_x] != 0) {
                continue;
            }

            float base_cost = (dir.first == 0 || dir.second == 0) ? 1.0f : 1.414f;
            float obstacle_penalty = 0.0f;

            if (enable_penalty) {
                for (int dy = -penalty_radius; dy <= penalty_radius; ++dy) {
                    for (int dx = -penalty_radius; dx <= penalty_radius; ++dx) {
                        int cx = new_x + dx;
                        int cy = new_y + dy;
                        if (cx >= 0 && cx < grid_map[0].size() && 
                            cy >= 0 && cy < grid_map.size() &&
                            grid_map[cy][cx] == 1) {
                            obstacle_penalty += penalty_weight;
                        }
                    }
                }
            }

            float new_g_cost = current.g_cost + base_cost + obstacle_penalty;
            float new_h_cost = std::hypot(goal.first - new_x, goal.second - new_y);

            std::pair<int, int> next_node = {new_x, new_y};
            if (came_from.find(next_node) == came_from.end()) {
                came_from[next_node] = {current.x, current.y};
                open_list.emplace(new_x, new_y, new_g_cost, new_h_cost, current.x, current.y);
            }
        }
    }

    // ROS_WARN("A* failed to find a path");
    return a_star_path;
}


void AStarPlanningNode::publishAStarPathRviz(const std::vector<A_Star_Path_>& path,
                                             double x_min, double y_min,
                                             double grid_resolution_meters) {
    if (!config["Publish_A_Star_Path"].as<bool>() || path.empty()) return;

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();;
    path_msg.header.frame_id = "odom";

    for (const auto& node : path) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();;
        pose.header.frame_id = "odom";

        double real_x = x_min + node.position.x() * grid_resolution_meters;
        double real_y = y_min + node.position.y() * grid_resolution_meters;

        pose.pose.position.x = real_x;
        pose.pose.position.y = real_y;
        pose.pose.position.z = 0.0;

        tf::Quaternion q;
        q.setRPY(0, 0, node.yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path_msg.poses.push_back(pose);
    }

    a_star_path_rviz_pub_.publish(path_msg);
}

void AStarPlanningNode::publishAStarGridMapRviz(const std::vector<std::vector<int>>& grid_map,
                                                double x_min, double y_min,
                                                double grid_resolution_meters,
                                                const std::pair<int, int>& start,
                                                const std::pair<int, int>& goal,
                                                ros::Publisher& grid_pub_) {
    if (!config["Publish_A_Star_Map"].as<bool>()) return;

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

void AStarPlanningNode::publishGoalMarker() {
    visualization_msgs::Marker marker;
    
    // 设置基本属性
    marker.header.frame_id = "odom";  // 使用里程计坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal";  // 使用简单的命名空间
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置位置和姿态
    marker.pose = current_goal_.pose;
    marker.pose.orientation = current_goal_.pose.orientation;
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
    text_marker.pose = current_goal_.pose;
    text_marker.pose.position.z += 0.7;  // 在球体上方显示文本
    text_marker.pose.orientation = current_goal_.pose.orientation;
    
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