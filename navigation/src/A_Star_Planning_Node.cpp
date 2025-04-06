#include "A_Star_Planning_Node.hpp"
#include <cmath>

AStarPlanningNode::AStarPlanningNode(std::shared_ptr<MappingNode> mapping_node) : mapping_node_(mapping_node)
{
    ros::NodeHandle nh;
    
    config_yaml_path = "/jackal_ws/src/navigation/config/config.yaml";
    
    // 创建定时器
    timer_ = nh.createTimer(ros::Duration(0.02), &AStarPlanningNode::TimerCallback, this);

    // 初始化发布者
    a_star_path_pub_ = nh.advertise<astar_msgs::AStarPathArray>("/A_Star_Planned_Path", 10);
    a_star_path_rviz_pub_ = nh.advertise<nav_msgs::Path>("/A_Star_Planned_Path_Rviz", 10);

    ROS_INFO("Trajectory planning node initialized and started");
}

void AStarPlanningNode::TimerCallback(const ros::TimerEvent& event)
{
    if (!mapping_node_->goal_state_.have_goal_ || !mapping_node_->ready) {
        return;
    }
    config = YAML::LoadFile(config_yaml_path);
    vehicle_pose = mapping_node_->vehicle_pose;
    map_size =  mapping_node_->map_size;
    double grid_resolution_meters = mapping_node_->grid_resolution_meters;
    if (!last_path_.empty() && mapping_node_->mapOriginChanged) {
        last_path_ = remapPathToNewMapOrigin(
            last_path_,
            last_path_origin_xmin_, last_path_origin_ymin_,
            map_size.xmin, map_size.ymin, grid_resolution_meters);
    
        last_path_origin_xmin_ = map_size.xmin;
        last_path_origin_ymin_ = map_size.ymin;
    }
    
    // 计算起点和终点的栅格坐标
    std::pair<int, int> start(
        static_cast<int>(std::round((vehicle_pose.x - map_size.xmin) / grid_resolution_meters)),
        static_cast<int>(std::round((vehicle_pose.y - map_size.ymin) / grid_resolution_meters))
    );
    
    std::pair<int, int> goal(
        static_cast<int>(std::round((mapping_node_->goal_state_.current_goal_.pose.position.x - map_size.xmin) / grid_resolution_meters)),
        static_cast<int>(std::round((mapping_node_->goal_state_.current_goal_.pose.position.y - map_size.ymin) / grid_resolution_meters))
    );
    
    // 尝试复用 last_path_：使用“放宽版”地图，只考虑车身本体，无安全冗余
    bool path_found = false;

    if (!last_path_.empty() &&
        !isPathColliding(last_path_, mapping_node_->relaxed_grid_map, map_size.xmin, map_size.ymin, grid_resolution_meters)) {
        // ROS_INFO("Reusing previous path under relaxed check (safety_hor = 0.0)");
        a_star_path_ = last_path_;
        path_found = true;
    } else 
    {
        double safety_step = 0.02;
        double current_safety = mapping_node_->safety_hor;
        double best_safety = -1.0;
        bool path_found = false;

        while (current_safety >= 0.0) {
            auto grid_map = mapping_node_->generateInflatedGridMap(map_size.xmin, map_size.xmax,
                                                                    map_size.ymin, map_size.ymax,
                                                                    grid_resolution_meters,
                                                                    mapping_node_->car_length, mapping_node_->car_width, current_safety);

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

        // if (path_found) {
        //     ROS_INFO("Final path chosen with safety_hor = %.2f", best_safety);
        // } else {
        //     ROS_ERROR("A* failed under all safety margins.");
        // }
    }
    
    // 检查路径是否为空
    if (last_path_.empty()) {
        // ROS_WARN("Path planning failed: no valid path found.");
    } else {
        publishLastAStarPath(); 
        publishAStarPathRviz(last_path_, map_size.xmin, map_size.ymin, grid_resolution_meters);
    }
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

    for (const auto& pt : last_path_) {
        astar_msgs::AStarPath path_msg;

        // 将 grid 坐标转换为实际世界坐标（meter）
        path_msg.position.x = map_size.xmin + pt.position.x() * mapping_node_->grid_resolution_meters;
        path_msg.position.y = map_size.ymin + pt.position.y() * mapping_node_->grid_resolution_meters;
        path_msg.position.z = pt.yaw;

        path_array.paths.push_back(path_msg);
    }
    path_array.rush_sign = mapping_node_->rush_sign;
    a_star_path_pub_.publish(path_array);
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