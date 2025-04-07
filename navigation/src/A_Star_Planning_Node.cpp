#include "A_Star_Planning_Node.hpp"
#include <cmath>
#include <limits>

AStarPlanningNode::AStarPlanningNode(std::shared_ptr<MappingNode> mapping_node, std::shared_ptr<TFSubscriberNode> tf_subscriber_node) 
    : mapping_node_(mapping_node), tf_subscriber_node_(tf_subscriber_node)
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
    Matrix Base_To_Odom_Matrix = tf_subscriber_node_->Matrix_Read("odom", "base_link");
    Eigen::Vector3d Base_To_Odom_Translation = Base_To_Odom_Matrix.Translation_Read();
    double roll, pitch, yaw;
    tf::Quaternion transform_q = Base_To_Odom_Matrix.Quaternion_Read();
    tf::Matrix3x3(transform_q).getRPY(roll, pitch, yaw);
    vehicle_pose.x = Base_To_Odom_Translation.x();
    vehicle_pose.y = Base_To_Odom_Translation.y();
    vehicle_pose.yaw = yaw;

    map_size = mapping_node_->map_size;
    double grid_resolution_meters = mapping_node_->grid_resolution_meters;

    // 计算起点和终点的栅格坐标
    std::pair<int, int> start = mapping_node_->origin_map.worldToGrid(vehicle_pose.x, vehicle_pose.y);
    std::pair<int, int> goal = mapping_node_->origin_map.worldToGrid(
        mapping_node_->goal_state_.current_goal_.pose.position.x,
        mapping_node_->goal_state_.current_goal_.pose.position.y
    );
    
    // 尝试复用 last_path_：使用"放宽版"地图，只考虑车身本体，无安全冗余
    if (!last_path_.empty() && 
        !isPathColliding(last_path_, mapping_node_->relaxed_grid_map) && 
        findClosestPathDistance(last_path_, vehicle_pose.x, vehicle_pose.y) < config["Reuse_Path_Distance"].as<double>()) {
        // ROS_INFO("Reusing previous path under relaxed check");
        a_star_path_ = last_path_;
    } else {
        double safety_step = 0.02;
        double current_safety = mapping_node_->safety_hor;
        double best_safety = -1.0;

        while (current_safety >= 0.0) {
            // 创建临时地图用于A*搜索，从原始地图复制
            DenseGridMap temp_map = mapping_node_->origin_map;
            
            // 使用新的boundedInflation接口直接在地图上执行膨胀
            temp_map.boundedInflation(std::max(mapping_node_->car_length/2.0, mapping_node_->car_width/2.0) + current_safety);
            
            // 确保起点和终点可通行
            temp_map.setCellValue(vehicle_pose.x, vehicle_pose.y, 0);
            temp_map.setCellValue(mapping_node_->goal_state_.current_goal_.pose.position.x, 
                                 mapping_node_->goal_state_.current_goal_.pose.position.y, 0);

            a_star_path_ = AStarSearch(temp_map, start, goal, grid_resolution_meters);

            if (!a_star_path_.empty()) {
                // ROS_INFO("A* succeeded with safety_hor = %.2f", current_safety);
                best_safety = current_safety;
                last_path_ = a_star_path_;
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
        ROS_WARN("Path planning failed: no valid path found.");
    } else {
        publishLastAStarPath(last_path_, mapping_node_->rush_sign); 
        publishAStarPathRviz(last_path_, grid_resolution_meters);
    }
}

double AStarPlanningNode::findClosestPathDistance(std::vector<A_Star_Path_> path, double vehicle_x, double vehicle_y) 
{
    if (path.empty()) {
        return -1.0;
    }

    double min_distance = std::numeric_limits<double>::max();

    // 遍历路径中的所有点
    for (size_t i = 0; i < path.size(); ++i) {
        // 将路径点的栅格坐标转换为世界坐标
        std::pair<double, double> world_coords = mapping_node_->origin_map.gridToWorld(
            path[i].position.x(), path[i].position.y());
        
        // 计算距离
        double dx = world_coords.first - vehicle_x;
        double dy = world_coords.second - vehicle_y;
        double distance = std::hypot(dx, dy);
        
        // 更新最近距离
        if (distance < min_distance) {
            min_distance = distance;
        }
    }

    return min_distance;
}

bool AStarPlanningNode::isPathColliding(
    const std::vector<A_Star_Path_>& path,
    DenseGridMap& grid_map)
{
    for (const auto& point : path) {
        auto world_coords = grid_map.gridToWorld(point.position.x(), point.position.y());
        // 检查该点在地图中的值
        int grid_value = grid_map.getCellValue(world_coords.first, world_coords.second);
        // 栅格值为非0表示障碍，认为路径发生碰撞
        if (grid_value != 0) {
            return true;
        }
    }

    return false;
}

std::vector<A_Star_Path_> AStarPlanningNode::AStarSearch(
    DenseGridMap& grid_map, 
    const std::pair<int, int>& start, 
    const std::pair<int, int>& goal, 
    double grid_resolution_meters) 
{
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::map<std::pair<int, int>, bool> closed_list;
    std::vector<A_Star_Path_> a_star_path;

    // === 从配置读取参数 ===
    bool enable_penalty = config["Enable_Obstacle_Penalty"].as<bool>();
    double penalty_weight = config["Penalty_Weight"].as<double>();
    int penalty_radius = config["Penalty_Radius"].as<int>();
    open_list.emplace(start.first, start.second, 0.0f, 
                      std::hypot(goal.first - start.first, goal.second - start.second), -1, -1);

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

    // 获取地图边界
    int map_min_x, map_min_y, map_max_x, map_max_y;
    grid_map.getMapBounds(map_min_x, map_min_y, map_max_x, map_max_y);

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        // 目标检查
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
                a_star_path.push_back(a_star_node);
            }

            return a_star_path;
        }

        // 当前节点标记为已访问
        std::pair<int, int> current_pos = {current.x, current.y};
        closed_list[current_pos] = true;

        // 探索邻居
        for (const auto& dir : directions) {
            int new_x = current.x + dir.first;
            int new_y = current.y + dir.second;
            
            // 确保在地图边界内
            if (new_x < map_min_x || new_x > map_max_x || new_y < map_min_y || new_y > map_max_y) {
                continue;
            }
            
            std::pair<int, int> next_pos = {new_x, new_y};

            // 跳过已访问的节点
            if (closed_list.find(next_pos) != closed_list.end()) {
                continue;
            }

            // 使用世界坐标检查该点是否是障碍物
            std::pair<double, double> world_coords = grid_map.gridToWorld(new_x, new_y);
            if (grid_map.getCellValue(world_coords.first, world_coords.second) != 0) {
                continue;
            }

            // 计算基础代价和障碍物惩罚
            float base_cost = (dir.first == 0 || dir.second == 0) ? 1.0f : 1.414f;
            float obstacle_penalty = 0.0f;

            if (enable_penalty) {
                for (int dy = -penalty_radius; dy <= penalty_radius; ++dy) {
                    for (int dx = -penalty_radius; dx <= penalty_radius; ++dx) {
                        int cx = new_x + dx;
                        int cy = new_y + dy;
                        
                        // 确保在地图范围内
                        if (cx >= map_min_x && cx <= map_max_x && cy >= map_min_y && cy <= map_max_y) {
                            std::pair<double, double> nearby_world = grid_map.gridToWorld(cx, cy);
                            if (grid_map.getCellValue(nearby_world.first, nearby_world.second) == 1) {
                                obstacle_penalty += penalty_weight;
                            }
                        }
                    }
                }
            }

            float new_g_cost = current.g_cost + base_cost + obstacle_penalty;
            float new_h_cost = std::hypot(goal.first - new_x, goal.second - new_y);

            // 检查节点是否已经在来源映射中
            if (came_from.find(next_pos) == came_from.end()) {
                came_from[next_pos] = current_pos;
                open_list.emplace(new_x, new_y, new_g_cost, new_h_cost, current.x, current.y);
            }
        }
    }

    // ROS_WARN("A* failed to find a path");
    return a_star_path;
}

void AStarPlanningNode::publishLastAStarPath(std::vector<A_Star_Path_>& path, bool rush_sign) {
    if (path.empty()) {
        ROS_WARN("No path to publish.");
        return;
    }

    astar_msgs::AStarPathArray path_array;
    path_array.header.stamp = ros::Time::now();
    path_array.header.frame_id = "odom";

    for (const auto& pt : path) {
        astar_msgs::AStarPath path_msg;

        // 将栅格坐标转换为实际世界坐标
        std::pair<double, double> world_coords = mapping_node_->origin_map.gridToWorld(pt.position.x(), pt.position.y());
        path_msg.position.x = world_coords.first;
        path_msg.position.y = world_coords.second;
        path_msg.position.z = 0.0;
        path_array.paths.push_back(path_msg);
    }
    
    path_array.rush_sign = rush_sign;
    a_star_path_pub_.publish(path_array);
}

void AStarPlanningNode::publishAStarPathRviz(const std::vector<A_Star_Path_>& path,
                                           double grid_resolution_meters) {
    if (!config["Publish_A_Star_Path"].as<bool>() || path.empty()) return;

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "odom";

    for (const auto& node : path) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "odom";

        // 将栅格坐标转换为实际世界坐标
        std::pair<double, double> world_coords = mapping_node_->origin_map.gridToWorld(node.position.x(), node.position.y());
        pose.pose.position.x = world_coords.first;
        pose.pose.position.y = world_coords.second;
        pose.pose.position.z = 0.0;

        // 设置方向
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        path_msg.poses.push_back(pose);
    }

    a_star_path_rviz_pub_.publish(path_msg);
}