#include "DenseGridMap.hpp"
#include <limits>
#include <cmath>

DenseGridMap::DenseGridMap(double res, int initial_size) :
    resolution(res), 
    origin_x(0), 
    origin_y(0), 
    initial_map_size(initial_size),
    center_offset(initial_size / 2),
    current_min_x(center_offset), 
    current_min_y(center_offset), 
    current_max_x(center_offset), 
    current_max_y(center_offset)
{
    // 安全地初始化网格
    try {
        grid = Eigen::MatrixXi::Zero(initial_map_size, initial_map_size);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize grid: %s", e.what());
        throw;
    }
    
    // ROS_INFO("Initialized grid with size %dx%d, center at (%d, %d)", 
    //          initial_map_size, initial_map_size, center_offset, center_offset);
}

void DenseGridMap::setResolution(double res) {
    resolution = res;
}

void DenseGridMap::setOrigin(double x, double y) {
    origin_x = x;
    origin_y = y;
}

void DenseGridMap::updateMap(const Eigen::MatrixXd& new_obstacles) {
    // 安全检查：确保输入有效
    if (new_obstacles.cols() == 0) {
        ROS_WARN("No obstacles provided to updateMap");
        return;
    }

    // 将所有输入点转换为网格坐标
    for (int i = 0; i < new_obstacles.cols(); ++i) {
        double x = new_obstacles(0, i);
        double y = new_obstacles(1, i);
        
        // 将世界坐标转换为网格坐标
        int grid_x = static_cast<int>(std::round((x - origin_x) / resolution)) + center_offset;
        int grid_y = static_cast<int>(std::round((y - origin_y) / resolution)) + center_offset;
        
        // 计算在当前地图中的相对坐标
        int local_x = grid_x - current_min_x;
        int local_y = grid_y - current_min_y;
        
        // 标记障碍物及其膨胀（1格）
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int nx = local_x + dx;
                int ny = local_y + dy;
                
                // 确保在网格范围内
                if (nx >= 0 && nx < grid.cols() && ny >= 0 && ny < grid.rows()) {
                    grid(ny, nx) = 1;
                }
            }
        }
    }
}

void DenseGridMap::resizeMap(int new_min_x, int new_min_y, int new_max_x, int new_max_y) {
    // 确保新的边界有效
    if (new_min_x > new_max_x || new_min_y > new_max_y) {
        ROS_ERROR("Invalid map boundaries in resizeMap: (%d,%d)-(%d,%d)", 
                  new_min_x, new_min_y, new_max_x, new_max_y);
        return;
    }

    // 重新计算宽度和高度
    int width = new_max_x - new_min_x + 1;
    int height = new_max_y - new_min_y + 1;

    // 安全检查：确保新大小是正的
    if (width <= 0 || height <= 0) {
        ROS_ERROR("Invalid map size in resizeMap: width=%d, height=%d", width, height);
        return;
    }
    
    // ROS_INFO("Resizing map from (%d,%d)-(%d,%d) to (%d,%d)-(%d,%d)",
    //          current_min_x, current_min_y, current_max_x, current_max_y,
    //          new_min_x, new_min_y, new_max_x, new_max_y);

    // 创建新的网格，初始化为零
    Eigen::MatrixXi new_grid = Eigen::MatrixXi::Zero(height, width);
    
    // 复制原地图内容到新地图，只复制有效范围内的数据
    int overlap_min_x = std::max(current_min_x, new_min_x);
    int overlap_min_y = std::max(current_min_y, new_min_y);
    int overlap_max_x = std::min(current_max_x, new_max_x);
    int overlap_max_y = std::min(current_max_y, new_max_y);
    
    // 只有当有重叠区域时才复制
    if (overlap_min_x <= overlap_max_x && overlap_min_y <= overlap_max_y) {
        for (int old_y = overlap_min_y; old_y <= overlap_max_y; ++old_y) {
            for (int old_x = overlap_min_x; old_x <= overlap_max_x; ++old_x) {
                // 计算在原地图中的相对坐标
                int src_x = old_x - current_min_x;
                int src_y = old_y - current_min_y;
                
                // 计算在新地图中的相对坐标
                int dst_x = old_x - new_min_x;
                int dst_y = old_y - new_min_y;
                
                // 确保坐标有效
                if (src_x >= 0 && src_x < grid.cols() && src_y >= 0 && src_y < grid.rows() &&
                    dst_x >= 0 && dst_x < width && dst_y >= 0 && dst_y < height) {
                    new_grid(dst_y, dst_x) = grid(src_y, src_x);
                }
            }
        }
    }
    
    // 更新网格和边界
    grid = new_grid;
    current_min_x = new_min_x;
    current_min_y = new_min_y;
    current_max_x = new_max_x;
    current_max_y = new_max_y;

    // ROS_INFO("Map successfully resized to: width=%d, height=%d", width, height);
}

void DenseGridMap::getMapBounds(int& min_x, int& min_y, int& max_x, int& max_y) const {
    min_x = current_min_x;
    min_y = current_min_y;
    max_x = current_max_x;
    max_y = current_max_y;
}

std::pair<int, int> DenseGridMap::worldToGrid(double x, double y) const {
    int grid_x = static_cast<int>(std::round((x - origin_x) / resolution)) + center_offset;
    int grid_y = static_cast<int>(std::round((y - origin_y) / resolution)) + center_offset;
    return {grid_x, grid_y};
}

std::pair<double, double> DenseGridMap::gridToWorld(int grid_x, int grid_y) const {
    double world_x = (grid_x - center_offset) * resolution + origin_x;
    double world_y = (grid_y - center_offset) * resolution + origin_y;
    return {world_x, world_y};
}

int DenseGridMap::getCellValue(double world_x, double world_y) const {
    auto [grid_x, grid_y] = worldToGrid(world_x, world_y);
    
    // 计算在当前地图中的相对坐标
    int local_x = grid_x - current_min_x;
    int local_y = grid_y - current_min_y;
    
    if (local_x >= 0 && local_x < grid.cols() && local_y >= 0 && local_y < grid.rows()) {
        return grid(local_y, local_x);
    }
    return 0;
}

void DenseGridMap::setCellValue(double world_x, double world_y, int value) {
    auto [grid_x, grid_y] = worldToGrid(world_x, world_y);
    
    // 计算在当前地图中的相对坐标
    int local_x = grid_x - current_min_x;
    int local_y = grid_y - current_min_y;
    
    if (local_x >= 0 && local_x < grid.cols() && local_y >= 0 && local_y < grid.rows()) {
        grid(local_y, local_x) = value;
    }
}

void DenseGridMap::publishMap(ros::Publisher& pub, const std::string& frame_id,
                              double resolution,
                              const std::pair<int, int>& start, const std::pair<int, int>& goal) {
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = frame_id;
    grid_msg.header.stamp = ros::Time::now();

    // 计算地图尺寸
    int width = current_max_x - current_min_x + 1;
    int height = current_max_y - current_min_y + 1;

    // 安全检查：确保地图尺寸有效
    if (width <= 0 || height <= 0) {
        ROS_ERROR("Invalid map size in publishMap: width=%d, height=%d", width, height);
        return;
    }

    // 设置地图信息
    grid_msg.info.resolution = resolution;
    grid_msg.info.width = width;
    grid_msg.info.height = height;

    // 计算世界坐标的原点
    auto [wx, wy] = gridToWorld(current_min_x, current_min_y);
    grid_msg.info.origin.position.x = wx;
    grid_msg.info.origin.position.y = wy;
    grid_msg.info.origin.position.z = -0.1;

    // 初始化地图数据
    grid_msg.data.resize(width * height, 0);

    // 填充地图数据
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            if (grid(y, x) == 1) {
                grid_msg.data[idx] = 100;
            }
        }
    }

    // 标记起点和终点
    int start_local_x = start.first - current_min_x;
    int start_local_y = start.second - current_min_y;
    int goal_local_x = goal.first - current_min_x;
    int goal_local_y = goal.second - current_min_y;
    
    if (start_local_x >= 0 && start_local_x < width && 
        start_local_y >= 0 && start_local_y < height) {
        int start_idx = start_local_y * width + start_local_x;
        grid_msg.data[start_idx] = 25;
    }
    
    if (goal_local_x >= 0 && goal_local_x < width && 
        goal_local_y >= 0 && goal_local_y < height) {
        int goal_idx = goal_local_y * width + goal_local_x;
        grid_msg.data[goal_idx] = 50;
    }

    // 发布地图
    pub.publish(grid_msg);
}

void DenseGridMap::boundedInflation(double inflation_radius) {
    // 计算膨胀半径对应的网格单元数
    int radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution));
    
    // ROS_INFO("Performing bounded inflation with radius %.2f cells (%d)", 
    //          inflation_radius, radius_cells);
    
    // 创建一个新网格来存储膨胀结果，保持与原始地图相同大小
    Eigen::MatrixXi inflated_grid = Eigen::MatrixXi::Zero(grid.rows(), grid.cols());
    
    // 遍历当前网格寻找障碍物点
    for (int y = 0; y < grid.rows(); ++y) {
        for (int x = 0; x < grid.cols(); ++x) {
            if (grid(y, x) == 1) {
                // 对每个障碍物点进行膨胀
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                        // 计算距离
                        double dist = std::sqrt(dx*dx + dy*dy) * resolution;
                        if (dist <= inflation_radius) {
                            // 计算膨胀点在地图中的坐标
                            int inflate_y = y + dy;
                            int inflate_x = x + dx;
                            
                            // 确保在网格范围内，如果超出则丢弃
                            if (inflate_x >= 0 && inflate_x < grid.cols() && 
                                inflate_y >= 0 && inflate_y < grid.rows()) {
                                inflated_grid(inflate_y, inflate_x) = 1;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // 更新网格，替换为膨胀后的结果
    grid = inflated_grid;
}