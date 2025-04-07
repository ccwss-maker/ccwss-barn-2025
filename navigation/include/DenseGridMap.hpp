#ifndef DENSE_GRID_MAP_HPP
#define DENSE_GRID_MAP_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>

class DenseGridMap {
public:
    DenseGridMap(double resolution = 0.1, int initial_size = 2000);
    
    // 设置地图分辨率
    void setResolution(double res);
    
    // 设置地图原点
    void setOrigin(double x, double y);
    
    // 更新地图大小和障碍物
    void updateMap(const Eigen::MatrixXd& new_obstacles);
    
    // 调整地图大小 - 公有方法
    void resizeMap(int new_min_x, int new_min_y, int new_max_x, int new_max_y);
    
    // 获取当前地图的实际边界
    void getMapBounds(int& min_x, int& min_y, int& max_x, int& max_y) const;
    
    // 世界坐标和栅格坐标转换
    std::pair<int, int> worldToGrid(double x, double y) const;
    std::pair<double, double> gridToWorld(int grid_x, int grid_y) const;
    
    // 膨胀方法 - 修改后不需要外部传入地图
    void boundedInflation(double inflation_radius);
    
    // 发布地图
    void publishMap(ros::Publisher& pub, const std::string& frame_id,
                    double resolution,
                    const std::pair<int, int>& start, const std::pair<int, int>& goal);
    
    // 获取单元格值
    int getCellValue(double world_x, double world_y) const;
    void setCellValue(double world_x, double world_y, int value);

    // 获取原点
    std::pair<double, double> getOrigin() const { return {origin_x, origin_y}; }

    // 网格矩阵 - 公有以便直接访问
    Eigen::MatrixXi grid;

private:
    // 地图参数
    double resolution;
    double origin_x, origin_y;
    int center_offset;
    
    // 地图边界
    int current_min_x, current_min_y, current_max_x, current_max_y;
    
    // 初始地图大小
    int initial_map_size;
};

#endif // DENSE_GRID_MAP_HPP