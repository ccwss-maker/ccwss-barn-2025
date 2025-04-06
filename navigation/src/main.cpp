#include "ros/ros.h"
#include "Mapping.hpp"
#include "A_Star_Planning_Node.hpp"
#include "First_Optimization_Node.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_system");
    ros::NodeHandle nh;
    
    // 创建TF监听器节点
    auto tf_subscriber_node = std::make_shared<TFSubscriberNode>();
    
    auto mapping_node = std::allocate_shared<MappingNode>(
            Eigen::aligned_allocator<MappingNode>(), tf_subscriber_node);
    // 创建轨迹规划节点并传入TF监听器
    auto route_planning_node = std::make_shared<AStarPlanningNode>(mapping_node);
    
    
    auto first_optimization_node = std::make_shared<FirstOptimizationNode>();

    // 设置多线程回调队列
    ros::MultiThreadedSpinner spinner(4); // 使用4个线程
    spinner.spin(); // 启动并运行回调处理
    
    return 0;
}