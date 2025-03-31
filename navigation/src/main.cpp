#include "ros/ros.h"
#include "A_Star_Planning_Node.hpp"
// #include "Mapping.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_system");
    ros::NodeHandle nh;
    
    // 创建TF监听器节点
    auto tf_subscriber_node = std::make_shared<TFSubscriberNode>();
    
    // auto mapping_node = std::allocate_shared<MappingNode>(
    //         Eigen::aligned_allocator<MappingNode>(), tf_subscriber_node);
    // 创建轨迹规划节点并传入TF监听器
    // auto route_planning_node = std::allocate_shared<AStarPlanningNode>(
    //     Eigen::aligned_allocator<AStarPlanningNode>(), tf_subscriber_node ,mapping_node);
    auto route_planning_node = std::allocate_shared<AStarPlanningNode>(
        Eigen::aligned_allocator<AStarPlanningNode>(), tf_subscriber_node);

    // 设置多线程回调队列
    ros::MultiThreadedSpinner spinner(3); // 使用4个线程
    spinner.spin(); // 启动并运行回调处理
    
    return 0;
}