#include "ros/ros.h"
#include "MSG_Process.hpp"
#include "Pure_Pursuit.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_system");
    ros::NodeHandle nh;
    
    // 创建TF监听器节点
    auto tf_subscriber_node = std::make_shared<TFSubscriberNode>();

    auto msg_process_node = std::allocate_shared<MsgProcessNode>(
        Eigen::aligned_allocator<PurePursuitNode>(), tf_subscriber_node);

    auto pure_pursuit_node = std::allocate_shared<PurePursuitNode>(
        Eigen::aligned_allocator<PurePursuitNode>(), tf_subscriber_node, msg_process_node);

    // 设置多线程回调队列
    ros::MultiThreadedSpinner spinner(4); // 使用4个线程
    spinner.spin(); // 启动并运行回调处理
    
    return 0;
}