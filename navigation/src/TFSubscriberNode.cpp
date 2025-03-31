#include "TFSubscriberNode.hpp"

TFSubscriberNode::TFSubscriberNode() {
    tfListener_ = std::make_shared<tf::TransformListener>();
    
    // 等待 odom 到 laser_center 的变换变得可用
    while (ros::ok()) {
        
        if (tfListener_->waitForTransform("odom", "front_laser", ros::Time(0), ros::Duration(0.1))) {
            ROS_INFO("Suceessfully obtained transform from odom to front_laser");
            break; // 成功获取变换，退出循环
        }
        
        // 短暂休眠再重试
        ros::Duration(0.1).sleep();
    }
}

Matrix TFSubscriberNode::Matrix_Read(const std::string &target_frame, const std::string &source_frame)
{
    Matrix Transformation_Matrix;
    if (tfListener_->waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.1))) {
        try {
            geometry_msgs::TransformStamped transformStamped;
            tf::StampedTransform stamped_transform;
            tfListener_->lookupTransform(target_frame, source_frame, ros::Time(0), stamped_transform);
            
            // 转换tf::StampedTransform到geometry_msgs::TransformStamped
            transformStamped.header.stamp = stamped_transform.stamp_;
            transformStamped.header.frame_id = stamped_transform.frame_id_;
            transformStamped.child_frame_id = stamped_transform.child_frame_id_;
            
            transformStamped.transform.translation.x = stamped_transform.getOrigin().x();
            transformStamped.transform.translation.y = stamped_transform.getOrigin().y();
            transformStamped.transform.translation.z = stamped_transform.getOrigin().z();
            
            transformStamped.transform.rotation.x = stamped_transform.getRotation().x();
            transformStamped.transform.rotation.y = stamped_transform.getRotation().y();
            transformStamped.transform.rotation.z = stamped_transform.getRotation().z();
            transformStamped.transform.rotation.w = stamped_transform.getRotation().w();
            
            Transformation_Matrix.setFromTF(transformStamped);
        } catch (tf::TransformException &ex) {
            ROS_WARN("Could not transform %s to %s after becoming available: %s", 
                     target_frame.c_str(), source_frame.c_str(), ex.what());
        }
    }
    return Transformation_Matrix;
}