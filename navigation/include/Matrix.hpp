#ifndef MATRIX_HPP
#define MATRIX_HPP

#include "ros/ros.h"
#include "Eigen/Dense"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

class Matrix {
public:
    Matrix() = default;
    void Rotation_Set(double roll, double pitch, double yaw);
    void Translation_Set(double x, double y, double z);
    void Rotation_Translation_Set();
    Eigen::Matrix3d Rotation_Read();
    Eigen::Vector3d Translation_Read();
    Eigen::MatrixXd Rotation_Translation_Read();
    tf::Quaternion Quaternion_Read();
    void setFromTF(geometry_msgs::TransformStamped transformStamped);
private:
    tf::Quaternion Quaternion; // 四元数
    Eigen::Matrix3d Rotation; // 旋转矩阵3*3
    Eigen::Vector3d Translation; // 平移矩阵3*1
    Eigen::MatrixXd Rotation_Translation; // 旋转平移矩阵3*4
};
#endif // MATRIX_HPP