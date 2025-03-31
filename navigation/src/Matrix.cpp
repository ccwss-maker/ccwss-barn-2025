// Matrix.cpp
#include "Matrix.hpp"

void Matrix::Rotation_Set(double roll, double pitch, double yaw)
{
    double sinx = sin(roll);
    double cosx = cos(roll);
    double siny = sin(pitch);
    double cosy = cos(pitch);
    double sinz = sin(yaw);
    double cosz = cos(yaw);
    Eigen::Matrix3d mat_x, mat_y, mat_z;
    mat_x << 1.0, 0.0, 0.0,
             0.0, cosx, sinx,
             0.0, -sinx, cosx;
    mat_y << cosy, 0.0, -siny,
             0.0, 1.0, 0.0,
             siny, 0.0, cosy;
    mat_z << cosz, sinz, 0.0,
             -sinz, cosz, 0.0,
             0.0, 0.0, 1.0;
    Rotation = mat_z * mat_y * mat_x;
}

void Matrix::Translation_Set(double x, double y, double z)
{
    Translation << x, y, z;
}

void Matrix::Rotation_Translation_Set()
{
    Rotation_Translation = Eigen::MatrixXd::Zero(4, 4);
    Rotation_Translation.block<3, 3>(0, 0) = Rotation;
    Rotation_Translation.block<3, 1>(0, 3) = Translation;
    Rotation_Translation(3, 3) = 1;
}

Eigen::Matrix3d Matrix::Rotation_Read()
{
    return Rotation;
}

Eigen::Vector3d Matrix::Translation_Read()
{
    return Translation;
}

Eigen::MatrixXd Matrix::Rotation_Translation_Read()
{
    return Rotation_Translation;
}

tf::Quaternion Matrix::Quaternion_Read()
{
    return Quaternion;
}

void Matrix::setFromTF(geometry_msgs::TransformStamped transformStamped) {
    try {
        // 四元数
        Quaternion = tf::Quaternion(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        
        // 旋转矩阵
        tf::Matrix3x3 rotation_matrix(Quaternion);
        Rotation = Eigen::Matrix3d::Zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Rotation(i, j) = rotation_matrix[i][j];
            }
        }
        
        // 平移矩阵
        auto& t = transformStamped.transform.translation;
        Translation << t.x, t.y, t.z;
        
        // 旋转平移矩阵
        Rotation_Translation_Set();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return;
    }
}