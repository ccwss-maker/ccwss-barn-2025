#ifndef TF_SUBSCRIBER_NODE_HPP
#define TF_SUBSCRIBER_NODE_HPP
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "Matrix.hpp"

class TFSubscriberNode {
public:
    TFSubscriberNode();
    tf::TransformListener* getTfListener() const {
        return tfListener_.get();
    }
    Matrix Matrix_Read(const std::string &target_frame, const std::string &source_frame);
private:
    std::shared_ptr<tf::TransformListener> tfListener_;
};
#endif // TF_SUBSCRIBER_NODE_HPP