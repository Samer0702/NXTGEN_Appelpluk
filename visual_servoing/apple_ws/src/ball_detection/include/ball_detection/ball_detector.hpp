#ifndef BALL_DETECTOR_HPP_
#define BALL_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

namespace ball_detector
{

class BallDetector : public rclcpp::Node
{

public:
  BallDetector();

 ~BallDetector();

private:
    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const;
    
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

};
}  // namespace ball_detector

#endif  // BALL_DETECTOR_HPP_