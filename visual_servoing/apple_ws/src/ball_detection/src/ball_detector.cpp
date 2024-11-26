#include <ball_detection/ball_detector.hpp>

using std::placeholders::_1;

ball_detector::BallDetector::BallDetector()
  : Node("ball_detector"),
    node_handle_(std::shared_ptr<BallDetector>(this, [](auto *) {})),
    it_(node_handle_)
{
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/image_raw", 1, std::bind(&BallDetector::imageCb, this, _1));
  image_pub_ = it_.advertise("/ball_detection/output_image", 1);

}

ball_detector::BallDetector::~BallDetector()
{
}

void ball_detector::BallDetector::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

  cv::medianBlur(gray, gray, 3);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 0.5,
                  gray.rows/16,
                  100, 50, 50, 500);


  char txt[50];
  sprintf(txt, "%d circle(s) detected", (int)circles.size());
  cv::putText(cv_ptr->image, txt, cv::Point(50,50), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,255,255), 2, cv::LINE_AA);

  for (auto c : circles)
  {  
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > c[0] + c[2]/2 && cv_ptr->image.cols > c[1] + c[2]/2 &&
          c[0] - c[2]/2 > 0 && c[1] - c[2]/2 > 0)
      cv::circle(cv_ptr->image, cv::Point(cvRound(c[0]), cvRound(c[1])), cvRound(c[2]), CV_RGB(255,0,0), 3);
  }

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ball_detector::BallDetector>());
  return 0;
}