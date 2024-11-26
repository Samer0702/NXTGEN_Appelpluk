#include <ball_detection/ball_detector.hpp>

#include <thread>
#include <chrono>

struct AppleInfo{
  int appleNumber_ = 0;
  int x;
  int y;
  int radius;
};

using std::placeholders::_1;

ball_detector::BallDetector::BallDetector()
  : Node("ball_detector"),
    node_handle_(std::shared_ptr<BallDetector>(this, [](auto *) {})),
    it_(node_handle_)
{
  // Subscribe to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/image_raw", 1, std::bind(&BallDetector::imageCb, this, _1));
  image_pub_ = it_.advertise("/ball_detection/output_image", 1);

  cv::namedWindow(OPENCV_WINDOW);
}

ball_detector::BallDetector::~BallDetector()
{
}

void ball_detector::BallDetector::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const
{
    cv_bridge::CvImagePtr cv_ptr;
    AppleInfo Info;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the HSV range for detecting red color
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv_image, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);   // Lower red range
    cv::inRange(hsv_image, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2); // Upper red range

    // Combine both masks
    cv::bitwise_or(mask1, mask2, mask);

    // Remove noise using morphological operations (erode, then dilate)
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Use Hough Circle Transform to detect circular shapes (apples)
    std::vector<cv::Vec3f> apples;
    cv::HoughCircles(
        mask,              // Input image (binary image)
        apples,            // Output vector of circles
        cv::HOUGH_GRADIENT,// Detection method (we use HOUGH_GRADIENT)
        1,                 // Inverse ratio of the accumulator resolution to image resolution (1 means same resolution)
        100,               // Minimum distance between the centers of the detected circles
        200,               // Higher threshold for the Canny edge detector (edge detection threshold)
        13,                // Accumulator threshold for circle detection (lower it to detect more circles)
        60,                // Minimum radius of detected circles
        500                // Maximum radius of detected circles
    );

    // Store circles for detected apples
    std::vector<cv::Vec3f> detected_apples;

    for (size_t i = 0; i < apples.size(); ++i)
    {
        cv::Vec3i a = apples[i];
        cv::Point center = cv::Point(a[0], a[1]);  // (x, y) center
        int radius = a[2];  // radius of the circle

        // Filter based on circle size (radius range for apples)
        if (radius > 50 && radius < 130)
        {
            // Draw circle around the apple
            cv::circle(cv_ptr->image, center, radius, CV_RGB(0, 255, 0), 2);  // Green circle

            // Store the detected circle (optional)
            detected_apples.push_back(a);
  
            // Count the number of apples detected
            Info.appleNumber_ = detected_apples.size();
            Info.x = center.x;
            Info.y = center.y;
            Info.radius = radius;

            // Log the circle's (x, y) coordinates and radius to the terminal
            RCLCPP_INFO(this->get_logger(), "Apple number %d detected at (x: %d, y: %d) with radius: %d",Info.appleNumber_, Info.x, Info.y, Info.radius);

        }
    }


    // Display the number of detected apples
    char txt[50];
    sprintf(txt, "%d apple(s) detected", Info.appleNumber_);
    cv::putText(cv_ptr->image, txt, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 255, 255), 2, cv::LINE_AA);

    // Output the modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

    // Show the output image with the detected circles
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ball_detector::BallDetector>());
  return 0;
}