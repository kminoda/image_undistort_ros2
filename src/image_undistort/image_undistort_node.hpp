
#ifndef IMAGE_UNDISTORT_NODE__IMAGE_UNDISTORT_NODE_HPP__
#define IMAGE_UNDISTORT_NODE__IMAGE_UNDISTORT_NODE_HPP__

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace image_undistort
{
class ImageUndistortNode : public rclcpp::Node
{
private:
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;
public:
  explicit ImageUndistortNode(const rclcpp::NodeOptions & node_options);
private:
  void camera_image_callback(const Image::ConstSharedPtr & msg);
  void camera_info_callback(const CameraInfo::ConstSharedPtr & msg);

  image_transport::Subscriber sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_camera_info_;

  image_transport::Publisher pub_image_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_camera_info_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};
} // namespace image_undistort
#endif  // IMAGE_UNDISTORT_NODE__IMAGE_UNDISTORT_NODE_HPP__
