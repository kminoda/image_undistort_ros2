
#include "image_undistort_node.hpp"

namespace image_undistort
{

ImageUndistortNode::ImageUndistortNode(const rclcpp::NodeOptions & node_options)
: Node("image_undistort", node_options)
{
  sub_image_ = image_transport::create_subscription(
    this, "~/input/image", [this](const Image::ConstSharedPtr & msg) {this->camera_image_callback(msg);}, "raw"
  );
  sub_camera_info_ = this->create_subscription<CameraInfo>(
    "~/input/camera_info", rclcpp::QoS{1}, [this](const CameraInfo::ConstSharedPtr & msg) {this->camera_info_callback(msg);}
  );

  pub_image_ = image_transport::create_publisher(this, "~/output/image");
  pub_camera_info_ = this->create_publisher<CameraInfo>("~/output/camera_info", 1);
}

void ImageUndistortNode::camera_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (camera_matrix_.empty() || dist_coeffs_.empty())
  {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat undistorted_image;
  cv::undistort(cv_ptr->image, undistorted_image, camera_matrix_, dist_coeffs_);

  Image::SharedPtr undistorted_msg = cv_bridge::CvImage(msg->header, msg->encoding, undistorted_image).toImageMsg();
  pub_image_.publish(undistorted_msg);
}

void ImageUndistortNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  camera_matrix_ = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
  dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void *)msg->d.data()).clone();

  pub_camera_info_->publish(*msg);
}

}