
#include "image_undistort_node.hpp"

namespace image_undistort
{

ImageUndistortNode::ImageUndistortNode(const rclcpp::NodeOptions & node_options)
: Node("image_undistort", node_options)
{
  sub_compressed_image_ = image_transport::create_subscription(
    this, "~/input/image", [this](const Image::ConstSharedPtr & msg) {this->image_callback(msg);}, "compressed", rmw_qos_profile_sensor_data
  );
  sub_camera_info_ = this->create_subscription<CameraInfo>(
    "~/input/camera_info", rclcpp::SensorDataQoS(), [this](const CameraInfo::ConstSharedPtr & msg) {this->camera_info_callback(msg);}
  );

  pub_compressed_image_ = image_transport::create_publisher(this, "~/output/image");
  pub_camera_info_ = this->create_publisher<CameraInfo>("~/output/camera_info", 10);
}

void ImageUndistortNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::cout << "camera_image_callback" << std::endl;
  if (camera_matrix_.empty() || dist_coeffs_.empty())
  {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat undistorted_image;
  cv::undistort(cv_ptr->image, undistorted_image, camera_matrix_, dist_coeffs_);

  Image::SharedPtr undistorted_msg = cv_bridge::CvImage(msg->header, msg->encoding, undistorted_image).toImageMsg();
  pub_compressed_image_.publish(undistorted_msg);
}

void ImageUndistortNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::cout << "camera_info_callback" << std::endl;

  camera_matrix_ = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
  dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void *)msg->d.data()).clone();

  pub_camera_info_->publish(*msg);
}

} // namespace image_undistort

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::ImageUndistortNode)
