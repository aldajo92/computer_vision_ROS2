#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using sensor_msgs::msg::Image;

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber_node")
  {
    // Create a subscription to the image topic
    // Quality of Service (QoS) is set to 'best effort' with a history depth of 1
    subscription_ = this->create_subscription<Image>(
      "/camera/image_raw", // Image topic name
      rclcpp::SensorDataQoS(), // Recommended QoS for sensor data
      std::bind(&ImageSubscriber::image_callback, this, _1));

    // Create a publisher for the processed (grayscale) image
    publisher_ = this->create_publisher<Image>(
      "/cv/gray_image", // Output topic name
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Image Subscriber Node Initialized.");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: /camera/image_raw");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: /cv/gray_image");
  }

private:
  void image_callback(const Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received image message.");

    // Convert ROS Image message to OpenCV Mat
    try
    {
      // The bridge handles the conversion. "bgr8" is a common format for color images.
      cv::Mat cv_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

      if (!cv_image.empty())
      {
        // --- OpenCV Processing Example ---
        RCLCPP_INFO(this->get_logger(),
          "Image received: Size = %dx%d, Type = %s",
          cv_image.cols,
          cv_image.rows,
          getCvType(cv_image.type()).c_str());

        // Example processing: Convert to grayscale
        cv::Mat gray_image;
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);

        // Convert processed image back to ROS Image message
        std_msgs::msg::Header header;
        header.stamp = msg->header.stamp; // Keep the same timestamp
        header.frame_id = msg->header.frame_id;

        // Create cv_bridge image and convert to ROS message
        cv_bridge::CvImage cv_bridge_image(header, "mono8", gray_image);
        Image::SharedPtr gray_msg = cv_bridge_image.toImageMsg();

        // Publish the processed image
        publisher_->publish(*gray_msg);
        RCLCPP_INFO(this->get_logger(), "Published grayscale image to /cv/gray_image");
        // --- End of Example ---
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Received an empty image!");
      }
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  // Helper function to get the string representation of an OpenCV Mat type
  std::string getCvType(int type) {
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "UNKNOWN"; break;
    }
    r += "C";
    r += (chans+'0');
    return r;
  }

  rclcpp::Subscription<Image>::SharedPtr subscription_;
  rclcpp::Publisher<Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
