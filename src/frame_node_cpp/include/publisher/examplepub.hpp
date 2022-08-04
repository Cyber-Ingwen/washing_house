#include <rclcpp/rclcpp.hpp>


class ExamPublisher : public rclcpp::Node
{
public:
  ExamPublisher();
  void sendPointCloud(const LidarPointCloudMsg& msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

inline void PointCloudRosAdapter::init(const YAML::Node& config)
{
  bool send_point_cloud_ros;
  std::string ros_send_topic;
  node_ptr_.reset(new rclcpp::Node("rslidar_points_adapter"));
  yamlRead<bool>(config, "send_point_cloud_ros", send_point_cloud_ros, false);
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");
  if (send_point_cloud_ros)
  {
    point_cloud_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, 1);
  }
}

inline void PointCloudRosAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  point_cloud_pub_->publish(toRosMsg(msg));
}