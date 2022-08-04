#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"


class point_cloud_node: public rclcpp::Node
{
    /* 创建point_cloud节点 */
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;
        std::string topic_name = "/kitti/velo/pointcloud";

    public:
        point_cloud_node(std::string name): Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "point_cloud_node已创建");
            sub_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, std::bind(&point_cloud_node::callback, this, std::placeholders::_1));
        }

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
        {
           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<point_cloud_node>("point_cloud_node");

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
