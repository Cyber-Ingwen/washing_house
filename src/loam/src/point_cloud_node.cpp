#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"


class point_cloud_node: public rclcpp::Node
{
    /*
    创建point_cloud节点 
    */
    public:
        std::string viz_name;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;

        point_cloud_node(std::string name): Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "point_cloud_node已创建");

            /*创建并初始化接收*/
            std::string topic_name = "/rslidar_points";
            sub_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, std::bind(&point_cloud_node::callback, this, std::placeholders::_1));
        
            /*配置可视化*/
            viz_name = "pcl cloud";
            visualizer = boost::make_shared<pcl::visualization::PCLVisualizer>(viz_name);
        }

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
        {
            /*处理点云*/
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::fromROSMsg(*msg_ptr, *cloud);

            /*可视化点云*/
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(cloud, "z");
            visualizer->removeAllPointClouds();
            visualizer->addPointCloud(cloud, intensity, viz_name, 0);
            visualizer->spinOnce(0.001);
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
