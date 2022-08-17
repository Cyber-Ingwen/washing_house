#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "LidarOdometry.hpp"
#include <ctime>

clock_t t0, t1;

class point_cloud_node: public rclcpp::Node
{
    /*
    创建point_cloud节点 
    */
    public:
        std::string viz_name;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;

        LidarOdometry lidar_odometry;
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud;

        point_cloud_node(std::string name): Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "point_cloud_node已创建");

            /*创建并初始化接收*/
            std::string topic_name = "/rslidar_points";
            sub_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, std::bind(&point_cloud_node::callback, this, std::placeholders::_1));
        
            /*配置可视化*/
            viz_name = "pcl cloud";
            visualizer = boost::make_shared<pcl::visualization::PCLVisualizer>(viz_name);
            visualizer->setBackgroundColor(.3, .3, .3);

            
        }

        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
        {
            /*处理点云*/
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pcl::fromROSMsg(*msg_ptr, *cloud);
        
            /*运行算法*/
            t0 = clock();

            lidar_odometry.input(cloud);

            auto last_cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            *last_cloud_ptr = *cloud;
            full_cloud = cloud;
            
            if (lidar_odometry.init_flag == 1)
            {
                for (int i = lidar_odometry.T_list.size() - 1; i >= 0; i--)
                {
                    full_cloud = lidar_odometry.transform(full_cloud, lidar_odometry.T_list[i]);
                }
            }
            /*
            full_cloud = lidar_odometry.transform(cloud, lidar_odometry.T);
            */

            auto ptr3 = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            *ptr3 = *lidar_odometry.plane_points;
            ptr3 = lidar_odometry.transform(ptr3, lidar_odometry.T);

            t1 = clock();
            double endtime=(double)(t1-t0)/CLOCKS_PER_SEC;
            cout<<"Total time:"<<endtime*1000<<"ms"<<endl;

            /*可视化点云*/
            auto ptr = last_cloud_ptr;
            auto ptr2 = full_cloud;
            //auto ptr3 = lidar_odometry.plane_points;

            //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(ptr, "intensity");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color1(ptr, 155, 120, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color2(ptr2, 120, 150, 155);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color3(ptr3, 255, 0, 0);

            visualizer->removeAllPointClouds();
            visualizer->addPointCloud(ptr3, color3, "3", 0);
            visualizer->addPointCloud(ptr2, color2, "2", 0);
            visualizer->addPointCloud(ptr, color1, "1", 0);
            visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.2, "2");
            visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3");
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
