#include "sloam/header.h"

using namespace Eigen;
using namespace std;


class MapOptmization: public rclcpp::Node
{
    /*
    创建MapOptmization节点 
    */
    public:
        int count;
        double mean;
        double var;

        double x, y, z;
        double vx, vy, vz;
        double roll, pitch, yaw;

        double filtered_ax, filtered_ay, filtered_az;
        int filter_init_flag;
        vector<int> occ_list;
        YAML::Node config;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu2odom;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_test;

        std_msgs::msg::Header hd;

        MapOptmization(std::string name);
        void mapHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);

        void cul_val_mean_var(double input);

        ~MapOptmization(void){}
};

MapOptmization::MapOptmization(std::string name): Node(name)
{
    /*创建接收和发布*/
    sub_map = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sum_lidar_odom_cloud2", 100, std::bind(&MapOptmization::mapHandler, this, std::placeholders::_1));
    pub_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 100);
    pub_map_test = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_test", 100);

    std::string config_path;
    config_path = "src/sloam/config/config.yaml";
    config = YAML::LoadFile(config_path);

    float r = config["resolution"].as<float>();
    int width = int(float(config["width"].as<int>()) / r);
    int height = int(float(config["height"].as<int>()) / r);
    for (int i = 0; i < height * width; i++)
    {
        occ_list.push_back(-1);
    }

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> MapOptmization Started.\033[0m");
    
}

void MapOptmization::mapHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
{
    nav_msgs::msg::OccupancyGrid grid_map;

    // 地图数据 
    float r = config["resolution"].as<float>();
    int width = int(float(config["width"].as<int>()) / r);
    int height = int(float(config["height"].as<int>()) / r);

    grid_map.header.stamp = hd.stamp;
    grid_map.info.map_load_time = msg_ptr->header.stamp;
    grid_map.info.resolution = r;
    grid_map.info.width = width;
    grid_map.info.height = height;
    grid_map.info.origin.position.x = -config["width"].as<int>()/2;
    grid_map.info.origin.position.y = -config["height"].as<int>()/2;
    grid_map.info.origin.position.z = 0;
    grid_map.info.origin.orientation.x = 0;
    grid_map.info.origin.orientation.y = 0;
    grid_map.info.origin.orientation.z = 0;
    grid_map.info.origin.orientation.w = 1;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg_ptr, *cloud);

    float max, min;
    int threshold;
    max = config["max"].as<float>();
    min = config["min"].as<float>();
    threshold = config["threshold"].as<int>();

    for (int i = 0; i < cloud->points.size(); i++)
    {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
        if ((x / r < width/2) && (y / r < width/2) && (x / r > -width/2) && (y / r > -width/2))
        {
            int ind = ((int(y / r) + width/2)) * height + int(x / r) + width/2;
            if (ind > 0)
            {
                if (occ_list[ind] == -1)
                {
                    occ_list[ind] = 10;
                }
                if(z > min && z < max)
                {
                    if (occ_list[ind] < threshold)
                    {
                        occ_list[ind] += 10;
                    }
                    if (occ_list[ind] >= threshold)
                    {
                        occ_list[ind] = 100;
                    }
                }
            }
        }
    }

    for (int i = 0; i < height * width; i++)
    {
        grid_map.data.push_back(int8_t(occ_list[i]));
    }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    // for (int i = 0; i < cloud->points.size(); i++)
    // {
    //     float x = cloud->points[i].x;
    //     float y = cloud->points[i].y;
    //     float z = cloud->points[i].z;
    //     if ((x < width/2) && (y < width/2) && (x > -width/2) && (y > -width/2))
    //     {
    //         if(z > min && z < max)
    //         {
    //             cloud_temp->push_back(cloud->points[i]);
    //         }
    //     }
    // }
    
    // sensor_msgs::msg::PointCloud2 res_cloud_msgs;
    // pcl::toROSMsg(*cloud_temp, res_cloud_msgs);
    // res_cloud_msgs.header.frame_id = "map";
    // pub_map_test->publish(res_cloud_msgs);

    grid_map.header.frame_id = "map";
    pub_map->publish(grid_map);
}

void MapOptmization::cul_val_mean_var(double input)
{
    count += 1;
    var += mean * mean;
    mean = mean * (count - 1) / double(count) + input / double(count);
    var = var * (count - 1) / double(count) + input * input / double(count);
    var -= mean * mean;
    cout << "平均：" << mean << "方差：" << var << endl;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOptmization>("MapOptmization");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
