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
        void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);
        void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg_ptr);
        void mapHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);

        void imu2odom(const sensor_msgs::msg::Imu::SharedPtr msg_ptr);
        void kalman(const sensor_msgs::msg::Imu::SharedPtr msg_ptr);
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

void MapOptmization::odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg_ptr)
{
    nav_msgs::msg::Odometry new_msg;
    new_msg = *msg_ptr;

    new_msg.header.frame_id = "map";
    hd = new_msg.header;

    auto temp = new_msg.pose.pose.position.y;
    new_msg.pose.pose.position.x = -new_msg.pose.pose.position.x;
    new_msg.pose.pose.position.y = -temp;

    auto temp2 = new_msg.pose.pose.orientation.y;
    auto temp3 = new_msg.pose.pose.orientation.z;

    new_msg.pose.pose.orientation.y = new_msg.pose.pose.orientation.x;
    new_msg.pose.pose.orientation.x = -temp2;
    new_msg.pose.pose.orientation.z = new_msg.pose.pose.orientation.w;
    new_msg.pose.pose.orientation.w = -temp3;

    pub_odom->publish(new_msg);
}

void MapOptmization::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg_ptr)
{
    sensor_msgs::msg::Imu new_msg;
    new_msg = *msg_ptr;

    new_msg.header.stamp = hd.stamp;
    new_msg.header.frame_id = "map";

    new_msg.linear_acceleration.x = msg_ptr->linear_acceleration.x + 0.421;
    new_msg.linear_acceleration.y = msg_ptr->linear_acceleration.y + 0.265;
    new_msg.linear_acceleration.z = msg_ptr->linear_acceleration.z + 0.016;

    new_msg.angular_velocity.x = (msg_ptr->angular_velocity.x - 0.0182214) * 3.1416 / 180.0;
    new_msg.angular_velocity.y = (msg_ptr->angular_velocity.y - 0.0831042) * 3.1416 / 180.0;
    new_msg.angular_velocity.z = (msg_ptr->angular_velocity.z - 0.0688892) * 3.1416 / 180.0;

    new_msg.orientation_covariance = {1000000.0, 0, 0,
                                    0, 1000000, 0,
                                    0, 0, 0.000001};
    new_msg.angular_velocity_covariance = new_msg.orientation_covariance;
    new_msg.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

    pub_imu->publish(new_msg);

    this->imu2odom(msg_ptr);
}

void MapOptmization::imu2odom(const sensor_msgs::msg::Imu::SharedPtr msg_ptr)
{
    auto delta_t = 0.01;
    auto k = 1;

    auto ax = msg_ptr->linear_acceleration.x + 0.421;
    auto ay = msg_ptr->linear_acceleration.y + 0.265;
    auto az = msg_ptr->linear_acceleration.z;
    auto g = 9.816;
    cout << "加速度：" << ax << " " << ay << " " << az << endl;

    // 姿态
    auto acc_roll = -atan2(ay, az);
    auto acc_pitch = asinh(ax / g);

    auto omega_x = (msg_ptr->angular_velocity.x - 0.0182214) * 3.1416 / 180.0;
    auto omega_y = (msg_ptr->angular_velocity.y - 0.0831042) * 3.1416 / 180.0;
    auto omega_z = (msg_ptr->angular_velocity.z - 0.0688892) * 3.1416 / 180.0;

    roll = (1 - k * delta_t) * roll + k * delta_t * acc_roll + omega_x * delta_t;
    pitch = (1 - k * delta_t) * pitch + k * delta_t * acc_pitch + omega_y * delta_t;
    yaw = yaw + omega_z * delta_t;

    this->cul_val_mean_var(roll);

    cout << "旋转：" << roll << " " << pitch << " " << yaw << endl;

    // 位置
    Matrix3f R;
    VectorXf a_vect(3), a_global(3);
    R << cos(pitch)*cos(yaw), -sin(yaw)*cos(pitch), sin(pitch),
         sin(roll)*sin(pitch)*cos(yaw) + sin(yaw)*cos(roll), -sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), -sin(roll)*cos(pitch),
         sin(roll)*sin(yaw) - sin(pitch)*cos(roll)*cos(yaw), sin(roll)*cos(yaw) + sin(pitch)*sin(yaw)*cos(roll), cos(roll)*cos(pitch);
    a_vect << ax, ay, az;
    a_global = R * a_vect.matrix();

    ax = a_global(0);
    ay = a_global(1);
    az = a_global(2) - g;

    cout << "加速度：" << ax << " " << ay << " " << az << endl;

    vx = vx + ax * delta_t;
    vy = vy + ay * delta_t;
    vz = vz + az * delta_t;

    x = x + vx * delta_t;
    y = y + vy * delta_t;
    z = z + vz * delta_t;

    cout << "位置：" << x << " " << y << " " << z << endl;

    auto cy = cos(yaw * 0.5);
    auto sy = sin(yaw * 0.5);
    auto cp = cos(pitch * 0.5);
    auto sp = sin(pitch * 0.5);
    auto cr = cos(roll * 0.5);
    auto sr = sin(roll * 0.5);

    nav_msgs::msg::Odometry IO;
    IO.header.frame_id = "map";
    IO.child_frame_id = "base_link";
    IO.header.stamp = msg_ptr->header.stamp;
    IO.pose.pose.position.x = x;
    IO.pose.pose.position.y = y;
    IO.pose.pose.position.z = z;
    IO.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
    IO.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
    IO.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr;
    IO.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr;

    pub_imu2odom->publish(IO);
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
