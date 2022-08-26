#include "sloam/header.h"

using namespace Eigen;
using namespace std;


class MapOptmization: public rclcpp::Node
{
    /*
    创建MapOptmization节点 
    */
    public:

        double x, y, z;
        double vx, vy, vz;
        double roll, pitch, yaw;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu2odom;

        std_msgs::msg::Header hd;

        MapOptmization(std::string name);
        void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);
        void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg_ptr);
        void imu2odom(const sensor_msgs::msg::Imu::SharedPtr msg_ptr);
};

MapOptmization::MapOptmization(std::string name): Node(name)
{
    /*创建接收和发布*/
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&MapOptmization::odomHandler, this, std::placeholders::_1));

    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom2", 100);

    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 100, std::bind(&MapOptmization::imuHandler, this, std::placeholders::_1));

    pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu2", 100);
    pub_imu2odom = this->create_publisher<nav_msgs::msg::Odometry>("/imu2odom", 100);

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

    pub_imu->publish(new_msg);

    this->imu2odom(msg_ptr);
}

void MapOptmization::imu2odom(const sensor_msgs::msg::Imu::SharedPtr msg_ptr)
{
    // 姿态
    auto delta_t = 0.01;
    auto k = 10;

    auto ax = msg_ptr->linear_acceleration.x;
    auto ay = msg_ptr->linear_acceleration.y;
    auto az = msg_ptr->linear_acceleration.z;
    auto g = sqrt(ax * ax + ay * ay + az * az);
    auto acc_roll = -atan2(ay, az);
    auto acc_pitch = asinh(ax / g);

    cout << "重力加速度：" << g << endl;

    auto omega_x = msg_ptr->angular_velocity.x;
    auto omega_y = msg_ptr->angular_velocity.y;
    auto omega_z = msg_ptr->angular_velocity.z;

    roll = (1 - k * delta_t) * roll + k * delta_t * acc_roll + omega_x * delta_t;
    pitch = (1 - k * delta_t) * pitch + k * delta_t * acc_pitch + omega_y * delta_t;
    yaw = yaw + omega_z * delta_t;

    cout << "姿态：" << roll << " " << pitch << " " << yaw << endl;
    cout << "加速度：" << ax << " " << ay << " " << az << endl;

    // 位置
    Matrix3f R;
    VectorXf a_vect(3), a_global(3);
    R << cos(pitch)*cos(yaw), -sin(yaw)*cos(pitch), sin(pitch),
         sin(roll)*sin(pitch)*cos(yaw) + sin(yaw)*cos(roll), -sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), -sin(roll)*cos(pitch),
         sin(roll)*sin(yaw) - sin(pitch)*cos(roll)*cos(yaw), sin(roll)*cos(yaw) + sin(pitch)*sin(yaw)*cos(roll), cos(roll)*cos(pitch);
    a_vect << ax, ay, az;
    a_global = R.inverse() * a_vect.matrix();

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

    cout << "位置：" << x << " " << y << " " << z << endl;

    pub_imu2odom->publish(IO);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOptmization>("MapOptmization");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}