#include "sloam/header.h"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace Eigen;


class LidarOdometry: public rclcpp::Node
{
    /*
    创建LidarOdometry节点 
    */
    public:

        typedef pcl::PointXYZI PointType;
    
        int isDone=1;         //标识运算是否完成
        float planeMax=0.5;   //平面判断门槛值
        std::mutex mLock;      //多线程锁
        pcl::VoxelGrid<PointType> downSizeFilterMap;  //定义点云下采样对象，用于点云抽稀
        

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_plane_frame_cloud;   //接收平面特征点云

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sum_lidar_odom_cloud;   //发布拼接后的点云地图

        //存储当前帧点云
        pcl::PointCloud<PointType>::Ptr lastFramePlanePtr = boost::make_shared<pcl::PointCloud<PointType>>();
        //存储上一帧点云
        pcl::PointCloud<PointType>::Ptr currFramePlanePtr = boost::make_shared<pcl::PointCloud<PointType>>();
        //存储拼接后总点云
        pcl::PointCloud<PointType>::Ptr sumPlaneCloudPtr = boost::make_shared<pcl::PointCloud<PointType>>();
        std::queue<sensor_msgs::msg::PointCloud2> planeQueue;   //定义点云消息队列

        std_msgs::msg::Header currHead;                         //定义ros消息头变量
        double timePlane=0;                                     //定义平面点云帧时间戳变量
        int numFrame=0;                                         //定义帧计数变量
        int flagStart=0;                                        //定义是否开始标志

        std::string topic_name;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

        geometry_msgs::msg::TransformStamped transformStamped;
        

        LidarOdometry(std::string name): Node(name, 
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            /*创建并初始化接收*/
            this->get_parameter("topic_name",topic_name);
            sub_plane_frame_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, std::bind(&LidarOdometry::cloudHandler, this, std::placeholders::_1));
            
            pub_sum_lidar_odom_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sum_lidar_odom_cloud2", 10);

            Eigen::Quaterniond temp(1,0,0,0); 
            downSizeFilterMap.setLeafSize(0.4,0.4,0.4);

            RCLCPP_INFO(this->get_logger(), "\033[1;32m----> LidarOdometry Started.\033[0m");
        }

        void publishResult();
        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cldMsg);
        void cloudThread();
};

//发布点云、里程计、轨迹、地图
void LidarOdometry::publishResult(){

        //发布拼接点云地图
        if(numFrame % 5 == 0)
        {
            double r,p,y;

            // transformStamped = tf_buffer_->lookupTransform("base_link", "lidar",tf2::TimePointZero);

            try {
            transformStamped = tf_buffer_->lookupTransform(
                "odom", "lidar",
                tf2::TimePointZero);
            } catch (tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                "base_link", "lidar", ex.what());
            return;
            }
            
            // // DEBUG
            // RCLCPP_INFO(this->get_logger(), "\033[1;32m----> TF: Rotation: %f %f %f %f\033[0m", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

            // RCLCPP_INFO(this->get_logger(), "\033[1;32m----> TF: Translation: %f %f %f \033[0m", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);

            pcl::PointCloud<PointType>::Ptr cloud_res = boost::make_shared<pcl::PointCloud<PointType>>();
            pcl::transformPointCloud(*currFramePlanePtr, *cloud_res, tf2::transformToEigen(transformStamped.transform).matrix());
            *sumPlaneCloudPtr += *cloud_res;

            pcl::PointCloud<PointType>::Ptr cloud_temp = boost::make_shared<pcl::PointCloud<PointType>>();
            downSizeFilterMap.setInputCloud(sumPlaneCloudPtr);
            downSizeFilterMap.filter(*cloud_temp);

            sensor_msgs::msg::PointCloud2 res_cloud_msgs;
            pcl::toROSMsg(*cloud_temp, res_cloud_msgs);
            res_cloud_msgs.header.stamp = currHead.stamp;
            res_cloud_msgs.header.frame_id = "odom";
            pub_sum_lidar_odom_cloud->publish(res_cloud_msgs);
    }

}

//接收平面特征点云，添加到消息队列中
void LidarOdometry::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cldMsg) {
    mLock.lock();
    planeQueue.push(*cldMsg);
    mLock.unlock();
}

//点云处理多线程函数
void LidarOdometry::cloudThread()
{
    rclcpp::WallRate loop_rate(10);                 //控制处理频率为10hz，为最终频率
    while(rclcpp::ok())
    {
        loop_rate.sleep();
        rclcpp::WallRate loop_rate2(50);            //内层处理频率为50hz
        if(isDone==0) continue;
        while (planeQueue.size()>0)
        {
            if(isDone==0) continue;
            isDone=0;
            numFrame++;
            loop_rate2.sleep();
 
            mLock.lock();                           //锁线程，取数据
            currFramePlanePtr->clear();
            currHead=planeQueue.front().header;
            timePlane=double(planeQueue.front().header.stamp.sec) + 1e-9 * double(planeQueue.front().header.stamp.nanosec);
            pcl::fromROSMsg(planeQueue.front(),*currFramePlanePtr);
            planeQueue.pop();
            mLock.unlock();
 
            if(flagStart==0)
            {
                flagStart=1;
            }
            else
            {   
                this->publishResult();
            }
            *lastFramePlanePtr=*currFramePlanePtr;
            isDone=1;
        }
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarOdometry>("CloudMap");

    std::thread t_cloud_thread(&LidarOdometry::cloudThread, node);
    
    rclcpp::spin(node);
    t_cloud_thread.join();
    rclcpp::shutdown();
    
    return 0;
}
