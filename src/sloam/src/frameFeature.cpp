#include "sloam/header.h"


class FrameFeature: public rclcpp::Node
{
    /*
    创建FrameFeature节点 
    */
    public:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane_frame_cloud;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_org_frame_cloud;

        std::string topic_name;

        typedef VelodynePointXYZIRT PointType;   //点类型名称重定义，用于接收点云中各点
        typedef pcl::PointXYZI PointTypeOut;     //pcl点类型名称重定义，简化
        //定义pcl点云对象，存储原始点云
        pcl::PointCloud<PointType>::Ptr framePtr = boost::make_shared<pcl::PointCloud<PointType>>();
        //定义pcl点云对象，存储平面特征点云
        pcl::PointCloud<PointTypeOut>::Ptr framePlanePtr = boost::make_shared<pcl::PointCloud<PointTypeOut>>();
        //定义容器，存储每线点云
        std::vector<pcl::PointCloud<PointType>> v_scan_row = std::vector<pcl::PointCloud<PointType>>(N_SCAN_ROW);
        //定义容器，存储每线点云中各点信息
        std::vector<std::vector<PointInfo>> v_scan_row_info = std::vector<std::vector<PointInfo>>(N_SCAN_ROW);
        float planeMin=0.05;      //定义平面曲率最小门槛值
        int planeSpan=3;         //定义点间隔，用于抽稀
        int rowIndexStart=0;     //定义点云线内点起点索引
        int rowIndexEnd=0;       //定义点云线内点终点索引
        pcl::VoxelGrid<PointTypeOut> downSizeFilterPlane;  //定义点云下采样对象，用于点云抽稀

        FrameFeature(std::string name): Node(name, 
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {
            /*创建并初始化接收*/
            this->get_parameter("topic_name",topic_name);
            // std::string topic_name = "/rslidar_points";
            subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 100, std::bind(&FrameFeature::cloudHandler, this, std::placeholders::_1));
        
            pub_plane_frame_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_points", 100);
            pub_org_frame_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/org_frame_cloud1", 100);

            downSizeFilterPlane.setLeafSize(0.2,0.2,0.2);

            RCLCPP_INFO(this->get_logger(), "\033[1;32m----> FrameFeature Started.\033[0m");
        }

        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cldMsg);
};


//接收原始点云，处理，发布
void FrameFeature::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cldMsg) {
    framePtr->clear();                    //存储点云之前需要先清空
    framePlanePtr->clear();
    v_scan_row = std::vector<pcl::PointCloud<PointType>>(N_SCAN_ROW);
    v_scan_row_info = std::vector<std::vector<PointInfo>>(N_SCAN_ROW);
    //将ros点云消息类型转换为pcl点云对象
    pcl::fromROSMsg(*cldMsg, *framePtr);
    //遍历点云各点，重建点云索引
    for (size_t i = 0; i < framePtr->points.size(); ++i) {
        PointType point;
        point.x = framePtr->points[i].x;
        point.y = framePtr->points[i].y;
        point.z = framePtr->points[i].z;
        point.intensity = framePtr->points[i].intensity;
        int scanID = -1;
        int flag = 2;         //1-使用原始点云线号ring信息  2-根据垂向角度计算点云线号
        if (flag == 1) {
            // scanID = framePtr->points[i].ring;
        } else {
            //计算垂向角度
            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            if (N_SCAN_ROW == 16) {     //16线点云线号计算
                if (angle >= -15 || angle <= 15) {
                    scanID = int((angle + 15) / 2 + 0.5);
                }
            }
            if (N_SCAN_ROW == 64) {     //64线点云线号计算
                if (angle >= -24.33 || angle <= 2) {
                    if (angle >= -8.83){
                        scanID = int((2 - angle) * 3.0 + 0.5);
                    }else{
                        scanID = N_SCAN_ROW / 2 + int((-8.83 - angle) * 2.0 + 0.5);
                    }
                }
            }
        }
        if (scanID > -1 && scanID < N_SCAN_ROW) {        //每条扫描线是一个点云对象
            PointInfo p_info;
            p_info.value = 0;
            p_info.indexInRow = v_scan_row[scanID].size();
            point.intensity=p_info.indexInRow+scanID/100.0;
            v_scan_row[scanID].push_back(point);         //用数组存储各线点云数据
            v_scan_row_info[scanID].push_back(p_info);   //用另一个数组同步存储各点信息
        }
    }
 
    //计算点云曲率
    for (int i = 0+rowIndexStart; i < N_SCAN_ROW-rowIndexEnd; i++) {
        for (int j = 5; j < int(v_scan_row[i].size()) - 5; j++) {
            float diffX =
                    v_scan_row[i].points[j - 5].x + v_scan_row[i].points[j - 4].x + v_scan_row[i].points[j - 3].x +
                    v_scan_row[i].points[j - 2].x + v_scan_row[i].points[j - 1].x
                    - 10 * v_scan_row[i].points[j].x
                    + v_scan_row[i].points[j + 1].x + v_scan_row[i].points[j + 2].x + v_scan_row[i].points[j + 3].x +
                    v_scan_row[i].points[j + 4].x + v_scan_row[i].points[j + 5].x;
            float diffY =
                    v_scan_row[i].points[j - 5].y + v_scan_row[i].points[j - 4].y + v_scan_row[i].points[j - 3].y +
                    v_scan_row[i].points[j - 2].y + v_scan_row[i].points[j - 1].y
                    - 10 * v_scan_row[i].points[j].y
                    + v_scan_row[i].points[j + 1].y + v_scan_row[i].points[j + 2].y + v_scan_row[i].points[j + 3].y +
                    v_scan_row[i].points[j + 4].y + v_scan_row[i].points[j + 5].y;
            float diffZ =
                    v_scan_row[i].points[j - 5].z + v_scan_row[i].points[j - 4].z + v_scan_row[i].points[j - 3].z +
                    v_scan_row[i].points[j - 2].z + v_scan_row[i].points[j - 1].z
                    - 10 * v_scan_row[i].points[j].z
                    + v_scan_row[i].points[j + 1].z + v_scan_row[i].points[j + 2].z + v_scan_row[i].points[j + 3].z +
                    v_scan_row[i].points[j + 4].z + v_scan_row[i].points[j + 5].z;
            //存储各线各点曲率值
            v_scan_row_info[i][j].value = (diffX * diffX + diffY * diffY + diffZ * diffZ);
        }
    }
 
    //遍历各线，再遍历各点，根据曲率门槛值筛选出平面特征点云
    for (int i = 0+rowIndexStart; i < N_SCAN_ROW-rowIndexEnd; i++) {
        size_t jstart = 0;
        for (size_t j = 0; j < v_scan_row_info[i].size(); j++) {
            if (j >= jstart && v_scan_row_info[i][j].value < planeMin) {
                PointTypeOut pt;
                pt.x = v_scan_row[i][v_scan_row_info[i][j].indexInRow].x;
                pt.y = v_scan_row[i][v_scan_row_info[i][j].indexInRow].y;
                pt.z = v_scan_row[i][v_scan_row_info[i][j].indexInRow].z;
                pt.intensity = v_scan_row[i][v_scan_row_info[i][j].indexInRow].intensity;
                framePlanePtr->push_back(pt);
                jstart = j + planeSpan;      //按指定间隔提取点云，相当于抽稀
            }
        }
    }
    //点云下采样，抽稀
    pcl::PointCloud<PointTypeOut>::Ptr cloud_temp(new pcl::PointCloud<PointTypeOut>());
    downSizeFilterPlane.setInputCloud(framePlanePtr);
    downSizeFilterPlane.filter(*cloud_temp);
    //发布平面特征点云
    sensor_msgs::msg::PointCloud2 planeCloudMsg;
    pcl::toROSMsg(*cloud_temp, planeCloudMsg);      //将pcl点云对象转换为ros点云消息类型
    planeCloudMsg.header.stamp = cldMsg->header.stamp;
    planeCloudMsg.header.frame_id = "odom";
    pub_plane_frame_cloud->publish(planeCloudMsg);
    //发布原始点云
    sensor_msgs::msg::PointCloud2 orgCloudMsg;
    orgCloudMsg=*cldMsg;
    orgCloudMsg.header.frame_id="odom";
    pub_org_frame_cloud->publish(orgCloudMsg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameFeature>("FrameFeature");

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
