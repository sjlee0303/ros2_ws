#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class LidarPreprocessor : public rclcpp::Node
{
public:
    LidarPreprocessor()
    : Node("lidar_preprocessing_node")
    {
        // Left LiDAR
        sub_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10, std::bind(&LidarPreprocessor::callbackLeft, this, std::placeholders::_1));
        pub_left_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_pre_left", 10);

        // Right LiDAR
        sub_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10, std::bind(&LidarPreprocessor::callbackRight, this, std::placeholders::_1));
        pub_right_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_pre_right", 10);

        // Down LiDAR
        sub_down_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10, std::bind(&LidarPreprocessor::callbackDown, this, std::placeholders::_1));
        pub_down_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_pre_down", 10);
    }

private:
    // 공통 Voxel Grid 크기
    const float voxel_leaf_size_ = 0.05f;

    // LiDAR별 ROI 설정
    struct ROI {
        float min_x, max_x;
        float min_y, max_y;
        float min_z, max_z;
    };

    ROI roi_left_{-1.0f, 5.0f, -3.0f, 2.0f, -1.0f, 1.5f};
    ROI roi_right_{-0.5f, 6.0f, -2.0f, 3.0f, -0.5f, 1.2f};
    ROI roi_down_{0.0f, 4.0f, -2.5f, 2.5f, -1.0f, 0.8f};

    void callbackLeft(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  { process(msg, roi_left_, pub_left_); }
    void callbackRight(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { process(msg, roi_right_, pub_right_); }
    void callbackDown(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  { process(msg, roi_down_, pub_down_); }

    void process(const sensor_msgs::msg::PointCloud2::SharedPtr input,
                 const ROI& roi,
                 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
    {
        // ROS → PCL 변환
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*input, *cloud_in);

        // Voxel Grid 필터링
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud_in);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*cloud_voxel);

        // ROI 필터링
        pcl::PassThrough<pcl::PointXYZI> pass;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>());
        pass.setInputCloud(cloud_voxel);

        pass.setFilterFieldName("x");
        pass.setFilterLimits(roi.min_x, roi.max_x);
        pass.filter(*cloud_roi);

        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(roi.min_y, roi.max_y);
        pass.filter(*cloud_roi);

        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(roi.min_z, roi.max_z);
        pass.filter(*cloud_roi);

        // PCL → ROS 변환 후 Publish
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_roi, output);
        output.header = input->header;
        pub->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_left_, sub_right_, sub_down_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_left_, pub_right_, pub_down_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
