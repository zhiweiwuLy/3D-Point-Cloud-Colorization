#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>  // 可选降采样
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <mutex>

class RGBMapper
{
public:
    RGBMapper(ros::NodeHandle& nh)
    {
        std::string yaml_path;
        nh.param<std::string>("camera_yaml", yaml_path, "config/camera.yaml");
        YAML::Node cam = YAML::LoadFile(yaml_path);

        // ------------------ 相机内参 ------------------
        auto K_data = cam["camera_matrix"]["data"];
        K_ = (cv::Mat_<double>(3,3) <<
            K_data[0].as<double>(), K_data[1].as<double>(), K_data[2].as<double>(),
            K_data[3].as<double>(), K_data[4].as<double>(), K_data[5].as<double>(),
            K_data[6].as<double>(), K_data[7].as<double>(), K_data[8].as<double>());

        auto D_data = cam["distortion_coefficients"]["data"];
        D_ = (cv::Mat_<double>(5,1) <<
            D_data[0].as<double>(), D_data[1].as<double>(), D_data[2].as<double>(),
            D_data[3].as<double>(), D_data[4].as<double>());

        auto P_data = cam["projection_matrix"]["data"];
        P_ = (cv::Mat_<double>(3,4) <<
            P_data[0].as<double>(), P_data[1].as<double>(), P_data[2].as<double>(), P_data[3].as<double>(),
            P_data[4].as<double>(), P_data[5].as<double>(), P_data[6].as<double>(), P_data[7].as<double>(),
            P_data[8].as<double>(), P_data[9].as<double>(), P_data[10].as<double>(), P_data[11].as<double>());

        auto W2C_data = cam["world_to_camera"]["data"];
        world_to_camera_ = (cv::Mat_<double>(4,4) <<
            W2C_data[0].as<double>(), W2C_data[1].as<double>(), W2C_data[2].as<double>(), W2C_data[3].as<double>(),
            W2C_data[4].as<double>(), W2C_data[5].as<double>(), W2C_data[6].as<double>(), W2C_data[7].as<double>(),
            W2C_data[8].as<double>(), W2C_data[9].as<double>(), W2C_data[10].as<double>(), W2C_data[11].as<double>(),
            W2C_data[12].as<double>(), W2C_data[13].as<double>(), W2C_data[14].as<double>(), W2C_data[15].as<double>());

        // 合并投影矩阵：世界坐标系直接到像素坐标系（3x4）
        proj_ = P_ * world_to_camera_;   // 3x4

        // ------------------ 话题 ------------------
        input_pc_topic_ = cam["topics"]["input_pointcloud"].as<std::string>();
        input_img_topic_ = cam["topics"]["input_image"].as<std::string>();
        output_pc_topic_ = cam["topics"]["output_pointcloud"].as<std::string>();

        // ------------------ ROI ------------------
        roi_x_min_ = cam["roi"]["x_min"].as<double>();
        roi_x_max_ = cam["roi"]["x_max"].as<double>();
        roi_y_min_ = cam["roi"]["y_min"].as<double>();
        roi_y_max_ = cam["roi"]["y_max"].as<double>();
        roi_z_min_ = cam["roi"]["z_min"].as<double>();
        roi_z_max_ = cam["roi"]["z_max"].as<double>();

        // 订阅/发布
        pc_sub_ = nh.subscribe(input_pc_topic_, 1, &RGBMapper::pcCallback, this);
        img_sub_ = nh.subscribe(input_img_topic_, 1, &RGBMapper::imgCallback, this);
        pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_pc_topic_, 1);

        ROS_INFO("RGBMapper node initialized (optimized with OpenMP and precomputed maps).");
    }

    void imgCallback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");

            // 预计算畸变映射表（只需计算一次）
            if (!map_initialized_)
            {
                cv::initUndistortRectifyMap(K_, D_, cv::Mat(), K_,
                    cv::Size(cv_ptr->image.cols, cv_ptr->image.rows),
                    CV_32FC1, map1_, map2_);
                map_initialized_ = true;
            }

            // 应用畸变校正
            cv::Mat undistorted;
            cv::remap(cv_ptr->image, undistorted, map1_, map2_, cv::INTER_LINEAR);
            cv_ptr->image = undistorted;
            cv_ptr_ = cv_ptr;  // 存储最新校正后的图像
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
    {
        auto t_total_start = std::chrono::high_resolution_clock::now();

        std::lock_guard<std::mutex> lock(img_mutex_);
        if (!cv_ptr_)
        {
            ROS_WARN_THROTTLE(5, "No image received yet, skip coloring");
            return;
        }

        // 转换点云数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc_msg, *cloud);
        size_t raw_point_count = cloud->points.size();

        // ---- 1. ROI 过滤（仅保留在长方体内部的点）----
        auto t_roi_start = std::chrono::high_resolution_clock::now();

        std::vector<size_t> roi_indices;
        roi_indices.reserve(cloud->points.size());
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const auto& p = cloud->points[i];
            if (p.x >= roi_x_min_ && p.x <= roi_x_max_ &&
                p.y >= roi_y_min_ && p.y <= roi_y_max_ &&
                p.z >= roi_z_min_ && p.z <= roi_z_max_)
            {
                roi_indices.push_back(i);
            }
        }

        auto t_roi_end = std::chrono::high_resolution_clock::now();
        double roi_ms = std::chrono::duration<double, std::milli>(t_roi_end - t_roi_start).count();
        size_t roi_count = roi_indices.size();

        if (roi_indices.empty())
        {
            ROS_WARN_THROTTLE(5, "No points in ROI, skip coloring");
            return;
        }

        // ---- 2. 创建彩色点云并预分配空间 ----
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->header = cloud->header;
        colored_cloud->points.resize(roi_indices.size());

        // ---- 3. 准备投影矩阵数据（连续双精度数组）和图像参数 ----
        double* proj_data = proj_.ptr<double>();   // 3x4 矩阵，行优先
        cv::Mat& img = cv_ptr_->image;
        const int rows = img.rows;
        const int cols = img.cols;
        const uchar* img_data = img.data;
        const int step = img.step;   // 每行字节数

        // ---- 4. 并行为每个点赋予颜色 ----
        auto t_color_start = std::chrono::high_resolution_clock::now();

        #pragma omp parallel for
        for (size_t i = 0; i < roi_indices.size(); ++i)
        {
            size_t idx = roi_indices[i];
            const auto& p = cloud->points[idx];
            pcl::PointXYZRGB& cp = colored_cloud->points[i];
            cp.x = p.x; cp.y = p.y; cp.z = p.z;

            // 手动计算投影： [u, v, w]^T = proj_ * [x,y,z,1]^T
            double x = p.x, y = p.y, z = p.z;
            double u = proj_data[0] * x + proj_data[1] * y + proj_data[2] * z + proj_data[3];
            double v = proj_data[4] * x + proj_data[5] * y + proj_data[6] * z + proj_data[7];
            double w = proj_data[8] * x + proj_data[9] * y + proj_data[10] * z + proj_data[11];

            // 确保点在相机前方
            if (w <= 0.0)
            {
                cp.r = cp.g = cp.b = 255;   // 白色
                continue;
            }

            double inv_w = 1.0 / w;
            int col = static_cast<int>(u * inv_w);
            int row = static_cast<int>(v * inv_w);

            if (row >= 0 && row < rows && col >= 0 && col < cols)
            {
                // 直接访问图像内存（BGR 顺序）
                const uchar* ptr = img_data + row * step + col * 3;
                cp.b = ptr[0];
                cp.g = ptr[1];
                cp.r = ptr[2];
            }
            else
            {
                cp.r = cp.g = cp.b = 255;
            }
        }

        auto t_color_end = std::chrono::high_resolution_clock::now();
        double color_ms = std::chrono::duration<double, std::milli>(t_color_end - t_color_start).count();

        // ---- 5. 设置点云属性并发布 ----
        colored_cloud->width = colored_cloud->points.size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;

        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*colored_cloud, out_msg);
        pc_pub_.publish(out_msg);

        auto t_total_end = std::chrono::high_resolution_clock::now();
        double total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

        // 打印性能统计
        ROS_INFO("RGBMapper: raw=%zu roi=%zu | ROI filter: %.2f ms | Color map: %.2f ms | Total: %.2f ms",
                 raw_point_count, roi_count, roi_ms, color_ms, total_ms);
    }

private:
    ros::Subscriber pc_sub_, img_sub_;
    ros::Publisher pc_pub_;
    std::mutex img_mutex_;
    cv_bridge::CvImagePtr cv_ptr_;

    cv::Mat K_, D_, P_, world_to_camera_;
    cv::Mat proj_;          // 合并后的 3x4 投影矩阵
    cv::Mat map1_, map2_;   // 畸变校正映射表
    bool map_initialized_ = false;

    std::string input_pc_topic_, input_img_topic_, output_pc_topic_;

    // ROI 参数
    double roi_x_min_, roi_x_max_;
    double roi_y_min_, roi_y_max_;
    double roi_z_min_, roi_z_max_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgb_mapper_node");
    ros::NodeHandle nh("~");
    RGBMapper mapper(nh);
    ros::spin();
    return 0;
}