// src/map_accumulator_node_with_disroi.cpp
// 基于 map_accumulator_node.cpp 添加 DisROI (忽略ROI) 功能
// DisROI 功能：定义一个区域，该区域内的点云将被直接舍弃，不参与地图积累
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>

// 地图ROI参数结构
struct MapROIParams {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
    bool enable_roi_filter;
};

// *** 新增：DisROI参数结构 ***
struct DisROIParams {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
    bool enable_disroi_filter;
};

class MapAccumulatorWithDisROI {
public:
    MapAccumulatorWithDisROI(ros::NodeHandle& nh) 
        : nh_(nh), private_nh_("~"), tf_listener_(tf_buffer_), accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          save_requested_(false), clear_requested_(false) {
        // 读取参数
        private_nh_.param("input_topic", input_topic_, std::string("/cloud_registered"));
        private_nh_.param("map_frame", map_frame_, std::string("camera_init"));
        private_nh_.param("base_frame", base_frame_, std::string("base_link"));
        private_nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.1);
        private_nh_.param("max_points", max_points_, 50000); // 500万点
        private_nh_.param("save_directory", save_directory_, std::string("/tmp/fastlio_maps"));
        private_nh_.param("save_interval", save_interval_, 30.0); // 30秒保存一次
        private_nh_.param("enable_realtime_preview", enable_realtime_preview_, true);
        private_nh_.param("preview_rate", preview_rate_, 20.0); // 20Hz预览
        private_nh_.param("time_window", time_window_, 5000.0); // 新增：时间窗口参数，默认10秒
        
        // 地图ROI过滤参数
        private_nh_.param("enable_map_roi_filter", map_roi_params_.enable_roi_filter, true);
        private_nh_.param("map_roi_min_x", map_roi_params_.min_x, -1.0);
        private_nh_.param("map_roi_max_x", map_roi_params_.max_x, 5.00);
        private_nh_.param("map_roi_min_y", map_roi_params_.min_y, -5.0);
        private_nh_.param("map_roi_max_y", map_roi_params_.max_y, 5.0);
        private_nh_.param("map_roi_min_z", map_roi_params_.min_z, 0.3);
        private_nh_.param("map_roi_max_z", map_roi_params_.max_z, 1.3);

        // *** 新增：DisROI过滤参数 ***
        private_nh_.param("enable_disroi_filter", disroi_params_.enable_disroi_filter, false);
        private_nh_.param("disroi_min_x", disroi_params_.min_x, -2.0);
        private_nh_.param("disroi_max_x", disroi_params_.max_x, 2.0);
        private_nh_.param("disroi_min_y", disroi_params_.min_y, -2.0);
        private_nh_.param("disroi_max_y", disroi_params_.max_y, 2.0);
        private_nh_.param("disroi_min_z", disroi_params_.min_z, 0.0);
        private_nh_.param("disroi_max_z", disroi_params_.max_z, 1.8);

        // 创建保存目录
        boost::filesystem::create_directories(save_directory_);

        // 订阅话题
        cloud_sub_ = nh_.subscribe(input_topic_, 10, &MapAccumulatorWithDisROI::cloudCallback, this);
        odom_sub_ = nh_.subscribe("/Odometry", 10, &MapAccumulatorWithDisROI::odomCallback, this);

        // 发布累积地图
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_accumulator/accumulated_map", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/map_accumulator/map_info", 1);
        
        // *** 新增：发布ROI可视化 ***
        roi_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/map_accumulator/roi_markers", 1);

        // 服务
        save_service_ = nh_.advertiseService("/map_accumulator/save_map", &MapAccumulatorWithDisROI::saveMapService, this);
        clear_service_ = nh_.advertiseService("/map_accumulator/clear_map", &MapAccumulatorWithDisROI::clearMapService, this);
        get_info_service_ = nh_.advertiseService("/map_accumulator/get_info", &MapAccumulatorWithDisROI::getInfoService, this);

        // 发布话题
        info_pub_ = nh_.advertise<std_msgs::String>("/map_accumulator/info", 1);

        // 初始化时间
        last_save_time_ = ros::Time::now();
        last_preview_time_ = ros::Time::now();

        // 统计计数器
        total_points_received_ = 0;
        disroi_filtered_points_ = 0;
        roi_filtered_points_ = 0;
        accumulated_points_ = 0;

        ROS_INFO("MapAccumulatorWithDisROI initialized");
        ROS_INFO("Input topic: %s", input_topic_.c_str());
        ROS_INFO("Map frame: %s", map_frame_.c_str());
        ROS_INFO("Save directory: %s", save_directory_.c_str());
        ROS_INFO("Time window: %.1f seconds (only accumulating recent %.1f seconds of data)", time_window_, time_window_);
        
        if (map_roi_params_.enable_roi_filter) {
            ROS_INFO("Map ROI Filter enabled: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]", 
                     map_roi_params_.min_x, map_roi_params_.max_x,
                     map_roi_params_.min_y, map_roi_params_.max_y,
                     map_roi_params_.min_z, map_roi_params_.max_z);
        } else {
             ROS_INFO("Map ROI Filter is DISABLED.");
        }

        // *** 新增：DisROI信息输出 ***
        if (disroi_params_.enable_disroi_filter) {
            ROS_INFO("DisROI Filter enabled: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]", 
                     disroi_params_.min_x, disroi_params_.max_x,
                     disroi_params_.min_y, disroi_params_.max_y,
                     disroi_params_.min_z, disroi_params_.max_z);
            ROS_INFO("Points inside DisROI will be DISCARDED and NOT accumulated.");
        } else {
             ROS_INFO("DisROI Filter is DISABLED.");
        }

        // 启动后台线程
        background_thread_ = std::thread(&MapAccumulatorWithDisROI::backgroundThread, this);
    }

    ~MapAccumulatorWithDisROI() {
        if (background_thread_.joinable()) {
            background_thread_.join();
        }
        // 最后保存一次地图
        saveMap("final_map");
        
        // *** 输出统计信息 ***
        ROS_INFO("=== Final Statistics ===");
        ROS_INFO("Total points received: %zu", total_points_received_);
        ROS_INFO("DisROI filtered points: %zu (%.2f%%)", disroi_filtered_points_, 
                 total_points_received_ > 0 ? 100.0 * disroi_filtered_points_ / total_points_received_ : 0.0);
        ROS_INFO("ROI filtered points: %zu (%.2f%%)", roi_filtered_points_,
                 total_points_received_ > 0 ? 100.0 * roi_filtered_points_ / total_points_received_ : 0.0);
        ROS_INFO("Accumulated points: %zu (%.2f%%)", accumulated_points_,
                 total_points_received_ > 0 ? 100.0 * accumulated_points_ / total_points_received_ : 0.0);
    }

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher map_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher roi_marker_pub_;  // *** 新增：ROI可视化发布器 ***
    ros::Publisher info_pub_;
    ros::ServiceServer save_service_;
    ros::ServiceServer clear_service_;
    ros::ServiceServer get_info_service_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 参数
    std::string input_topic_;
    std::string map_frame_;
    std::string base_frame_;
    std::string save_directory_;
    double voxel_leaf_size_;
    int max_points_;
    double save_interval_;
    bool enable_realtime_preview_;
    double preview_rate_;
    double time_window_; // 新增：时间窗口参数（秒）

    // ROI参数
    MapROIParams map_roi_params_;
    DisROIParams disroi_params_;     // *** 新增：DisROI参数 ***

    // 数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, ros::Time>> cloud_buffer_; // 新增：带时间戳的点云缓冲区
    std::mutex cloud_mutex_;
    ros::Time last_save_time_;
    ros::Time last_preview_time_;
    int frame_count_ = 0;
    std::vector<geometry_msgs::PoseStamped> trajectory_;

    // *** 新增：统计计数器 ***
    size_t total_points_received_;
    size_t disroi_filtered_points_;
    size_t roi_filtered_points_;
    size_t accumulated_points_;

    // 控制标志
    bool save_requested_;
    bool clear_requested_;
    std::string save_name_;
    std::mutex control_mutex_;

    // 线程
    std::thread background_thread_;

    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

    // 点云处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // ROI过滤函数
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterMapROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterDisROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);  // *** 新增：DisROI过滤 ***

    // 地图管理
    void addCloudToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void removeOldClouds(); // 新增：移除超时的点云数据
    void rebuildAccumulatedMap(); // 新增：重建累积地图
    void publishMap();
    void publishMapInfo();
    void publishROIMarkers();  // *** 新增：发布ROI可视化 ***

    // 服务回调
    bool saveMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool clearMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool getInfoService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    // 文件操作
    bool saveMap(const std::string& name = "");
    bool loadMap(const std::string& filename);

    // 后台线程
    void backgroundThread();

    // 实时预览
    void realtimePreview();

    // 工具函数
    std::string getCurrentTimeString();
    std::string generateSaveFilename(const std::string& name = "");
};

void MapAccumulatorWithDisROI::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 转换点云到地图坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = transformCloud(cloud_msg);
    if (!transformed_cloud || transformed_cloud->empty()) {
        return;
    }

    // *** 统计原始点数 ***
    total_points_received_ += transformed_cloud->size();

    // 添加到累积地图 (内部会处理ROI过滤)
    addCloudToMap(transformed_cloud);

    // 实时输出当前累积地图点云总数
    ROS_INFO_THROTTLE(1.0, "[MapAccumulator] Current frame points: %zu, accumulated map points: %zu", 
        transformed_cloud ? transformed_cloud->size() : 0, accumulated_cloud_->size());

    // 实时预览
    if (enable_realtime_preview_ && 
        (ros::Time::now() - last_preview_time_).toSec() > 1.0/preview_rate_) {
        realtimePreview();
        last_preview_time_ = ros::Time::now();
    }
    frame_count_++;
}

void MapAccumulatorWithDisROI::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    // 保存轨迹点
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg->header;
    pose_stamped.pose = odom_msg->pose.pose;
    trajectory_.push_back(pose_stamped);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapAccumulatorWithDisROI::transformCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    try {
        // 转换到地图坐标系
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            map_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.1));
        sensor_msgs::PointCloud2 transformed_msg;
        tf2::doTransform(*cloud_msg, transformed_msg, transform);
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_msg, *cloud);
        return cloud;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return nullptr;
    }
}

void MapAccumulatorWithDisROI::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || voxel_leaf_size_ <= 0) {
        return;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.filter(*cloud);
}

// 原有的 Map ROI 过滤函数（保留ROI内的点）
pcl::PointCloud<pcl::PointXYZ>::Ptr MapAccumulatorWithDisROI::filterMapROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || !map_roi_params_.enable_roi_filter) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr region_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;

    // X 方向过滤
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(map_roi_params_.min_x, map_roi_params_.max_x);
    pass.filter(*temp_cloud);

    if (temp_cloud->empty()) {
        roi_filtered_points_ += cloud->size();
        return temp_cloud;
    }

    // Y 方向过滤
    pass.setInputCloud(temp_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(map_roi_params_.min_y, map_roi_params_.max_y);
    pass.filter(*region_cloud);

    if (region_cloud->empty()) {
        roi_filtered_points_ += cloud->size();
        return region_cloud;
    }

    // Z 方向过滤
    pass.setInputCloud(region_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(map_roi_params_.min_z, map_roi_params_.max_z);
    pass.filter(*region_cloud);

    // *** 统计过滤的点数 ***
    size_t filtered_count = cloud->size() - region_cloud->size();
    roi_filtered_points_ += filtered_count;

    ROS_DEBUG("Map ROI filtering: %zu -> %zu points", cloud->size(), region_cloud->size());
    return region_cloud;
}

// *** 新增：DisROI 过滤函数（移除DisROI内的点）***
// 注意：DisROI区域是在世界坐标系(map_frame)下定义的
// 该函数移除在世界坐标系下指定DisROI区域内的所有点云
pcl::PointCloud<pcl::PointXYZ>::Ptr MapAccumulatorWithDisROI::filterDisROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || !disroi_params_.enable_disroi_filter) {
        return cloud; // 如果未启用DisROI过滤，直接返回原云
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    size_t discard_count = 0;

    // 手动遍历每个点，检查是否在DisROI内
    for (const auto& point : cloud->points) {
        bool inside_disroi = (point.x >= disroi_params_.min_x && point.x <= disroi_params_.max_x) &&
                            (point.y >= disroi_params_.min_y && point.y <= disroi_params_.max_y) &&
                            (point.z >= disroi_params_.min_z && point.z <= disroi_params_.max_z);
        
        if (!inside_disroi) {
            // 如果点不在DisROI内，则保留
            filtered_cloud->points.push_back(point);
        } else {
            // 如果点在DisROI内，则丢弃（计数）
            discard_count++;
        }
    }

    // 更新点云信息
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = cloud->is_dense;
    filtered_cloud->header = cloud->header;

    // *** 统计丢弃的点数 ***
    disroi_filtered_points_ += discard_count;

    ROS_DEBUG("DisROI filtering: %zu -> %zu points (%zu discarded)", 
             cloud->size(), filtered_cloud->size(), discard_count);

    return filtered_cloud;
}

void MapAccumulatorWithDisROI::addCloudToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    // *** 处理流程：1. DisROI过滤（移除） 2. ROI过滤（保留） 3. 降采样 4. 添加到缓冲区 ***
    
    // 步骤1：应用DisROI过滤（移除DisROI内的点）
    pcl::PointCloud<pcl::PointXYZ>::Ptr disroi_filtered_cloud = filterDisROI(cloud);
    if (disroi_filtered_cloud->empty()) {
        ROS_DEBUG("All points filtered out by DisROI, skipping addition to map.");
        return;
    }

    // 步骤2：应用Map ROI过滤（保留ROI内的点）
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_filtered_cloud = filterMapROI(disroi_filtered_cloud);
    if (roi_filtered_cloud->empty()) {
        ROS_DEBUG("All points filtered out by Map ROI, skipping addition to map.");
        return;
    }

    // 步骤3：降采样过滤后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>(*roi_filtered_cloud));
    downsampleCloud(downsampled_cloud);

    // 步骤4：添加到缓冲区（带时间戳）
    ros::Time current_time = ros::Time::now();
    cloud_buffer_.push_back(std::make_pair(downsampled_cloud, current_time));

    // *** 统计实际积累的点数 ***
    accumulated_points_ += downsampled_cloud->size();

    // 移除超时的点云数据
    removeOldClouds();

    // 重建累积地图
    rebuildAccumulatedMap();

    ROS_DEBUG("Added %zu points (after DisROI/ROI/downsample), total: %zu in buffer with %zu frames", 
             downsampled_cloud->size(), accumulated_cloud_->size(), cloud_buffer_.size());
}

// 新增：移除超时的点云数据
void MapAccumulatorWithDisROI::removeOldClouds() {
    ros::Time current_time = ros::Time::now();
    auto it = cloud_buffer_.begin();
    
    while (it != cloud_buffer_.end()) {
        double time_diff = (current_time - it->second).toSec();
        if (time_diff > time_window_) {
            // 从统计中减去被移除的点数
            accumulated_points_ -= it->first->size();
            it = cloud_buffer_.erase(it);
        } else {
            ++it;
        }
    }
}

// 新增：重建累积地图
void MapAccumulatorWithDisROI::rebuildAccumulatedMap() {
    accumulated_cloud_->clear();
    
    // 将缓冲区中的所有点云合并到累积地图中
    for (const auto& cloud_pair : cloud_buffer_) {
        *accumulated_cloud_ += *(cloud_pair.first);
    }
    
    // 如果点数过多，进行整体降采样
    if (accumulated_cloud_->size() > static_cast<size_t>(max_points_)) {
        ROS_WARN("Map size exceeds limit (%d points), downsampling the entire map...", max_points_);
        downsampleCloud(accumulated_cloud_);
    }
}

void MapAccumulatorWithDisROI::publishMap() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
        return;
    }
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*accumulated_cloud_, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_;
    map_pub_.publish(map_msg);
}

void MapAccumulatorWithDisROI::publishMapInfo() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    // 发布文本信息
    std_msgs::String info_msg;
    std::stringstream ss;
    ss << "Map Info (Time Window: " << std::fixed << std::setprecision(1) << time_window_ << "s):\n";
    ss << "Points: " << accumulated_cloud_->size() << "\n";
    ss << "Frames in buffer: " << cloud_buffer_.size() << "\n";
    ss << "Total frames processed: " << frame_count_ << "\n";
    ss << "Trajectory points: " << trajectory_.size() << "\n";
    
    // *** 新增：统计信息 ***
    if (total_points_received_ > 0) {
        ss << "=== Filtering Statistics ===\n";
        ss << "Total received: " << total_points_received_ << "\n";
        ss << "DisROI filtered: " << disroi_filtered_points_ 
           << " (" << std::fixed << std::setprecision(1) 
           << 100.0 * disroi_filtered_points_ / total_points_received_ << "%)\n";
        ss << "ROI filtered: " << roi_filtered_points_
           << " (" << std::fixed << std::setprecision(1) 
           << 100.0 * roi_filtered_points_ / total_points_received_ << "%)\n";
        ss << "Accumulated: " << accumulated_points_
           << " (" << std::fixed << std::setprecision(1) 
           << 100.0 * accumulated_points_ / total_points_received_ << "%)\n";
    }
    
    if (map_roi_params_.enable_roi_filter) {
        ss << "Map ROI: X[" << map_roi_params_.min_x << "," << map_roi_params_.max_x << "] "
           << "Y[" << map_roi_params_.min_y << "," << map_roi_params_.max_y << "] "
           << "Z[" << map_roi_params_.min_z << "," << map_roi_params_.max_z << "]\n";
    }
    if (disroi_params_.enable_disroi_filter) {
        ss << "DisROI: X[" << disroi_params_.min_x << "," << disroi_params_.max_x << "] "
           << "Y[" << disroi_params_.min_y << "," << disroi_params_.max_y << "] "
           << "Z[" << disroi_params_.min_z << "," << disroi_params_.max_z << "]\n";
    }
    
    info_msg.data = ss.str();
    info_pub_.publish(info_msg);

    // 发布可视化标记
    if (!accumulated_cloud_->empty()) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = map_frame_;
        marker.ns = "map_info";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 5; // 在上方显示
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.text = "Points: " + std::to_string(accumulated_cloud_->size()) + 
                     "\nFrames in buffer: " + std::to_string(cloud_buffer_.size()) +
                     "\nTime window: " + std::to_string(static_cast<int>(time_window_)) + "s" +
                     "\nDisROI: " + std::to_string(disroi_filtered_points_);
        marker_pub_.publish(marker);
    }
}

// *** 新增：发布ROI可视化标记 ***
void MapAccumulatorWithDisROI::publishROIMarkers() {
    visualization_msgs::MarkerArray marker_array;
    
    // Map ROI 可视化（绿色框）
    if (map_roi_params_.enable_roi_filter) {
        visualization_msgs::Marker map_roi_marker;
        map_roi_marker.header.stamp = ros::Time::now();
        map_roi_marker.header.frame_id = map_frame_;
        map_roi_marker.ns = "map_roi";
        map_roi_marker.id = 0;
        map_roi_marker.type = visualization_msgs::Marker::CUBE;
        map_roi_marker.action = visualization_msgs::Marker::ADD;
        
        // 位置和尺寸
        map_roi_marker.pose.position.x = (map_roi_params_.min_x + map_roi_params_.max_x) / 2.0;
        map_roi_marker.pose.position.y = (map_roi_params_.min_y + map_roi_params_.max_y) / 2.0;
        map_roi_marker.pose.position.z = (map_roi_params_.min_z + map_roi_params_.max_z) / 2.0;
        map_roi_marker.pose.orientation.w = 1.0;
        
        map_roi_marker.scale.x = map_roi_params_.max_x - map_roi_params_.min_x;
        map_roi_marker.scale.y = map_roi_params_.max_y - map_roi_params_.min_y;
        map_roi_marker.scale.z = map_roi_params_.max_z - map_roi_params_.min_z;
        
        // 绿色半透明
        map_roi_marker.color.r = 0.0;
        map_roi_marker.color.g = 1.0;
        map_roi_marker.color.b = 0.0;
        map_roi_marker.color.a = 0.2;
        
        marker_array.markers.push_back(map_roi_marker);
    }
    
    // DisROI 可视化（红色框）
    if (disroi_params_.enable_disroi_filter) {
        visualization_msgs::Marker disroi_marker;
        disroi_marker.header.stamp = ros::Time::now();
        disroi_marker.header.frame_id = map_frame_;
        disroi_marker.ns = "disroi";
        disroi_marker.id = 1;
        disroi_marker.type = visualization_msgs::Marker::CUBE;
        disroi_marker.action = visualization_msgs::Marker::ADD;
        
        // 位置和尺寸
        disroi_marker.pose.position.x = (disroi_params_.min_x + disroi_params_.max_x) / 2.0;
        disroi_marker.pose.position.y = (disroi_params_.min_y + disroi_params_.max_y) / 2.0;
        disroi_marker.pose.position.z = (disroi_params_.min_z + disroi_params_.max_z) / 2.0;
        disroi_marker.pose.orientation.w = 1.0;
        
        disroi_marker.scale.x = disroi_params_.max_x - disroi_params_.min_x;
        disroi_marker.scale.y = disroi_params_.max_y - disroi_params_.min_y;
        disroi_marker.scale.z = disroi_params_.max_z - disroi_params_.min_z;
        
        // 红色半透明
        disroi_marker.color.r = 1.0;
        disroi_marker.color.g = 0.0;
        disroi_marker.color.b = 0.0;
        disroi_marker.color.a = 0.3;
        
        marker_array.markers.push_back(disroi_marker);
    }
    
    if (!marker_array.markers.empty()) {
        roi_marker_pub_.publish(marker_array);
    }
}

bool MapAccumulatorWithDisROI::saveMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    save_requested_ = true;
    save_name_ = "service_requested_map";
    ROS_INFO("Save map service requested");
    return true;
}

bool MapAccumulatorWithDisROI::clearMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    clear_requested_ = true;
    ROS_INFO("Clear map service requested");
    return true;
}

bool MapAccumulatorWithDisROI::getInfoService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    publishMapInfo();
    return true;
}

bool MapAccumulatorWithDisROI::saveMap(const std::string& name) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
        ROS_WARN("Cannot save empty map");
        return false;
    }
    std::string filename = generateSaveFilename(name);
    try {
        int result = pcl::io::savePCDFileBinary(filename, *accumulated_cloud_);
        if (result == 0) {
            ROS_INFO("Map saved to: %s (%zu points)", filename.c_str(), accumulated_cloud_->size());
            
            // *** 保存统计信息 ***
            std::string stats_filename = filename.substr(0, filename.find_last_of('.')) + "_stats.txt";
            std::ofstream stats_file(stats_filename);
            if (stats_file.is_open()) {
                stats_file << "=== Map Accumulator Statistics ===\n";
                stats_file << "Total points received: " << total_points_received_ << "\n";
                stats_file << "DisROI filtered points: " << disroi_filtered_points_ 
                          << " (" << (total_points_received_ > 0 ? 100.0 * disroi_filtered_points_ / total_points_received_ : 0.0) << "%)\n";
                stats_file << "ROI filtered points: " << roi_filtered_points_
                          << " (" << (total_points_received_ > 0 ? 100.0 * roi_filtered_points_ / total_points_received_ : 0.0) << "%)\n";
                stats_file << "Accumulated points: " << accumulated_points_
                          << " (" << (total_points_received_ > 0 ? 100.0 * accumulated_points_ / total_points_received_ : 0.0) << "%)\n";
                stats_file << "Frames processed: " << frame_count_ << "\n";
                
                if (disroi_params_.enable_disroi_filter) {
                    stats_file << "DisROI: X[" << disroi_params_.min_x << "," << disroi_params_.max_x << "] "
                              << "Y[" << disroi_params_.min_y << "," << disroi_params_.max_y << "] "
                              << "Z[" << disroi_params_.min_z << "," << disroi_params_.max_z << "]\n";
                }
                if (map_roi_params_.enable_roi_filter) {
                    stats_file << "Map ROI: X[" << map_roi_params_.min_x << "," << map_roi_params_.max_x << "] "
                              << "Y[" << map_roi_params_.min_y << "," << map_roi_params_.max_y << "] "
                              << "Z[" << map_roi_params_.min_z << "," << map_roi_params_.max_z << "]\n";
                }
                
                stats_file.close();
                ROS_INFO("Statistics saved to: %s", stats_filename.c_str());
            }
            
            // 保存轨迹信息
            std::string traj_filename = filename.substr(0, filename.find_last_of('.')) + "_trajectory.txt";
            std::ofstream traj_file(traj_filename);
            if (traj_file.is_open()) {
                for (const auto& pose : trajectory_) {
                    traj_file << pose.pose.position.x << " " 
                             << pose.pose.position.y << " " 
                             << pose.pose.position.z << " "
                             << pose.pose.orientation.x << " "
                             << pose.pose.orientation.y << " "
                             << pose.pose.orientation.z << " "
                             << pose.pose.orientation.w << "\n";
                }
                traj_file.close();
                ROS_INFO("Trajectory saved to: %s", traj_filename.c_str());
            }
            return true;
        } else {
            ROS_ERROR("Failed to save map to: %s", filename.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while saving map: %s", e.what());
        return false;
    }
}

bool MapAccumulatorWithDisROI::loadMap(const std::string& filename) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        int result = pcl::io::loadPCDFile(filename, *loaded_cloud);
        if (result == 0) {
            *accumulated_cloud_ = *loaded_cloud;
            ROS_INFO("Map loaded from: %s (%zu points)", filename.c_str(), accumulated_cloud_->size());
            return true;
        } else {
            ROS_ERROR("Failed to load map from: %s", filename.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while loading map: %s", e.what());
        return false;
    }
}

void MapAccumulatorWithDisROI::backgroundThread() {
    ros::Rate rate(10); // 10Hz
    while (ros::ok()) {
        // 检查是否需要保存
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (save_requested_) {
                saveMap(save_name_);
                save_requested_ = false;
            }
            if (clear_requested_) {
                std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
                accumulated_cloud_->clear();
                cloud_buffer_.clear(); // 清空缓冲区
                trajectory_.clear();
                frame_count_ = 0;
                // *** 重置统计计数器 ***
                total_points_received_ = 0;
                disroi_filtered_points_ = 0;
                roi_filtered_points_ = 0;
                accumulated_points_ = 0;
                clear_requested_ = false;
                ROS_INFO("Map cleared (including buffer and statistics)");
            }
        }

        // 定期自动保存
        if ((ros::Time::now() - last_save_time_).toSec() > save_interval_) {
            saveMap("periodic_save");
            last_save_time_ = ros::Time::now();
        }

        // 定期发布地图信息和ROI可视化
        static int counter = 0;
        if (++counter % 50 == 0) { // 大约每5秒
            publishMapInfo();
            publishROIMarkers(); // *** 新增：发布ROI可视化 ***
        }
        rate.sleep();
    }
}

void MapAccumulatorWithDisROI::realtimePreview() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
        return;
    }
    // 发布当前地图用于实时预览
    sensor_msgs::PointCloud2 preview_msg;
    pcl::toROSMsg(*accumulated_cloud_, preview_msg);
    preview_msg.header.stamp = ros::Time::now();
    preview_msg.header.frame_id = map_frame_;
    map_pub_.publish(preview_msg);
}

std::string MapAccumulatorWithDisROI::getCurrentTimeString() {
    time_t rawtime;
    char buffer[80];
    time(&rawtime);
    strftime(buffer, 80, "%Y%m%d_%H%M%S", localtime(&rawtime));
    return std::string(buffer);
}

std::string MapAccumulatorWithDisROI::generateSaveFilename(const std::string& name) {
    std::string base_name = name.empty() ? "map_" + getCurrentTimeString() : name;
    return save_directory_ + "/" + base_name + ".pcd";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_accumulator");
    ros::NodeHandle nh;
    MapAccumulatorWithDisROI accumulator(nh);
    ros::spin();
    return 0;
}
