#include "lidar_detector.h"

#define checkCudaErrors(status)                                         \
    {                                                                   \
        if (status != 0)                                                \
        {                                                               \
            std::cout << "Cuda failure: " << cudaGetErrorString(status) \
                      << " at line " << __LINE__                        \
                      << " in file " << __FILE__                        \
                      << " error status: " << status                    \
                      << std::endl;                                     \
            abort();                                                    \
        }                                                               \
    }

using std::placeholders::_1;

LidarDetector::LidarDetector() : Node("lidar_object_detection")
{
    // Params params_;
    stream = NULL;
    checkCudaErrors(cudaStreamCreate(&stream));

    std::string model_file = "/home/docker/ament_ws/pointpillar.onnx";
    RCLCPP_INFO(this->get_logger(), "Initializing model");
    pointpillar = std::make_shared<PointPillar>(model_file, stream);
    RCLCPP_INFO(this->get_logger(), "Model is initialized");

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR_TOP", 1, std::bind(&LidarDetector::lidarPointsCallback, this, _1));
    lidar_detection_pub = this->create_publisher<common_msgs::msg::ObstacleList>(
        "/lidar_cuda_dets", 1);
    // debug_boxes_pub = this->create_publisher<common_msgs::msg::BoundingBoxArray>(
    //     "/detections_3d", 1);
    debug_boxes_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "/detections_3d_debug", default_qos);
    publisher_ = this->create_publisher<std_msgs::msg::String>("simple_topic", default_qos);
    dummy_point_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/dummy_point_cloud", default_qos);
};

int loadData(const char *file, void **data, unsigned int *length)
{
    std::fstream dataFile(file, std::ifstream::in);

    if (!dataFile.is_open())
    {
        std::cout << "Can't open files: " << file << std::endl;
        return -1;
    }

    // get length of file:
    unsigned int len = 0;
    dataFile.seekg(0, dataFile.end);
    len = dataFile.tellg();
    dataFile.seekg(0, dataFile.beg);

    // allocate memory:
    char *buffer = new char[len];
    if (buffer == NULL)
    {
        std::cout << "Can't malloc buffer." << std::endl;
        dataFile.close();
        exit(-1);
    }

    // read data as a block:
    dataFile.read(buffer, len);
    dataFile.close();

    *data = (void *)buffer;
    *length = len;
    return 0;
}

void LidarDetector::test()
{
    RCLCPP_INFO(this->get_logger(), "Test function");

    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;

    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));

    std::vector<Bndbox> nms_pred;
    nms_pred.reserve(100);

    for (int i = 0; i < 10; i++)
    {
        std::string dataFile = "/home/docker/ament_ws/src/lidar_object_detection/data/data/";
        std::string Save_Dir = "/home/docker/ament_ws/lidar_object_detection";
        std::stringstream ss;

        ss << i;

        int n_zero = 6;
        std::string _str = ss.str();
        std::string index_str = std::string(n_zero - _str.length(), '0') + _str;
        dataFile += index_str;
        dataFile += ".bin";

        std::cout << "<<<<<<<<<<<" << std::endl;
        std::cout << "load file: " << dataFile << std::endl;

        // load points cloud
        unsigned int length = 0;
        void *data = NULL;
        std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());
        loadData(dataFile.data(), &data, &length);
        buffer.reset((char *)data);

        float *points = (float *)buffer.get();
        size_t points_size = length / sizeof(float) / 4;

        std::cout << "find points num: " << points_size << std::endl;

        float *points_data = nullptr;
        unsigned int points_data_size = points_size * 4 * sizeof(float);
        checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
        checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
        checkCudaErrors(cudaDeviceSynchronize());

        cudaEventRecord(start, stream);

        pointpillar->doinfer(points_data, points_size, nms_pred);
        cudaEventRecord(stop, stream);
        cudaEventSynchronize(stop);
        cudaEventElapsedTime(&elapsedTime, start, stop);
        std::cout << "TIME: pointpillar: " << elapsedTime << " ms." << std::endl;

        checkCudaErrors(cudaFree(points_data));

        std::cout << "Bndbox objs: " << nms_pred.size() << std::endl;
        // std::string save_file_name = Save_Dir + index_str + ".txt";
        // SaveBoxPred(nms_pred, save_file_name);

        nms_pred.clear();

        std::cout << ">>>>>>>>>>>" << std::endl;
    }
}

void LidarDetector::lidarPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr in_sensor_cloud)
{

    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2!!";
    publisher_->publish(message);

    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;
    checkCudaErrors(cudaStreamCreate(&stream));

    unsigned int length = 0;

    lidar_cloud_mutex_.lock();

    CloudData cloud_data;
    cloud_data.time = in_sensor_cloud->header.stamp.sec;
    pcl::fromROSMsg(*in_sensor_cloud, *(cloud_data.cloud_ptr));
    cloud_data_buff_.push_back(cloud_data);

    lidar_cloud_mutex_.unlock();

    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();
    size_t points_size = current_cloud_data_.cloud_ptr->size();

    std::vector<float> pointcloud_tmp;
    for (auto point : current_cloud_data_.cloud_ptr->points)
    {
        if (!std::isnan(point.x))
        {
            pointcloud_tmp.emplace_back(point.x);
            pointcloud_tmp.emplace_back(point.y);
            pointcloud_tmp.emplace_back(point.z);
            pointcloud_tmp.emplace_back(point.intensity / 255.0);
        }
    }
    float *points = &pointcloud_tmp[0];
    float *points_data = nullptr;
    unsigned int points_data_size = points_size * 4 * sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
    checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
    checkCudaErrors(cudaDeviceSynchronize());
    RCLCPP_INFO(this->get_logger(), "points_size: %d", points_size);
    if (points_size <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No points detected");
        return;
    }
    // convert points_data to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    debug_cloud->header.frame_id = "LIDAR_TOP";
    debug_cloud->height = 1;
    debug_cloud->width = points_size;
    debug_cloud->is_dense = false;
    debug_cloud->points.resize(points_size);
    for (int i = 0; i < points_size; i++)
    {
        // if distance >= 10 set the coords to 0
        if (sqrt(points_data[i * 4 + 0] * points_data[i * 4 + 0] + points_data[i * 4 + 1] * points_data[i * 4 + 1] + points_data[i * 4 + 2] * points_data[i * 4 + 2]) >= 25)
        {
            points_data[i * 4 + 0] = 0;
            points_data[i * 4 + 1] = 0;
            points_data[i * 4 + 2] = 0;
        } 
        debug_cloud->points[i].x = points_data[i * 4 + 0];
        debug_cloud->points[i].y = points_data[i * 4 + 1];
        debug_cloud->points[i].z = points_data[i * 4 + 2];
        debug_cloud->points[i].intensity = points_data[i * 4 + 3] * 255.0;
    }
    sensor_msgs::msg::PointCloud2 debug_cloud_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*debug_cloud, debug_cloud_msg);

    pp_mutex_.lock();
    pointpillar->doinfer(points_data, points_size, nms_pred);
    pp_mutex_.unlock();

    checkCudaErrors(cudaFree(points_data));

    publishVis(nms_pred);
    dummy_point_cloud->publish(debug_cloud_msg);

    nms_pred.clear();
};

void LidarDetector::publishVis(
    const std::vector<Bndbox> &boxes)
{
    RCLCPP_INFO(this->get_logger(), "Num Boxes %d", boxes.size());
    auto timestamp = this->now();
    int box_id = 0;
    for (int i = 0; i < boxes.size(); i++)
    {
        auto bounding_box = visualization_msgs::msg::Marker();
        bounding_box.header.frame_id = "LIDAR_TOP";
        bounding_box.header.stamp = timestamp;
        bounding_box.ns = "bounding_boxes";
        bounding_box.id = box_id++;
        bounding_box.type = visualization_msgs::msg::Marker::CUBE;
        bounding_box.action = visualization_msgs::msg::Marker::ADD;

        bounding_box.color.r = 1.0;
        bounding_box.color.g = 0.0;
        bounding_box.color.b = 0.0;
        bounding_box.color.a = 1.0;
        bounding_box.pose.position.x = boxes.at(i).x;
        bounding_box.pose.position.y = boxes.at(i).y;
        bounding_box.pose.position.z = boxes.at(i).z;
        bounding_box.scale.x = boxes.at(i).l;
        bounding_box.scale.y = boxes.at(i).w;
        bounding_box.scale.z = boxes.at(i).h;
        RCLCPP_INFO(this->get_logger(), "Box %d: x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f", i, boxes.at(i).x, boxes.at(i).y, boxes.at(i).z, boxes.at(i).w, boxes.at(i).l, boxes.at(i).h, boxes.at(i).rt);

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, boxes.at(i).rt);
        // turn it -90 degrees
        tf2::Quaternion quat_offset;
        quat_offset.setRPY(0.0, 0.0, -1.5708);
        quat = quat * quat_offset;

        bounding_box.pose.orientation.x = quat.x();
        bounding_box.pose.orientation.y = quat.y();
        bounding_box.pose.orientation.z = quat.z();
        bounding_box.pose.orientation.w = quat.w();

        debug_boxes_pub->publish(bounding_box);
    }
}
