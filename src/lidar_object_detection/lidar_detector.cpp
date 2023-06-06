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

    lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar", 1, std::bind(&LidarDetector::lidarPointsCallback, this, _1));
    lidar_detection_pub = this->create_publisher<common_msgs::msg::ObstacleList>(
        "/lidar_cuda_dets", 1);
    debug_boxes_pub = this->create_publisher<common_msgs::msg::BoundingBoxArray>(
        "/detections_3d", 1);
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
    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;
    checkCudaErrors(cudaStreamCreate(&stream));

    unsigned int length = 0;
    // void *data = NULL;

    // new code

    lidar_cloud_mutex_.lock();

    CloudData cloud_data;
    cloud_data.time = in_sensor_cloud->header.stamp.sec;
    pcl::fromROSMsg(*in_sensor_cloud, *(cloud_data.cloud_ptr));
    cloud_data_buff_.push_back(cloud_data);

    lidar_cloud_mutex_.unlock();

    // std::stringstream pcd_filename;
    // pcd_filename << dump_pcd_path << "/" << std::to_string(cloud_data.time) << ".bin";
    // std::cout << pcd_filename.str() << std::endl;
    // // pcl::io::savePCDFileASCII<pcl::PointXYZI>(pcd_filename.str(), *cloud_data.cloud_ptr);
    // pcl::io::savePCDFileBinary<pcl::PointXYZI>(pcd_filename.str(), *cloud_data.cloud_ptr);

    /**
     * @ Read File Start
     */

    // unsigned int length = 0;
    // void *data = NULL;
    // std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());
    // loadData(pcd_filename.str().data(), &data, &length);
    // buffer.reset((char *)data);

    // float* points = (float*)buffer.get();
    // size_t points_size = length/sizeof(float)/4;

    // float *points_data = nullptr;
    // unsigned int points_data_size = points_size * 4 * sizeof(float);
    // checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
    // checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
    // checkCudaErrors(cudaDeviceSynchronize());
    // remove(pcd_filename.str().data());

    /**
     * @ Read File End
     */

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
            pointcloud_tmp.emplace_back(point.intensity / 255);
        }
    }
    float *points = &pointcloud_tmp[0];
    float *points_data = nullptr;
    unsigned int points_data_size = points_size * 4 * sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
    checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
    checkCudaErrors(cudaDeviceSynchronize());

    // new code end

    // RCLCPP_ERROR(this->getlogger(), "New code end pointcloud_tmp %d || points size %d || nms_pred %d", pointcloud_tmp.size(), points_size, nms_pred.size());
    // for (int i = 0; i < points_size; i++){
    //     if (i > points_size * 0.9){
    //         RCLCPP_ERROR(this->getlogger(), "i: %d || X: %f || Y: %f || Z: %f || I: %f", i, *(points_data+0+i*4), *(points_data+1+i*4), *(points_data+2+i*4), *(points_data+3+i*4));
    //     }
    // }

    if (points_size <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No points detected");
        return;
    }
    pp_mutex_.lock();
    pointpillar->doinfer(points_data, points_size, nms_pred);
    pp_mutex_.unlock();

    checkCudaErrors(cudaFree(points_data));

    publishVis(nms_pred);

    nms_pred.clear();
};

void LidarDetector::publishVis(
    const std::vector<Bndbox> &boxes)
{
    // RCLCPP_ERROR(this->get_logger(), "Num Boxes %d", boxes.size());
    common_msgs::msg::BoundingBoxArray jsk_boxes;
    jsk_boxes.header.frame_id = "base_link";
    for (int i = 0; i < boxes.size(); i++)
    {
        common_msgs::msg::BoundingBox bounding_box;
        bounding_box.header.frame_id = "base_link";
        bounding_box.pose.position.x = boxes.at(i).x;
        bounding_box.pose.position.y = boxes.at(i).y;
        bounding_box.pose.position.z = boxes.at(i).z;
        bounding_box.dimensions.x = boxes.at(i).w;
        bounding_box.dimensions.y = boxes.at(i).l;
        bounding_box.dimensions.z = boxes.at(i).h;

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, boxes.at(i).rt);
        bounding_box.pose.orientation = tf2::toMsg(quat);

        jsk_boxes.boxes.push_back(bounding_box);
    }
    debug_boxes_pub->publish(jsk_boxes);
}
