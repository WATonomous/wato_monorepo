#include "lidar_publisher.h"
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

LidarPublisher::LidarPublisher() : Node("lidar_publisher")
{
    frameNum = 0;
    data_dir = "/home/docker/ament_ws/src/lidar_publisher/data/data/";
    kitti_data_dir = "/home/docker/ament_ws/kitti-lidar/training/velodyne/";
    

    // initialize the publisher
    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar", 1);
    // create a hello publisher
    pub2 = this->create_publisher<std_msgs::msg::String>(
        "/hello", 1);
    // initialize the timer to run the pub_callback every 1000ms
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&LidarPublisher::pub_callback, this));
}


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

void LidarPublisher::pub_callback()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Reading frame num " << frameNum);

    std::string data_file_path = data_dir;
    std::stringstream ss;

    ss << frameNum;
    frameNum = (frameNum+1)%NUM_FRAMES;

    int n_zero = 6;
    std::string _str = ss.str();
    std::string index_str = std::string(n_zero - _str.length(), '0') + _str;
    data_file_path += index_str;
    data_file_path += ".bin";

    std::cout << "load file: " << data_file_path << std::endl;

    // load points cloud
    unsigned int length = 0;
    void *data = NULL;
    std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());
    loadData(data_file_path.data(), &data, &length);
    buffer.reset((char *)data);

    float *points = (float *)buffer.get();
    size_t points_size = length / sizeof(float) / 4;
    
    // create an XYZ PCL pointcloud2
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = points_size;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    RCLCPP_INFO_STREAM(this->get_logger(), "points size: " << points_size);

    // iterate over the points
    // kiti data format is x, y, z, intensity
    for (int i = 0; i < points_size; i++) {
        float x = points[i * 4 + 0];
        float y = points[i * 4 + 1];
        float z = points[i * 4 + 2];
        float intensity = points[i * 4 + 3];

        // append xyz to cloud
        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;
    }

    // convert to ROS2 message
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);

    // set the frame id
    msg.header.frame_id = "velodyne";
    // set the timestamp
    msg.header.stamp = this->now();

    pub->publish(msg);

    auto helloMsg := std_msgs::msg::String();
    helloMsg.data = "Hello, world!";
    pub2->publish(helloMsg);
}