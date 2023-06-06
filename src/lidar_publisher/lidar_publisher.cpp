#include "lidar_publisher.h"
#include <pcl_conversions/pcl_conversions.h>


using std::placeholders::_1;

LidarPublisher::LidarPublisher() : Node("lidar_publisher")
{
    frameNum = 0;
    data_dir = "/home/docker/ament_ws/src/lidar_publisher/data/data/";

    
    // initialize the publisher
    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar_point_cloud", 1);
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
    // log hello
    RCLCPP_INFO_STREAM(this->get_logger(), "Reading frame num " << frameNum);

    std::string dataFile = "/home/docker/ament_ws/src/lidar_publisher/data/data/";
    std::string Save_Dir = "/home/docker/ament_ws/lidar_publisher";
    std::stringstream ss;

    ss << frameNum;
    frameNum = (frameNum+1)%NUM_FRAMES;

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
    
    // create an XYZ PCL pointcloud2
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = points_size;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

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
    
    std::cout << "find points num: " << points_size << std::endl;

    float *points_data = nullptr;
    unsigned int points_data_size = points_size * 4 * sizeof(float);
}