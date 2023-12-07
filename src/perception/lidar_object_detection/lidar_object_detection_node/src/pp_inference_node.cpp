/*
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#define BOOST_BIND_NO_PLACEHOLDERS

#include <chrono>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <map>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <unistd.h>
#include <dirent.h>
#include <string>

#include "cuda_runtime.h"

#include "pointpillar/pointpillar.hpp"
#include "common/check.hpp"
// #include "../include/pp_infer/pointpillar.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
// #include "pcl_conversions/pcl_conversions.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// #include "../include/pp_infer/point_cloud2_iterator.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cmath> // For M_PI

using std::placeholders::_1;
using namespace std::chrono_literals;

std::string Data_File = "/home/docker/ament_ws/pointpillars_ws/lidar_object_detection_node/src/CUDA-PointPillars/data/";
std::string Save_Dir = "./";
std::string Model_File = "/home/docker/ament_ws/pointpillars_ws/lidar_object_detection_node/src/CUDA-PointPillars/model/backbone.plan";

// const char *in_dir = "/home/docker/ament_ws/pointpillars_ws/lidar_object_detection_node/src/CUDA-PointPillars/data/";
const char *in_dir = "/data/";
const char *out_dir = "/tmp/";

void GetDeviceInfo(void)
{

  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i)
  {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}

bool hasEnding(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length())
  {
    return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
  }
  else
  {
    return false;
  }
}

int getFolderFile(const char *path, std::vector<std::string> &files, const char *suffix = ".bin")
{
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(path)) != NULL)
  {
    while ((ent = readdir(dir)) != NULL)
    {
      std::string file = ent->d_name;
      if (hasEnding(file, suffix))
      {
        files.push_back(file.substr(0, file.length() - 4));
      }
    }
    closedir(dir);
  }
  else
  {
    printf("No such folder: %s.", path);
    exit(EXIT_FAILURE);
  }
  return EXIT_SUCCESS;
}

int loadData(const char *file, void **data, unsigned int *length)
{
  std::fstream dataFile(file, std::ifstream::in);

  if (!dataFile.is_open())
  {
    std::cout << "Can't open files: " << file << std::endl;
    return -1;
  }

  unsigned int len = 0;
  dataFile.seekg(0, dataFile.end);
  len = dataFile.tellg();
  dataFile.seekg(0, dataFile.beg);

  char *buffer = new char[len];
  if (buffer == NULL)
  {
    std::cout << "Can't malloc buffer." << std::endl;
    dataFile.close();
    exit(EXIT_FAILURE);
  }

  dataFile.read(buffer, len);
  dataFile.close();

  *data = (void *)buffer;
  *length = len;
  return 0;
}

void SaveBoxPred(std::vector<pointpillar::lidar::BoundingBox> boxes, std::string file_name)
{
  std::ofstream ofs;
  ofs.open(file_name, std::ios::out);
  if (ofs.is_open())
  {
    for (const auto box : boxes)
    {
      ofs << box.x << " ";
      ofs << box.y << " ";
      ofs << box.z << " ";
      ofs << box.w << " ";
      ofs << box.l << " ";
      ofs << box.h << " ";
      ofs << box.rt << " ";
      ofs << box.id << " ";
      ofs << box.score << "\n";
    }
  }
  else
  {
    std::cerr << "Output file cannot be opened!" << std::endl;
  }
  ofs.close();
  std::cout << "Saved prediction in: " << file_name << std::endl;
  return;
};

std::shared_ptr<pointpillar::lidar::Core> create_core()
{
  pointpillar::lidar::VoxelizationParameter vp;
  vp.min_range = nvtype::Float3(0, -39.68f, -3.0);
  vp.max_range = nvtype::Float3(69.12f, 39.68f, 1.0);
  // vp.min_range = nvtype::Float3(0.0, -39.68f, -3.0);
  // vp.max_range = nvtype::Float3(69.12f, 39.68f, 1.0);
  vp.voxel_size = nvtype::Float3(0.16f, 0.16f, 4.0f);
  vp.grid_size =
      vp.compute_grid_size(vp.max_range, vp.min_range, vp.voxel_size);
  vp.max_voxels = 40000;
  vp.max_points_per_voxel = 32;
  vp.max_points = 300000;
  vp.num_feature = 4;

  pointpillar::lidar::PostProcessParameter pp;
  pp.min_range = vp.min_range;
  pp.max_range = vp.max_range;
  pp.feature_size = nvtype::Int2(vp.grid_size.x / 2, vp.grid_size.y / 2);

  pointpillar::lidar::CoreParameter param;
  param.voxelization = vp;
  param.lidar_model = Model_File;
  param.lidar_post = pp;
  return pointpillar::lidar::create_core(param);
}

static bool startswith(const char *s, const char *with, const char **last)
{
  while (*s++ == *with++)
  {
    if (*s == 0 || *with == 0)
      break;
  }
  if (*with == 0)
    *last = s + 1;
  return *with == 0;
}

static void help()
{
  printf(
      "Usage: \n"
      "    ./pointpillar in/ out/ --timer\n"
      "    Run pointpillar inference with .bin under in, save .text under out\n"
      "    Optional: --timer, enable timer log\n");
  exit(EXIT_SUCCESS);
}

visualization_msgs::msg::Marker createBoundingBoxMarker(const pointpillar::lidar::BoundingBox &box, int id, rclcpp::Time stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link"; // Use the appropriate frame ID
  marker.header.stamp = stamp;
  marker.ns = "bounding_boxes";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = box.x;
  marker.pose.position.y = box.y;
  marker.pose.position.z = box.z;
  tf2::Quaternion q;
  const double YAW_OFFSET = M_PI / 2.0; // Offset of π/2 to align coordinate systems
  q.setRPY(0, 0, box.rt + YAW_OFFSET);
  marker.pose.orientation = tf2::toMsg(q);
  marker.scale.x = box.l;
  marker.scale.y = box.w;
  marker.scale.z = box.h;
  marker.color.a = 1.0; // Don't forget to set the alpha
  marker.color.r = 1.0; // Adjust color as needed
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  return marker;
}

class LidarDetectionNode : public rclcpp::Node
{
public:
  LidarDetectionNode()
      : Node("lidar_detection_node")
  {
    GetDeviceInfo();
    core_ = create_core();
    if (core_ == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Core initialization failed.");
      exit(EXIT_FAILURE);
    }
    cudaStreamCreate(&stream_);

    // Subscriber callback
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR_TOP", 10,
        std::bind(&LidarDetectionNode::lidarCallback, this, _1));

    // Publisher for the point cloud data
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("published_lidar_data", 100);
    lidar_timer_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("published_timer_lidar_data", 100);
    bbox_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("bounding_boxes", 10);

    // Timer to trigger publishing data every 3 minutes
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&LidarDetectionNode::timerCallback, this));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "done setup");
  }

  ~LidarDetectionNode()
  {
    cudaStreamDestroy(stream_);
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    cudaStream_t stream;
    cudaStreamCreate(&stream);

    // Convert ROS2 message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    // Publish the lidar data from cloud variable
    // Convert to ROS2 message
    sensor_msgs::msg::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Prepare data for inference
    std::vector<float> data;
    for (const auto &point : cloud->points)
    {
      // compute the distance and filter if it is too far
      float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (distance > 32.0 || point.y < 0)
      {
        continue;
      }
      auto newPoint = pcl::PointXYZI();
      newPoint.x = point.y;
      newPoint.y = point.x;
      newPoint.z = point.z;
      newPoint.intensity = point.intensity;

      filteredCloud->points.push_back(newPoint);
    }

    // Allocate buffer for the data
    unsigned int length = filteredCloud->points.size() * sizeof(float) * 4;
    char *buffer = new char[length];
    if (buffer == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffer.");
      return;
    }

    // Copy data to buffer
    for (size_t i = 0, j = 0; i < filteredCloud->points.size(); ++i, j += 4)
    {
      float *ptr = reinterpret_cast<float *>(buffer + j * sizeof(float));
      ptr[0] = filteredCloud->points[i].x;
      ptr[1] = filteredCloud->points[i].y;
      ptr[2] = filteredCloud->points[i].z;
      ptr[3] = filteredCloud->points[i].intensity;
    }

    // Run inference
    int points_size = length / sizeof(float) / 4;
    std::shared_ptr<char> managed_buffer(buffer, std::default_delete<char[]>());
    auto bboxes = core_->forward(reinterpret_cast<float *>(managed_buffer.get()), points_size, stream_);


    pcl::toROSMsg(*filteredCloud, output);
    output.header.frame_id = "base_link";
    output.header.stamp = this->now();

    // Publish the data
    lidar_pub_->publish(output);

    // Publish bounding boxes
    publishBoundingBoxes(bboxes);

    checkRuntime(cudaStreamDestroy(stream));
  }


  void timerCallback()
  {
    std::vector<std::string> files;
    getFolderFile(in_dir, files);
    std::sort(files.begin(), files.end());
    std::cout << "Total " << files.size() << std::endl;
    bool timer = false;

    auto core = create_core();
    if (core == nullptr)
    {
      printf("Core has been failed.\n");
    }

    cudaStream_t stream;
    cudaStreamCreate(&stream);

    core->print();
    core->set_timer(timer);
    cnt = (cnt+1)%files.size();
    {
      auto file = files[cnt];

      std::string dataFile = std::string(in_dir) + file + ".bin";

      std::cout << "\n<<<<<<<<<<<" << std::endl;
      std::cout << "Load file: " << dataFile << std::endl;

      // load points cloud
      unsigned int length = 0;
      void *data = NULL;
      std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());
      loadData(dataFile.data(), &data, &length);
      buffer.reset((char *)data);
      int points_size = length / sizeof(float) / 4;
      std::cout << "Lidar points count: " << points_size << std::endl;

      auto bboxes = core->forward((float *)buffer.get(), points_size, stream);
      std::cout << "Detections after NMS: " << bboxes.size() << std::endl;

      visualization_msgs::msg::MarkerArray marker_array;
      rclcpp::Time stamp = this->now();
      // Print out the bounding boxes using RCLCPP_INFO
      for (int i = 0; i < bboxes.size(); i++)
      {
        auto box = bboxes[i];
        if (box.score < 0.5)
        {
          continue;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f, id: %d, score: %f", box.x, box.y, box.z, box.w, box.l, box.h, box.rt, box.id, box.score);
        auto marker = createBoundingBoxMarker(box, i, stamp);
        marker_array.markers.push_back(marker);
        bbox_pub_->publish(marker);
      }

      // Convert to PCL point cloud
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      for (size_t i = 0; i < length; i += sizeof(float) * 4)
      {
        pcl::PointXYZI point;
        point.x = *((float *)(buffer.get() + i));
        point.y = *((float *)(buffer.get() + i + sizeof(float)));
        point.z = *((float *)(buffer.get() + i + 2 * sizeof(float)));
        point.intensity = *((float *)(buffer.get() + i + 3 * sizeof(float)));

        cloud->points.push_back(point);
      }
      cloud->width = cloud->points.size();
      cloud->height = 1;
      cloud->is_dense = false;

      // Convert to ROS2 message
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cloud, output);
      output.header.frame_id = "base_link"; // Set the appropriate frame id
      output.header.stamp = stamp;

      // Publish the data
      lidar_timer_pub_->publish(output);

      std::string save_file_name = std::string(out_dir) + file + ".txt";
      SaveBoxPred(bboxes, save_file_name);

      std::cout << ">>>>>>>>>>>" << std::endl;
      cnt += 1;
    }

    checkRuntime(cudaStreamDestroy(stream));
  }

  void publishBoundingBoxes(const std::vector<pointpillar::lidar::BoundingBox> &bboxes)
  {
    
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time stamp = this->now();
    // publish DELETEALL marker to clear the previous bounding boxes
    visualization_msgs::msg::MarkerArray delete_all_marker_array;
    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.header.frame_id = "base_link";
    delete_all_marker.header.stamp = this->now();
    delete_all_marker.ns = "bounding_boxes";
    delete_all_marker.id = 0;
    delete_all_marker.type = visualization_msgs::msg::Marker::DELETEALL;
    bbox_pub_->publish(delete_all_marker);
    for (int i = 0; i < bboxes.size(); ++i)
    {
      if (bboxes[i].score < 0.5)
        continue;
      auto marker = createBoundingBoxMarker(bboxes[i], i, stamp);
      // put the score in the text field of the marker
      marker.text = std::to_string(bboxes[i].score);

      // color the marker based on the id
      switch (bboxes[i].id)
      {
      // red
      case 0:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      // green
      case 1:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      // blue
      case 2:
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        break;
      }
      marker_array.markers.push_back(marker);
      bbox_pub_->publish(marker);

      // Log the boudning box result
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f, id: %d, score: %f", bboxes[i].x, bboxes[i].y, bboxes[i].z, bboxes[i].w, bboxes[i].l, bboxes[i].h, bboxes[i].rt, bboxes[i].id, bboxes[i].score);
    }
  }

  std::shared_ptr<pointpillar::lidar::Core> core_;
  int cnt = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  cudaStream_t stream_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_timer_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bbox_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{

  GetDeviceInfo();

  // std::cout << "Hello world pp" << std::endl;

  // std::vector<std::string> files;
  // getFolderFile(in_dir, files);
  // std::cout << "Total " << files.size() << std::endl;
  // bool timer = false;

  // auto core = create_core();
  // if (core == nullptr)
  // {
  //   printf("Core has been failed.\n");
  //   return -1;
  // }

  // cudaStream_t stream;
  // cudaStreamCreate(&stream);

  // core->print();
  // core->set_timer(timer);

  // for (const auto &file : files)
  // {
  //   std::string dataFile = std::string(in_dir) + file + ".bin";

  //   std::cout << "\n<<<<<<<<<<<" << std::endl;
  //   std::cout << "Load file: " << dataFile << std::endl;

  //   // load points cloud
  //   unsigned int length = 0;
  //   void *data = NULL;
  //   std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());
  //   loadData(dataFile.data(), &data, &length);
  //   buffer.reset((char *)data);
  //   int points_size = length / sizeof(float) / 4;
  //   std::cout << "Lidar points count: " << points_size << std::endl;

  //   auto bboxes = core->forward((float *)buffer.get(), points_size, stream);
  //   std::cout << "Detections after NMS: " << bboxes.size() << std::endl;

  //   // Print out the bounding boxes using RCLCPP_INFO
  //   for (const auto box : bboxes)
  //   {
  //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f, id: %d, score: %f", box.x, box.y, box.z, box.w, box.l, box.h, box.rt, box.id, box.score);
  //   }

  //   std::string save_file_name = std::string(out_dir) + file + ".txt";
  //   SaveBoxPred(bboxes, save_file_name);

  //   std::cout << ">>>>>>>>>>>" << std::endl;
  // }

  // checkRuntime(cudaStreamDestroy(stream));

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
