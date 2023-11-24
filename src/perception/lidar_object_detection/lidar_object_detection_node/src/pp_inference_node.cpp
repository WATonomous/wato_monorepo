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

using std::placeholders::_1;
using namespace std::chrono_literals;

std::string Data_File = "/home/docker/ament_ws/pointpillars_ws/lidar_object_detection_node/src/CUDA-PointPillars/data/";
std::string Save_Dir = "./";
std::string Model_File = "/home/docker/ament_ws/pointpillars_ws/lidar_object_detection_node/src/CUDA-PointPillars/model/backbone.plan";

const char *in_dir = "/home/docker/ament_ws/pointpillars_ws/lidar_object_detection_node/src/CUDA-PointPillars/data/";
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
  vp.min_range = nvtype::Float3(0.0, -39.68f, -3.0);
  vp.max_range = nvtype::Float3(69.12f, 39.68f, 1.0);
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

visualization_msgs::msg::Marker createBoundingBoxMarker(const pointpillar::lidar::BoundingBox& box, int id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link"; // Use the appropriate frame ID
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "bounding_boxes";
    // marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = box.x;
    marker.pose.position.y = box.y;
    marker.pose.position.z = box.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, box.rt);
    // marker.pose.orientation = tf2::toMsg(q);
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

    // Publisher for the point cloud data
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("published_lidar_data", 10);
    bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bounding_boxes", 10);

    // Timer to trigger publishing data every 3 minutes
    timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&LidarDetectionNode::timerCallback, this));


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "done setup");
  }

  ~LidarDetectionNode()
  {
    cudaStreamDestroy(stream_);
  }

private:
  void timerCallback()
  {
    std::vector<std::string> files;
    getFolderFile(in_dir, files);
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

    int count = 0;

    for (const auto &file : files)
    {
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
      // Print out the bounding boxes using RCLCPP_INFO
      for (const auto box : bboxes)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f, id: %d, score: %f", box.x, box.y, box.z, box.w, box.l, box.h, box.rt, box.id, box.score);
        auto marker = createBoundingBoxMarker(box, count++);
        marker_array.markers.push_back(marker);

      }
      bbox_pub_->publish(marker_array);


      // Convert to PCL point cloud
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      for (size_t i = 0; i < length; i += 4 * sizeof(float))
      {
        pcl::PointXYZI point;
        point.x = ((float *)buffer.get())[i];
        point.y = ((float *)buffer.get())[i + 1];
        point.z = ((float *)buffer.get())[i + 2];
        point.intensity = ((float *)buffer.get())[i + 3];
        cloud->points.push_back(point);
      }
      cloud->width = cloud->points.size();
      cloud->height = 1;
      cloud->is_dense = false;

      // Convert to ROS2 message
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cloud, output);
      output.header.frame_id = "base_link";  // Set the appropriate frame id

      // Publish the data
      lidar_pub_->publish(output);



      std::string save_file_name = std::string(out_dir) + file + ".txt";
      SaveBoxPred(bboxes, save_file_name);

      std::cout << ">>>>>>>>>>>" << std::endl;
    }

  checkRuntime(cudaStreamDestroy(stream));

  }

  std::shared_ptr<pointpillar::lidar::Core> core_;
  cudaStream_t stream_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{

  // GetDeviceInfo();

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
