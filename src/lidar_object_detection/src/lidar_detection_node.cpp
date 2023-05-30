// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include <iostream>
#include <sstream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "cuda_runtime.h"

#include "./params.h"
#include "./pointpillar.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LidarDetectionNode : public rclcpp::Node
{
public:
  LidarDetectionNode()
  : Node("lidar_object_detection"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&LidarDetectionNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing 2: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;


};
  std::string home_path = "/home/docker/ament_ws/lidar_object_detection/";
  std::string data_path = home_path + "data/";
  std::string model_path = home_path + "model/pointpillar.onnx";


void Getinfo(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----device id: %d info----\n", i);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  GPU : %s \n", prop.name);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Capbility: %d.%d\n", prop.major, prop.minor);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Const memory: %luKB\n", prop.totalConstMem  >> 10);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  warp size: %d\n", prop.warpSize);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  threads in a block: %d\n", prop.maxThreadsPerBlock);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
}


int loadData(const char *file, void **data, unsigned int *length)
{
  std::fstream dataFile(file, std::ifstream::in);

  if (!dataFile.is_open())
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Can't open files: "<< file);
	  std::cout << "Can't open files: "<< file<<std::endl;
	  return -1;
  }

  //get length of file:
  unsigned int len = 0;
  dataFile.seekg (0, dataFile.end);
  len = dataFile.tellg();
  dataFile.seekg (0, dataFile.beg);

  //allocate memory:
  char *buffer = new char[len];
  if(buffer==NULL) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Can't malloc buffer."<<std::endl);
	  std::cout << "Can't malloc buffer."<<std::endl;
    dataFile.close();
	  exit(-1);
  }

  //read data as a block:
  dataFile.read(buffer, len);
  dataFile.close();

  *data = (void*)buffer;
  *length = len;
  return 0;  
}


int main(int argc, char * argv[])
{
  Getinfo();  // print GPU info
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing lidar node...");


  cudaEvent_t start, stop;
  float elapsedTime = 0.0f;
  cudaStream_t stream = NULL;

  checkCudaErrors(cudaEventCreate(&start));
  checkCudaErrors(cudaEventCreate(&stop));
  checkCudaErrors(cudaStreamCreate(&stream));

  Params params_;

  std::vector<Bndbox> nms_pred;
  nms_pred.reserve(100);

  PointPillar pointpillar(model_path, stream);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized pointpillar model...");


  rclcpp::spin(std::make_shared<LidarDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
