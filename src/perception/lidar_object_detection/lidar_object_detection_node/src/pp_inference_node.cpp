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
#include <string>

#include "cuda_runtime.h"
#include "../include/pp_infer/pointpillar.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
// #include "../include/pp_infer/point_cloud2_iterator.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

#define checkCudaErrors(status)                                   \
  {                                                               \
    if (status != 0)                                              \
    {                                                             \
      std::cout << "Cuda failure: " << cudaGetErrorString(status) \
                << " at line " << __LINE__                        \
                << " in file " << __FILE__                        \
                << " error status: " << status                    \
                << std::endl;                                     \
      abort();                                                    \
    }                                                             \
  }


int loadData(const char *file, void **data, unsigned int *length)
{
    std::fstream dataFile(file, std::ifstream::in);

    if (!dataFile.is_open()) {
        std::cout << "Can't open files: "<< file<<std::endl;
        return -1;
    }

    unsigned int len = 0;
    dataFile.seekg (0, dataFile.end);
    len = dataFile.tellg();
    dataFile.seekg (0, dataFile.beg);

    char *buffer = new char[len];
    if (buffer==NULL) {
        std::cout << "Can't malloc buffer."<<std::endl;
        dataFile.close();
        exit(EXIT_FAILURE);
    }

    dataFile.read(buffer, len);
    dataFile.close();

    *data = (void*)buffer;
    *length = len;
    return 0;  
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher")
  {
    std::vector<std::string> class_names_values;
    this->declare_parameter<std::vector<std::string>>("class_names", class_names_values);
    this->declare_parameter<float>("nms_iou_thresh", 0.01);
    this->declare_parameter<int>("pre_nms_top_n", 4096);
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::string>("engine_path", "");
    this->declare_parameter<std::string>("data_type", "fp16");
    this->declare_parameter<float>("intensity_scale", 1.0);
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    rclcpp::Parameter class_names_param = this->get_parameter("class_names");
    class_names = class_names_param.as_string_array();
    nms_iou_thresh = this->get_parameter("nms_iou_thresh").as_double();
    pre_nms_top_n = this->get_parameter("pre_nms_top_n").as_int();
    model_path = this->get_parameter("model_path").as_string();
    engine_path = this->get_parameter("engine_path").as_string();
    RCLCPP_INFO(this->get_logger(), " engine path  %s", engine_path.c_str());
    RCLCPP_INFO(this->get_logger(), " model path  %s", model_path.c_str());

    data_type = this->get_parameter("data_type").as_string();
    intensity_scale = this->get_parameter("intensity_scale").as_double();

    cudaStream_t stream = NULL;
    pointpillar = new PointPillar(model_path, engine_path, stream, data_type);
    RCLCPP_INFO(this->get_logger(), " done loading");


    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bbox", 700);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", 700, std::bind(&MinimalPublisher::topic_callback, this, _1));
    dummy_point_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/dummy_point_cloud", default_qos);


    /*
    Load data
    */

    std::vector<std::string> files = {"/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000000.bin", 
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000001.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000002.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000003.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000004.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000005.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000006.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000007.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000008.bin",
    // "/home/docker/ament_ws/src/lidar_object_detection/lidar_object_detection_node/src/data/000009.bin"

    };
    while (true) {

    for (auto dataFile : files) {
        std::cout << "Load file: "<< dataFile <<std::endl;
        RCLCPP_INFO(this->get_logger(), "  load file! %s ", dataFile.c_str());

        //load points cloud
        unsigned int length = 0;
        void *data = NULL;
        std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());
        loadData(dataFile.data(), &data, &length);
        buffer.reset((char *)data);
        int points_size = length/sizeof(float)/4;
        std::cout << "Lidar points count: "<< points_size <<std::endl;

        RCLCPP_INFO(this->get_logger(), "  Lidar points count %d ", points_size);

        std::vector<Bndbox> nms_pred;
        nms_pred.reserve(100);

        float *points = static_cast<float *>(data);

        // Use 4 because PCL has padding (4th value now has intensity information)
        unsigned int points_data_size = points_size * sizeof(float) * 4;

        float *points_data = nullptr;
        unsigned int *points_num = nullptr;
        cudaEvent_t start, stop;
        // unsigned int points_data_size = points_size * num_point_values * sizeof(float);
        checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
        checkCudaErrors(cudaMallocManaged((void **)&points_num, sizeof(unsigned int)));
        checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
        checkCudaErrors(cudaMemcpy(points_num, &points_size, sizeof(unsigned int), cudaMemcpyDefault));
        checkCudaErrors(cudaDeviceSynchronize());

        cudaEventRecord(start, stream);

        pointpillar->doinfer(
            points_data, points_num, nms_pred,
            nms_iou_thresh,
            pre_nms_top_n,
            class_names,
            do_profile);


        RCLCPP_INFO(this->get_logger(), "  done inference! %d ", nms_pred.size());

        // write out the detections
        for (int i = 0; i < nms_pred.size(); i++)
        {
          RCLCPP_INFO(this->get_logger(), "  Box %d: x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f", i, nms_pred.at(i).x, nms_pred.at(i).y, nms_pred.at(i).z, nms_pred.at(i).w, nms_pred.at(i).l, nms_pred.at(i).h, nms_pred.at(i).rt);
          RCLCPP_INFO(this->get_logger(), "  Box %d: id: %d, score: %f", i, nms_pred.at(i).id, nms_pred.at(i).score);
        }

        
        // convert points_data to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        debug_cloud->header.frame_id = "LIDAR_TOP";
        debug_cloud->height = 1;
        debug_cloud->width = points_size;
        debug_cloud->is_dense = false;
        debug_cloud->points.resize(points_size);
        for (int i = 0; i < points_size; i+=4)
        {

          debug_cloud->points[i].x = points[i];
          debug_cloud->points[i].y = points[i+1];
          debug_cloud->points[i].z = points[i+2];
          debug_cloud->points[i].intensity = points[i+3];
        }
        sensor_msgs::msg::PointCloud2 debug_cloud_msg = sensor_msgs::msg::PointCloud2();
        pcl::toROSMsg(*debug_cloud, debug_cloud_msg);
        debug_cloud_msg.header.stamp = this->now();
        dummy_point_cloud->publish(debug_cloud_msg);


    auto pc_detection_arr = std::make_shared<vision_msgs::msg::Detection3DArray>();
    auto timestamp = this->now();
    std::vector<vision_msgs::msg::Detection3D> detections;
    int box_id = 0;
    auto marker_array = visualization_msgs::msg::MarkerArray();
    for (int i = 0; i < nms_pred.size(); i++)
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
      bounding_box.pose.position.x = nms_pred.at(i).x;
      bounding_box.pose.position.y = nms_pred.at(i).y;
      bounding_box.pose.position.z = nms_pred.at(i).z;
      bounding_box.scale.x = nms_pred.at(i).w;
      bounding_box.scale.y = nms_pred.at(i).l;
      bounding_box.scale.z = nms_pred.at(i).h;
      RCLCPP_INFO(this->get_logger(), "Box %d: x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f", i, nms_pred.at(i).x, nms_pred.at(i).y, nms_pred.at(i).z, nms_pred.at(i).w, nms_pred.at(i).l, nms_pred.at(i).h, nms_pred.at(i).rt);

      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0, nms_pred.at(i).rt);

      bounding_box.pose.orientation.x = quat.x();
      bounding_box.pose.orientation.y = quat.y();
      bounding_box.pose.orientation.z = quat.z();
      bounding_box.pose.orientation.w = quat.w();
      bounding_box.text = class_names.at(nms_pred.at(i).id);
      marker_array.markers.push_back(bounding_box);
    }
    publisher_->publish(marker_array);


    }


    }

  }

private:
  std::vector<std::string> class_names;
  float nms_iou_thresh;
  int pre_nms_top_n;
  bool do_profile{false};
  std::string model_path;
  std::string engine_path;
  std::string data_type;
  float intensity_scale;
  tf2::Quaternion myQuaternion;
  cudaStream_t stream = NULL;
  PointPillar *pointpillar;

  void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    assert(data_type == "fp32" || data_type == "fp16");
    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;

    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));

    std::vector<Bndbox> nms_pred;
    nms_pred.reserve(100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    unsigned int num_point_values = pcl_cloud->size();

    unsigned int points_size = pcl_cloud->points.size();

    std::vector<float> pcl_data;

    for (const auto &point : pcl_cloud->points)
    {
      pcl_data.push_back(point.x);
      pcl_data.push_back(point.y);
      pcl_data.push_back(point.z);
      pcl_data.push_back(point.intensity / intensity_scale);
    }

    float *points = static_cast<float *>(pcl_data.data());

    // Use 4 because PCL has padding (4th value now has intensity information)
    unsigned int points_data_size = points_size * sizeof(float) * 4;

    float *points_data = nullptr;
    unsigned int *points_num = nullptr;
    // unsigned int points_data_size = points_size * num_point_values * sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
    checkCudaErrors(cudaMallocManaged((void **)&points_num, sizeof(unsigned int)));
    checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
    checkCudaErrors(cudaMemcpy(points_num, &points_size, sizeof(unsigned int), cudaMemcpyDefault));
    checkCudaErrors(cudaDeviceSynchronize());

    cudaEventRecord(start, stream);

    pointpillar->doinfer(
        points_data, points_num, nms_pred,
        nms_iou_thresh,
        pre_nms_top_n,
        class_names,
        do_profile);

    auto pc_detection_arr = std::make_shared<vision_msgs::msg::Detection3DArray>();
    auto timestamp = this->now();
    std::vector<vision_msgs::msg::Detection3D> detections;
    int box_id = 0;
    auto marker_array = visualization_msgs::msg::MarkerArray();
    for (int i = 0; i < nms_pred.size(); i++)
    {
      // vision_msgs::msg::Detection3D detection;
      // detection.results.resize(1);
      // vision_msgs::msg::ObjectHypothesisWithPose hyp;
      // vision_msgs::msg::BoundingBox3D bbox;
      // geometry_msgs::msg::Pose center;
      // geometry_msgs::msg::Vector3 size;
      // geometry_msgs::msg::Point position;
      // geometry_msgs::msg::Quaternion orientation;

      // detection.bbox.center.position.x = nms_pred[i].x;
      // detection.bbox.center.position.y = nms_pred[i].y;
      // detection.bbox.center.position.z = nms_pred[i].z;
      // detection.bbox.size.x = nms_pred[i].l;
      // detection.bbox.size.y = nms_pred[i].w;
      // detection.bbox.size.z = nms_pred[i].h;

      // myQuaternion.setRPY(0, 0, nms_pred[i].rt);
      // orientation = tf2::toMsg(myQuaternion);

      // detection.bbox.center.orientation = orientation;

      // // hyp.id = std::to_string(nms_pred[i].id);
      // // hyp.score = nms_pred[i].score;

      // detection.header = msg->header;

      // detection.results[0] = hyp;
      // detections.push_back(detection);

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
      bounding_box.pose.position.x = nms_pred.at(i).x;
      bounding_box.pose.position.y = nms_pred.at(i).y;
      bounding_box.pose.position.z = nms_pred.at(i).z;
      bounding_box.scale.x = nms_pred.at(i).w;
      bounding_box.scale.y = nms_pred.at(i).l;
      bounding_box.scale.z = nms_pred.at(i).h;
      RCLCPP_INFO(this->get_logger(), "Box %d: x: %f, y: %f, z: %f, w: %f, l: %f, h: %f, rt: %f", i, nms_pred.at(i).x, nms_pred.at(i).y, nms_pred.at(i).z, nms_pred.at(i).w, nms_pred.at(i).l, nms_pred.at(i).h, nms_pred.at(i).rt);

      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0, nms_pred.at(i).rt);

      bounding_box.pose.orientation.x = quat.x();
      bounding_box.pose.orientation.y = quat.y();
      bounding_box.pose.orientation.z = quat.z();
      bounding_box.pose.orientation.w = quat.w();
      bounding_box.text = class_names.at(nms_pred.at(i).id);
      marker_array.markers.push_back(bounding_box);
    }
    publisher_->publish(marker_array);

    // convert points_data to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    debug_cloud->header.frame_id = "LIDAR_TOP";
    debug_cloud->height = 1;
    debug_cloud->width = points_size;
    debug_cloud->is_dense = false;
    debug_cloud->points.resize(points_size);
    for (int i = 0; i < pcl_cloud->points.size(); i++)
    {

      debug_cloud->points[i].x = pcl_cloud->points[i].x;
      debug_cloud->points[i].y = pcl_cloud->points[i].y;
      debug_cloud->points[i].z = pcl_cloud->points[i].z;
      debug_cloud->points[i].intensity = pcl_cloud->points[i].intensity;
    }
    sensor_msgs::msg::PointCloud2 debug_cloud_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*debug_cloud, debug_cloud_msg);
    debug_cloud_msg.header.stamp = timestamp;
    dummy_point_cloud->publish(debug_cloud_msg);

    // pc_detection_arr->header = msg->header;
    // pc_detection_arr->detections = detections;
    // publisher_->publish(*pc_detection_arr);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);

    auto message = std_msgs::msg::String();
    message.data = "TIME: " + std::to_string(elapsedTime) + " ms, Objects detected: " + std::to_string(nms_pred.size());
    RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());

    checkCudaErrors(cudaFree(points_data));
    checkCudaErrors(cudaFree(points_num));
    nms_pred.clear();

    checkCudaErrors(cudaEventDestroy(start));
    checkCudaErrors(cudaEventDestroy(stop));
    checkCudaErrors(cudaStreamDestroy(stream));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dummy_point_cloud;

  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
