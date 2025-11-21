#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <wato_test/wato_test.hpp>

using namespace std::chrono_literals;

TEST_CASE_METHOD(wato::test::TestExecutorFixture,
                 "KalmanFilterNode publishes predictions for incoming markers",
                 "[integration][kalman_filter]")
{
  // create publisher & subscriber test nodes and add them to the fixture executor
  auto pub_node = std::make_shared<rclcpp::Node>("kf_test_publisher");
  auto sub_node = std::make_shared<rclcpp::Node>("kf_test_subscriber");
  add_node(pub_node);
  add_node(sub_node);

  // start spinning executor in background
  start_spinning();

  // promise/future to wait for a prediction message
  auto prom = std::make_shared<std::promise<visualization_msgs::msg::MarkerArray::SharedPtr>>();
  auto fut = prom->get_future();

  // subscriber: listen for prediction markers and fulfill promise when we see an _pred namespace
  auto sub = sub_node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/prediction/boxes_markers", 10,
    [prom](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      for (const auto &mk : msg->markers) {
        if (mk.ns.find("_pred") != std::string::npos) {
          // set value only once
          try { prom->set_value(msg); } catch (...) {}
          return;
        }
      }
    });

  // publisher to input topic
  auto pub = pub_node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/tracks_markers", 10);

  // small delay for connections to establish
  std::this_thread::sleep_for(50ms);

  // prepare and publish a stamped marker (Kalman node should use per-marker stamp if present)
  auto ma = std::make_shared<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = pub_node->get_clock()->now().to_msg();
  m.ns = "testobject";
  m.id = 42;
  m.pose.position.x = 3.0;
  m.pose.position.y = -1.0;
  m.pose.position.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 4.0; m.scale.y = 2.0; m.scale.z = 1.6;
  ma->markers.push_back(m);

  pub->publish(*ma);

  // wait for prediction to appear
  auto status = fut.wait_for(std::chrono::seconds(5));
  REQUIRE(status == std::future_status::ready);

  auto received = fut.get();
  REQUIRE(received != nullptr);
  REQUIRE(received->markers.size() > 0);

  // basic sanity checks: at least one marker namespace contains "_pred"
  bool found_pred_ns = false;
  for (const auto &mk : received->markers) {
    if (mk.ns.find("_pred") != std::string::npos) { found_pred_ns = true; break; }
  }
  REQUIRE(found_pred_ns == true);

  // allow fixture cleanup to stop spinning and destroy nodes
}