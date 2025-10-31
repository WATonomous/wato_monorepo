#include "tracking_2d.hpp"

tracking_2d::tracking_2d() : Node("tracking_2d") {
  initializeParams();

  // Subscribers
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detections_topic_, 10,
    std::bind(&tracking_2d::detectionsCallback, this, std::placeholders::_1)
  );

  // Publishers
  tracked_dets_pub_ =
      this->create_publisher<tracking_2d_msgs::msg::Tracking2DArray>(track_topic_, 10);

  // ByteTrack tracker
  tracker_ = std::make_unique<byte_track::BYTETracker>(frame_rate_, track_buffer_, track_thresh_, high_thresh_, match_thresh_);
}

void tracking_2d::initializeParams() {
  // Declare parameters
  this->declare_parameter<std::string>("detections_topic", "/camera_object_detections");
  this->declare_parameter<std::string>("track_topic", "/tracked_boxes");

  this->declare_parameter<int>("frame_rate", 30);
  this->declare_parameter<int>("track_buffer", 30);
  this->declare_parameter<double>("track_thresh", 0.5);
  this->declare_parameter<double>("high_thresh", 0.6);
  this->declare_parameter<double>("match_thresh", 0.8);

  // Get parameters
  bool params_ok = true;

  // Automatic type assignment, catch errors with params_ok
  params_ok &= this->get_parameter("detections_topic", detections_topic_);  // string
  params_ok &= this->get_parameter("track_topic", track_topic_);            // string

  params_ok &= this->get_parameter("frame_rate", frame_rate_);              // int
  params_ok &= this->get_parameter("track_buffer", track_buffer_);          // int
  params_ok &= this->get_parameter("track_thresh", track_thresh_);          // float
  params_ok &= this->get_parameter("high_thresh", high_thresh_);            // float
  params_ok &= this->get_parameter("match_thresh", match_thresh_);          // float

  if (!params_ok) RCLCPP_WARN(this->get_logger(), "One or more parameters could not be initialized");
  else RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}


// Convert from ros msgs to bytetrack's required format
std::vector<byte_track::Object> tracking_2d::detsToObjects(
    const vision_msgs::msg::Detection2DArray::SharedPtr dets
) {
  std::vector<byte_track::Object> objs;
  objs.reserve(dets->detections.size());

  int inc_id = 0;  // Manually set incrementing id (approach subject to change)
  for (const auto &det : dets->detections) {
    // Convert from Detection2D to byte_track::Object
    float width = det.bbox.size_x;
    float height = det.bbox.size_y;
    float x = det.bbox.center.position.x - width / 2;
    float y = det.bbox.center.position.y - height / 2;

    byte_track::Rect<float> rect{x, y, width, height};
    int label;
    float prob;

    byte_track::Object obj(rect, label, prob);

    // Get highest scored hypothesis
    auto best_hyp = std::max_element(det.results.begin(), det.results.end(),
        [](const vision_msgs::msg::ObjectHypothesisWithPose &a, const vision_msgs::msg::ObjectHypothesisWithPose &b) {
          return a.hypothesis.score < b.hypothesis.score;
        });

    if (best_hyp == det.results.end()) {
      throw std::runtime_error("det.results must be non-empty");
    } else {
      label = inc_id;
      prob = best_hyp->hypothesis.score;
    }

    obj.rect = rect;
    obj.label = label;
    obj.prob = prob;
    objs.push_back(obj);

    ++inc_id;
  }

  return objs;
}


// Convert from bytetrack format back to ros msgs
tracking_2d_msgs::msg::Tracking2DArray tracking_2d::STracksToTracks(
  const std::vector<byte_track::BYTETracker::STrackPtr> &strk_ptrs,
  const std_msgs::msg::Header &header
) {
  // Use same header as detections for same time stamps
  tracking_2d_msgs::msg::Tracking2DArray trks;
  trks.header.frame_id = header.frame_id;
  trks.header.stamp = header.stamp;

  for (const auto &strk_ptr : strk_ptrs) {
    // Convert STrackPtr to Detection2D
    auto rect = strk_ptr->getRect();
    auto score = strk_ptr->getScore();
    auto trk_id = strk_ptr->getTrackId();

    tracking_2d_msgs::msg::Tracking2D trk;
    trk.header.frame_id = header.frame_id;
    trk.header.stamp = header.stamp;

    trk.id = trk_id;

    trk.cx = rect.x() + rect.width() / 2;
    trk.cy = rect.y() + rect.height() / 2;
    trk.width = rect.width();
    trk.height = rect.height();

    trk.score = score;

    trks.tracks.push_back(trk);
  }

  return trks;
}


void tracking_2d::detectionsCallback(
  vision_msgs::msg::Detection2DArray::SharedPtr msg
) {
  // Run bytetrack on detections
  auto objs = detsToObjects(msg);
  auto stracks = tracker_->update(objs);
  auto tracked_dets = STracksToTracks(stracks, msg->header);

  RCLCPP_DEBUG(this->get_logger(), "Publishing tracked detections...");
  tracked_dets_pub_->publish(tracked_dets);
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracking_2d>());
  rclcpp::shutdown();
  return 0;
}
