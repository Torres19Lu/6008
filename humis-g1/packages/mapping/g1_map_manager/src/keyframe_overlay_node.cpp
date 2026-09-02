// keyframe_overlay_node.cpp -- apply the active map's edit overlay to the backend's
// /slam/keyframes and republish /slam/keyframes_filtered (latched) for the costmap.
// Reloads the overlay when the active map/version changes; an empty overlay passes
// the array through unchanged. Keeps the backend + the /slam/keyframes contract
// untouched (the overlay logic lives here, with the edits/ files).

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <g1_msgs/KeyframeCloudArray.h>
#include <g1_msgs/MapManagerState.h>

#include "g1_map_manager/core/overlay.h"
#include "g1_map_manager/keyframe_filter.h"

namespace g1_map_manager {
namespace fs = std::filesystem;
namespace {

std::string readFile(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is) return "";
  std::ostringstream ss;
  ss << is.rdbuf();
  return ss.str();
}

}  // namespace

class KeyframeOverlayNode {
 public:
  KeyframeOverlayNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("maps_root", maps_root_, "");
    if (maps_root_.empty()) {
      const std::string share = ros::package::getPath("g1_maps");
      maps_root_ = share.empty() ? "maps" : share + "/maps";
    }
    pnh.param("delete_radius", delete_radius_, 0.15);
    pnh.param<std::string>("state_topic", state_topic_, "/map_manager/state");
    pnh.param<std::string>("keyframes_topic", in_topic_, "/slam/keyframes");
    pnh.param<std::string>("filtered_topic", out_topic_, "/slam/keyframes_filtered");

    pub_ = nh.advertise<g1_msgs::KeyframeCloudArray>(out_topic_, 1, /*latch=*/true);
    kf_sub_ = nh.subscribe(in_topic_, 1, &KeyframeOverlayNode::kfCb, this);
    state_sub_ = nh.subscribe(state_topic_, 1, &KeyframeOverlayNode::stateCb, this);
    ROS_INFO_STREAM("g1_keyframe_overlay: " << in_topic_ << " -> " << out_topic_);
  }

 private:
  std::string overlaysDir(const std::string& n) const {
    return maps_root_ + "/" + n + "/overlays";
  }

  void reloadOverlay(const std::string& name) {
    if (name.empty()) {
      overlay_ = Overlay{};
      return;
    }
    overlay_ = parseOverlay(readFile(overlaysDir(name) + "/obstacle_mask.yaml"),
                            readFile(overlaysDir(name) + "/keyframe_trim.yaml"), "");
  }

  void republish() {
    if (last_kf_) pub_.publish(filterKeyframes(*last_kf_, overlay_, delete_radius_));
  }

  void stateCb(const g1_msgs::MapManagerState::ConstPtr& msg) {
    const std::string key = msg->active_map + "@" + msg->current_label;
    if (key == active_key_) return;
    active_key_ = key;
    reloadOverlay(msg->active_map);
    republish();  // re-resolve the cached keyframes against the new overlay
  }

  void kfCb(const g1_msgs::KeyframeCloudArray::ConstPtr& msg) {
    last_kf_ = msg;
    republish();
  }

  std::string maps_root_, state_topic_, in_topic_, out_topic_, active_key_;
  double delete_radius_ = 0.15;
  Overlay overlay_;
  g1_msgs::KeyframeCloudArray::ConstPtr last_kf_;
  ros::Publisher pub_;
  ros::Subscriber kf_sub_, state_sub_;
};

}  // namespace g1_map_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_keyframe_overlay");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_map_manager::KeyframeOverlayNode node(nh, pnh);
  ros::spin();
  return 0;
}
