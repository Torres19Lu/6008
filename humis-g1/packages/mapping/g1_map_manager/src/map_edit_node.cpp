// map_edit_node.cpp -- offline, non-destructive map content editing. Tracks the
// active map from /map_manager/state, serves the MapEditCommand verb service backed
// by the ROS-free MapEditor, and picks obstacle voxels from RViz /clicked_point. On
// save it writes overlays/{obstacle_mask,keyframe_trim,metadata}.yaml via bounded
// history (overlays/.history); the geometry is never touched.

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>

#include <g1_msgs/MapEditCommand.h>
#include <g1_msgs/MapManagerState.h>

#include "g1_map_manager/core/map_editor.h"
#include "g1_map_manager/core/map_store.h"
#include "g1_map_manager/core/overlay.h"
#include "g1_map_manager/node_fs.h"

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

class MapEditNode {
 public:
  MapEditNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("maps_root", maps_root_, "");
    if (maps_root_.empty()) {
      const std::string share = ros::package::getPath("g1_maps");
      maps_root_ = share.empty() ? "maps" : share + "/maps";
    }
    pnh.param<std::string>("state_topic", state_topic_, "/map_manager/state");
    pnh.param<std::string>("clicked_point_topic", clicked_topic_, "/clicked_point");
    pnh.param("overlay_history_keep", history_keep_, 5);

    state_sub_ = nh.subscribe(state_topic_, 1, &MapEditNode::stateCb, this);
    clicked_sub_ = nh.subscribe(clicked_topic_, 10, &MapEditNode::clickedCb, this);
    cmd_srv_ =
        nh.advertiseService("map_edit/command", &MapEditNode::cmdSrv, this);
    ROS_INFO_STREAM("g1_map_edit: up. maps_root=" << maps_root_);
  }

 private:
  std::string overlaysDir(const std::string& name) const {
    return maps_root_ + "/" + name + "/overlays";
  }
  std::string obstaclePath(const std::string& n) const {
    return overlaysDir(n) + "/obstacle_mask.yaml";
  }
  std::string trimPath(const std::string& n) const {
    return overlaysDir(n) + "/keyframe_trim.yaml";
  }
  std::string metadataPath(const std::string& n) const {
    return overlaysDir(n) + "/metadata.yaml";
  }
  std::string historyDir(const std::string& n) const {
    return overlaysDir(n) + "/.history";
  }

  void loadOverlay(const std::string& name) {
    editor_.load(parseOverlay(readFile(obstaclePath(name)), readFile(trimPath(name)),
                              readFile(metadataPath(name))));
  }

  void stateCb(const g1_msgs::MapManagerState::ConstPtr& msg) {
    if (msg->active_map == active_map_) return;
    active_map_ = msg->active_map;
    if (!active_map_.empty()) loadOverlay(active_map_);
  }

  void clickedCb(const geometry_msgs::PointStamped::ConstPtr& pt) {
    if (active_map_.empty()) return;
    MapEditArgs a;
    a.points3 = {{pt->point.x, pt->point.y, pt->point.z}};
    std::string msg;
    editor_.apply("delete_obstacle", a, msg);  // stages; operator 'save's via command
    ROS_INFO_STREAM("g1_map_edit: staged obstacle delete at clicked point");
  }

  void writeOverlay(const std::string& name) {
    const MapFs fsi = makeMapFs();
    const std::string ts = std::to_string(ros::WallTime::now().toNSec());
    writeFileWithHistory(fsi, obstaclePath(name), historyDir(name), ts,
                         serializeObstacle(editor_.overlay()), history_keep_);
    writeFileWithHistory(fsi, trimPath(name), historyDir(name), ts,
                         serializeTrim(editor_.overlay()), history_keep_);
    writeFileWithHistory(fsi, metadataPath(name), historyDir(name), ts,
                         serializeMetadata(editor_.overlay()), history_keep_);
    editor_.markSaved();
  }

  bool cmdSrv(g1_msgs::MapEditCommand::Request& req,
              g1_msgs::MapEditCommand::Response& res) {
    const std::string target = req.map_name.empty() ? active_map_ : req.map_name;
    if (target.empty()) {
      res.success = false;
      res.message = "no active map to edit";
      return true;
    }
    MapEditArgs args;
    args.map_name = target;
    args.key = req.key;
    args.value = req.value;
    args.keyframe_id = req.keyframe_id;
    for (const auto& p : req.points) {
      args.points3.push_back({p.x, p.y, p.z});
      args.points2.push_back({p.x, p.y});
    }
    std::string msg;
    const bool ok = editor_.apply(req.command, args, msg);
    if (ok && req.command == "save" && editor_.dirty()) writeOverlay(target);
    res.success = ok;
    res.message = msg;
    return true;
  }

  MapEditor editor_;
  std::string maps_root_, state_topic_, clicked_topic_, active_map_;
  int history_keep_ = 5;
  ros::Subscriber state_sub_, clicked_sub_;
  ros::ServiceServer cmd_srv_;
};

}  // namespace g1_map_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_map_edit");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_map_manager::MapEditNode node(nh, pnh);
  ros::spin();
  return 0;
}
