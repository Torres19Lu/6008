// map_manager_node.cpp -- the thin ROS orchestrator for map management. Owns the
// map lifecycle (mapping / incremental / localization / stop+save), publishes the
// latched MapManagerState, and drives the long-lived g1_slam_backend in process via
// its services. All logic lives in the ROS-free core (MapStore, ModeMachine,
// NavPointStore, TopologyGraph); this file is ROS I/O only.
//
// Threading (mirror g1_nav): AsyncSpinner + one mutex mu_. NEVER call a blocking
// backend service .call() while holding mu_ -- gather state under the lock, release,
// do the ROS I/O, then re-acquire to commit the result.

#include <algorithm>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <g1_msgs/AddNavPoint.h>
#include <g1_msgs/NavigateToAction.h>
#include <g1_msgs/Gateway.h>
#include <g1_msgs/GatewayCapture.h>
#include <g1_msgs/RelocQuality.h>
#include <g1_msgs/GetActiveMap.h>
#include <g1_msgs/GetMapNeighbors.h>
#include <g1_msgs/GetTopology.h>
#include <g1_msgs/ListMaps.h>
#include <g1_msgs/LoadMap.h>
#include <g1_msgs/MapEntry.h>
#include <g1_msgs/MapManagerState.h>
#include <g1_msgs/NavPoint.h>
#include <g1_msgs/NavPointArray.h>
#include <g1_msgs/QueryNavPoint.h>
#include <g1_msgs/Relocalize.h>
#include <g1_msgs/SaveMap.h>
#include <g1_msgs/MapSnapshot.h>
#include <g1_msgs/SnapshotEntry.h>
#include <g1_msgs/StartIncremental.h>
#include <g1_msgs/StartLocalization.h>
#include <g1_msgs/StartMapping.h>
#include <g1_msgs/StopMapping.h>
#include <g1_msgs/KeyframePoseArray.h>

#include "g1_map_manager/core/gateway_capture.h"
#include "g1_map_manager/core/gateway_editor.h"
#include "g1_map_manager/core/map_types.h"
#include "g1_map_manager/core/mode_machine.h"
#include "g1_map_manager/core/map_lineage.h"
#include "g1_map_manager/core/map_store.h"
#include "g1_map_manager/core/nav_point_store.h"
#include "g1_map_manager/core/topology_graph.h"
#include "g1_map_manager/node_fs.h"

namespace g1_map_manager {
namespace fs = std::filesystem;
namespace {

std::vector<std::string> listSubdirs(const std::string& dir) {
  std::vector<std::string> out;
  std::error_code ec;
  if (!fs::is_directory(dir, ec)) return out;
  for (const auto& e : fs::directory_iterator(dir, ec)) {
    if (e.is_directory(ec)) out.push_back(e.path().filename().string());
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::string readFile(const std::string& path) {
  std::ifstream is(path, std::ios::binary);
  if (!is) return "";
  std::ostringstream ss;
  ss << is.rdbuf();
  return ss.str();
}

void writeFileAtomic(const std::string& path, const std::string& data) {
  const fs::path target(path);
  std::error_code ec;
  if (target.has_parent_path()) fs::create_directories(target.parent_path(), ec);
  const fs::path tmp = target.string() + ".tmp";
  {
    std::ofstream os(tmp, std::ios::binary | std::ios::trunc);
    os << data;
  }
  fs::rename(tmp, target, ec);
  if (ec) fs::copy_file(tmp, target, fs::copy_options::overwrite_existing, ec);
}

// Wall-clock stamps. isoNow() for lineage.created_at; snapStamp() sorts chronologically.
std::string isoNow() {
  const std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
  return buf;
}

std::string snapStamp() {
  const std::time_t t = std::time(nullptr);
  char buf[20];
  std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", std::gmtime(&t));
  return buf;
}

std::string slugify(const std::string& s) {
  std::string out;
  for (char c : s) out += (std::isalnum(static_cast<unsigned char>(c)) ? c : '-');
  return out.empty() ? "snapshot" : out;
}

uint8_t modeToMsg(Mode m) {
  switch (m) {
    case Mode::MAPPING: return g1_msgs::MapManagerState::MODE_MAPPING;
    case Mode::INCREMENTAL: return g1_msgs::MapManagerState::MODE_INCREMENTAL;
    case Mode::LOCALIZATION: return g1_msgs::MapManagerState::MODE_LOCALIZATION;
    case Mode::EDITING: return g1_msgs::MapManagerState::MODE_EDITING;
    case Mode::IDLE:
    default: return g1_msgs::MapManagerState::MODE_IDLE;
  }
}

Pose3 fromMsgPose(const geometry_msgs::Pose& p) {
  Pose3 o;
  o.x = p.position.x;
  o.y = p.position.y;
  o.z = p.position.z;
  o.qx = p.orientation.x;
  o.qy = p.orientation.y;
  o.qz = p.orientation.z;
  o.qw = p.orientation.w;
  return o;
}

geometry_msgs::Pose toMsgPose(const Pose3& p) {
  geometry_msgs::Pose o;
  o.position.x = p.x;
  o.position.y = p.y;
  o.position.z = p.z;
  o.orientation.x = p.qx;
  o.orientation.y = p.qy;
  o.orientation.z = p.qz;
  o.orientation.w = p.qw;
  return o;
}

g1_msgs::RelocQuality qualToMsg(const RelocQual& q) {
  g1_msgs::RelocQuality m;
  m.inlier_ratio = q.inlier_ratio;
  m.fitness = q.fitness;
  m.sc_distance = q.sc_distance;
  m.match_id = q.match_id;
  m.accepted = q.accepted;
  return m;
}

RelocQual qualFromMsg(const g1_msgs::RelocQuality& m) {
  RelocQual q;
  q.inlier_ratio = m.inlier_ratio;
  q.fitness = m.fitness;
  q.sc_distance = m.sc_distance;
  q.match_id = m.match_id;
  q.accepted = m.accepted;
  return q;
}

std::vector<KeyframePoseRec> toKfRecs(const g1_msgs::KeyframePoseArray::ConstPtr& a) {
  std::vector<KeyframePoseRec> out;
  if (!a) return out;
  out.reserve(a->poses.size());
  for (const auto& kp : a->poses) {
    KeyframePoseRec r;
    r.id = kp.id;
    r.pose = toIso(fromMsgPose(kp.pose));
    out.push_back(r);
  }
  return out;
}

}  // namespace

class MapManagerNode {
 public:
  MapManagerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("maps_root", maps_root_, "");
    if (maps_root_.empty()) {
      const std::string share = ros::package::getPath("g1_maps");
      maps_root_ = share.empty() ? "maps" : share + "/maps";
    }
    pnh.param("state_rate", state_rate_, 5.0);
    pnh.param("reloc_seed_timeout", reloc_seed_timeout_, 5.0);
    pnh.param("reloc_global_timeout", reloc_global_timeout_, 15.0);
    pnh.param<std::string>("state_topic", state_topic_, "/map_manager/state");
    pnh.param<std::string>("keyframe_poses_topic", kf_poses_topic_,
                           "/slam/keyframe_poses");
    pnh.param<std::string>("load_map_service", load_srv_name_, "/slam/load_map");
    pnh.param<std::string>("save_map_service", save_srv_name_, "/slam/save_map");
    pnh.param<std::string>("relocalize_service", reloc_srv_name_, "/slam/relocalize");
    pnh.param<std::string>("begin_incremental_service", begin_incr_srv_name_,
                           "/slam/begin_incremental");
    pnh.param<std::string>("reset_service", reset_srv_name_, "/slam/reset");
    pnh.param<std::string>("map_frame", map_frame_, "map");
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    pnh.param("nearest_anchor_max_dist", nearest_anchor_max_dist_, 5.0);
    pnh.param<std::string>("nav_points_topic", nav_points_topic_,
                           "/map_manager/nav_points");
    pnh.param<std::string>("nav_point_markers_topic", nav_point_markers_topic_,
                           "/map_manager/nav_point_markers");
    pnh.param<std::string>("gateway_markers_topic", gateway_markers_topic_,
                           "/map_manager/gateway_markers");

    pnh.param("overlay_history_keep", overlay_history_keep_, 5);
    pnh.param("topology_history_keep", topology_history_keep_, 5);
    pnh.param("snapshot_auto_keep", snapshot_auto_keep_, 2);

    map_store_ = std::make_unique<MapStore>(maps_root_, makeMapFs());

    load_client_ = nh.serviceClient<g1_msgs::LoadMap>(load_srv_name_);
    save_client_ = nh.serviceClient<g1_msgs::SaveMap>(save_srv_name_);
    reloc_client_ = nh.serviceClient<g1_msgs::Relocalize>(reloc_srv_name_);
    begin_incr_client_ = nh.serviceClient<std_srvs::Trigger>(begin_incr_srv_name_);
    reset_client_ = nh.serviceClient<std_srvs::Trigger>(reset_srv_name_);

    kf_poses_sub_ =
        nh.subscribe(kf_poses_topic_, 1, &MapManagerNode::kfPosesCb, this);

    state_pub_ = nh.advertise<g1_msgs::MapManagerState>(state_topic_, 1, /*latch=*/true);
    nav_points_pub_ =
        nh.advertise<g1_msgs::NavPointArray>(nav_points_topic_, 1, /*latch=*/true);
    nav_point_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        nav_point_markers_topic_, 1, /*latch=*/true);
    gateway_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        gateway_markers_topic_, 1, /*latch=*/true);

    start_mapping_srv_ = nh.advertiseService(
        "map_manager/start_mapping", &MapManagerNode::startMappingSrv, this);
    start_incremental_srv_ = nh.advertiseService(
        "map_manager/start_incremental", &MapManagerNode::startIncrementalSrv, this);
    start_localization_srv_ = nh.advertiseService(
        "map_manager/start_localization", &MapManagerNode::startLocalizationSrv, this);
    stop_mapping_srv_ = nh.advertiseService(
        "map_manager/stop_mapping", &MapManagerNode::stopMappingSrv, this);
    list_maps_srv_ = nh.advertiseService(
        "map_manager/list_maps", &MapManagerNode::listMapsSrv, this);
    get_active_srv_ = nh.advertiseService(
        "map_manager/get_active_map", &MapManagerNode::getActiveMapSrv, this);
    snapshot_srv_ = nh.advertiseService(
        "map_manager/snapshot", &MapManagerNode::snapshotSrv, this);
    add_nav_srv_ = nh.advertiseService(
        "map_manager/add_nav_point", &MapManagerNode::addNavPointSrv, this);
    query_nav_srv_ = nh.advertiseService(
        "map_manager/query_nav_point", &MapManagerNode::queryNavPointSrv, this);
    reload_topo_srv_ = nh.advertiseService(
        "map_manager/reload_topology", &MapManagerNode::reloadTopologySrv, this);
    get_topology_srv_ = nh.advertiseService(
        "map_manager/get_topology", &MapManagerNode::getTopologySrv, this);
    get_neighbors_srv_ = nh.advertiseService(
        "map_manager/get_map_neighbors", &MapManagerNode::getMapNeighborsSrv, this);
    pnh.param<std::string>("gateway_capture_service", gateway_capture_service_,
                           "map_manager/gateway_capture");
    gateway_capture_srv_ = nh.advertiseService(
        gateway_capture_service_, &MapManagerNode::gatewayCaptureSrv, this);

    pnh.param<std::string>("map_nav_action", map_nav_action_name_,
                           "/map_nav/navigate_to");
    pnh.param<std::string>("nav_action", nav_action_name_, "/navigate_to");
    pnh.param("transition_wait", transition_wait_, 3.0);
    pnh.param<std::string>("topology_path", topology_path_, "");
    if (topology_path_.empty()) {
      topology_path_ =
          (fs::path(maps_root_).parent_path() / "topology" / "topology.yaml").string();
    }
    loadTopology();

    ac_ = std::make_unique<actionlib::SimpleActionClient<g1_msgs::NavigateToAction>>(
        nav_action_name_, true);
    as_ = std::make_unique<actionlib::SimpleActionServer<g1_msgs::NavigateToAction>>(
        nh, map_nav_action_name_, boost::bind(&MapManagerNode::executeNav, this, _1),
        false);
    as_->start();

    state_timer_ = nh.createTimer(ros::Duration(1.0 / state_rate_),
                                  &MapManagerNode::stateTimerCb, this);

    {
      std::lock_guard<std::mutex> lk(mu_);
      mm_.setMode(Mode::IDLE);
      mm_.setGlobalTimeout(reloc_global_timeout_);
      publishStateLocked("idle");
    }
    ROS_INFO_STREAM("g1_map_manager: up. maps_root=" << maps_root_);
  }

 protected:
  // ---- backend orchestration (call OFF the lock) ----
  bool callReset(std::string* msg) {
    std_srvs::Trigger s;
    if (!reset_client_.call(s)) {
      if (msg) *msg = "backend /slam/reset unreachable";
      return false;
    }
    if (msg) *msg = s.response.message;
    return s.response.success;
  }
  bool callLoadMap(const std::string& path, bool read_only, std::string* msg) {
    g1_msgs::LoadMap s;
    s.request.path = path;
    s.request.read_only = read_only;
    if (!load_client_.call(s)) {
      if (msg) *msg = "backend /slam/load_map unreachable";
      return false;
    }
    if (msg) *msg = s.response.message;
    return s.response.success;
  }
  bool callBeginIncremental(std::string* msg) {
    std_srvs::Trigger s;
    if (!begin_incr_client_.call(s)) {
      if (msg) *msg = "backend /slam/begin_incremental unreachable";
      return false;
    }
    if (msg) *msg = s.response.message;
    return s.response.success;
  }
  bool callSaveMap(const std::string& path, std::string* msg) {
    g1_msgs::SaveMap s;
    s.request.path = path;
    if (!save_client_.call(s)) {
      if (msg) *msg = "backend /slam/save_map unreachable";
      return false;
    }
    if (msg) *msg = s.response.message;
    return s.response.success;
  }
  // Relocalize, retrying over `timeout` while the backend has no live scan yet.
  bool callRelocalizeUntil(bool use_guess, double timeout, std::string* msg) {
    g1_msgs::Relocalize s;
    s.request.use_guess = use_guess;
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout);
    do {
      if (reloc_client_.call(s) && s.response.success) {
        if (msg) *msg = s.response.message;
        return true;
      }
      ros::Duration(0.2).sleep();
    } while (ros::ok() && ros::Time::now() < deadline);
    if (msg) *msg = s.response.message.empty() ? "relocalization timed out"
                                               : s.response.message;
    return false;
  }

  // ---- lifecycle services ----
  bool startMappingSrv(g1_msgs::StartMapping::Request& req,
                       g1_msgs::StartMapping::Response& res) {
    std::string msg;
    const bool ok = callReset(&msg);
    std::lock_guard<std::mutex> lk(mu_);
    if (ok) {
      mm_.setMode(Mode::MAPPING);
      active_map_ = req.name;
      current_label_.clear();
      localized_ = false;
      loadActiveNavPointsLocked();
      republishNavPointsLocked();
      publishStateLocked("mapping '" + req.name + "'");
    }
    res.success = ok;
    res.message = ok ? "mapping started" : ("reset failed: " + msg);
    return true;
  }

  bool startLocalizationSrv(g1_msgs::StartLocalization::Request& req,
                            g1_msgs::StartLocalization::Response& res) {
    const std::string path = map_store_->geometryDir(req.name);
    std::string msg;
    const bool ok = callLoadMap(path, /*read_only=*/true, &msg);
    std::lock_guard<std::mutex> lk(mu_);
    if (ok) {
      mm_.setMode(Mode::LOCALIZATION);
      active_map_ = req.name;
      current_label_ = map_store_->readLineage(req.name).label;
      localized_ = false;
      loadActiveNavPointsLocked();
      republishNavPointsLocked();
      publishStateLocked("localization '" + req.name + "' (read-only)");
    }
    res.success = ok;
    res.message = ok ? "localization map loaded (read-only)" : ("load failed: " + msg);
    return true;
  }

  bool startIncrementalSrv(g1_msgs::StartIncremental::Request& req,
                           g1_msgs::StartIncremental::Response& res) {
    const std::string path = map_store_->geometryDir(req.name);
    std::string msg;
    if (!callLoadMap(path, /*read_only=*/false, &msg)) {
      res.success = false;
      res.message = "load failed: " + msg;
      return true;
    }
    if (!callRelocalizeUntil(/*use_guess=*/false, reloc_global_timeout_, &msg)) {
      res.success = false;
      res.message = "relocalize failed: " + msg;
      return true;
    }
    // Auto rollback point BEFORE we mutate: snapshot geometry/ + overlays/, then prune.
    const std::string snap_id = snapStamp() + "-pre-incremental";
    map_store_->createSnapshot(req.name, snap_id);
    map_store_->pruneSnapshots(req.name, "-pre-incremental", snapshot_auto_keep_);
    if (!callBeginIncremental(&msg)) {
      res.success = false;
      res.message = "begin_incremental failed: " + msg;
      return true;
    }
    std::lock_guard<std::mutex> lk(mu_);
    mm_.setMode(Mode::INCREMENTAL);
    active_map_ = req.name;
    current_label_ = map_store_->readLineage(req.name).label;
    pre_incr_snapshot_ = snap_id;  // becomes the next save's lineage.parent
    localized_ = true;
    loadActiveNavPointsLocked();
    republishNavPointsLocked();
    publishStateLocked("incremental '" + req.name + "'");
    res.success = true;
    res.message = "incremental mapping resumed";
    return true;
  }

  bool stopMappingSrv(g1_msgs::StopMapping::Request& req,
                      g1_msgs::StopMapping::Response& res) {
    std::string name;
    bool incremental = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!mm_.canWriteMap() ||
          (mm_.mode() != Mode::MAPPING && mm_.mode() != Mode::INCREMENTAL)) {
        res.success = false;
        res.message = "not in a mapping session";
        return true;
      }
      incremental = (mm_.mode() == Mode::INCREMENTAL);
      name = req.new_name.empty() ? active_map_ : req.new_name;
    }
    if (req.save) {
      if (name.empty()) {
        res.success = false;
        res.message = "no map name to save under";
        return true;
      }
      std::string msg;
      if (!callSaveMap(map_store_->geometryTmpDir(name), &msg)) {  // backend -> geometry.tmp/
        res.success = false;
        res.message = "save failed: " + msg;
        return true;
      }
      MapLineage lin;
      lin.name = name;
      lin.created_at = isoNow();
      lin.label = req.label.empty() ? (incremental ? "increment" : "initial") : req.label;
      lin.reason = incremental ? "incremental" : "initial";
      lin.parent = incremental ? pre_incr_snapshot_ : "";
      writeFileAtomic(map_store_->geometryTmpDir(name) + "/lineage.yaml",
                      serializeLineage(lin));
      map_store_->commitGeometry(name);  // atomic swap geometry.tmp -> geometry
      std::lock_guard<std::mutex> lk(mu_);
      mm_.setMode(Mode::IDLE);
      active_map_ = name;
      current_label_ = lin.label;
      pre_incr_snapshot_.clear();
      localized_ = false;
      publishStateLocked("saved '" + name + "' (" + lin.label + ")");
      res.success = true;
      res.message = "saved";
      res.saved_label = lin.label;
      return true;
    }
    std::lock_guard<std::mutex> lk(mu_);
    mm_.setMode(Mode::IDLE);
    localized_ = false;
    publishStateLocked("mapping stopped (not saved)");
    res.success = true;
    res.message = "stopped without saving";
    return true;
  }

  bool listMapsSrv(g1_msgs::ListMaps::Request&, g1_msgs::ListMaps::Response& res) {
    for (const std::string& name : map_store_->listMaps()) {
      g1_msgs::MapEntry e;
      e.name = name;
      const MapLineage lin = map_store_->readLineage(name);
      e.label = lin.label;
      e.created_at = lin.created_at;
      e.num_keyframes = parseBackendNumKeyframes(
          readFile(map_store_->geometryDir(name) + "/manifest.yaml"));
      e.num_snapshots =
          static_cast<uint32_t>(map_store_->listSnapshots(name).size());
      res.maps.push_back(e);
    }
    return true;
  }

  bool getActiveMapSrv(g1_msgs::GetActiveMap::Request&,
                       g1_msgs::GetActiveMap::Response& res) {
    std::lock_guard<std::mutex> lk(mu_);
    res.state = buildStateLocked(detail_);
    return true;
  }

  bool snapshotSrv(g1_msgs::MapSnapshot::Request& req,
                   g1_msgs::MapSnapshot::Response& res) {
    std::string name;
    {
      std::lock_guard<std::mutex> lk(mu_);
      name = req.name.empty() ? active_map_ : req.name;
    }
    if (name.empty()) {
      res.success = false;
      res.message = "no map";
      return true;
    }
    if (req.op == "list") {
      for (const std::string& id : map_store_->listSnapshots(name))
        res.snapshots.push_back(snapshotEntry(name, id));
      res.success = true;
      res.message = "ok";
      return true;
    }
    if (req.op == "create") {
      const std::string reason = req.reason.empty() ? "milestone" : req.reason;
      const std::string id =
          snapStamp() + "-" + slugify(req.label.empty() ? reason : req.label);
      map_store_->createSnapshot(name, id);
      res.snapshots.push_back(snapshotEntry(name, id));
      res.success = true;
      res.message = "created " + id;
      return true;
    }
    if (req.op == "restore") {
      if (req.snapshot_id.empty()) {
        res.success = false;
        res.message = "snapshot_id required";
        return true;
      }
      map_store_->restoreSnapshot(name, req.snapshot_id);
      std::lock_guard<std::mutex> lk(mu_);
      if (name == active_map_) {
        loadActiveNavPointsLocked();
        republishNavPointsLocked();
        current_label_ = map_store_->readLineage(name).label;
      }
      publishStateLocked("restored '" + name + "' <- " + req.snapshot_id);
      res.success = true;
      res.message = "restored " + req.snapshot_id;
      return true;
    }
    if (req.op == "delete") {
      if (req.snapshot_id.empty()) {
        res.success = false;
        res.message = "snapshot_id required";
        return true;
      }
      map_store_->removeSnapshot(name, req.snapshot_id);
      res.success = true;
      res.message = "deleted " + req.snapshot_id;
      return true;
    }
    res.success = false;
    res.message = "unknown op: " + req.op;
    return true;
  }

  g1_msgs::SnapshotEntry snapshotEntry(const std::string& name,
                                       const std::string& id) const {
    g1_msgs::SnapshotEntry e;
    e.id = id;
    const MapLineage lin = parseLineage(
        readFile(map_store_->snapshotDir(name, id) + "/geometry/lineage.yaml"));
    e.label = lin.label;
    e.reason = lin.reason;
    e.parent = lin.parent;
    e.created_at = lin.created_at;
    e.num_keyframes = parseBackendNumKeyframes(
        readFile(map_store_->snapshotDir(name, id) + "/geometry/manifest.yaml"));
    return e;
  }

  void kfPosesCb(const g1_msgs::KeyframePoseArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    kf_poses_ = msg;  // cached for nav-point anchoring
    republishNavPointsLocked();  // re-resolve world poses against the new keyframe poses
  }

  // ---- map-aware navigation (action server + g1_nav delegation) ----
  void loadTopology() {
    std::vector<std::string> maps;
    std::vector<GatewayEdge> edges;
    parseTopology(readFile(topology_path_), maps, edges);
    std::lock_guard<std::mutex> lk(mu_);
    topo_.load(maps, edges);
    publishGatewayMarkersLocked();
  }

  std::string topologyHistoryDir() const {
    return (fs::path(topology_path_).parent_path() / ".history").string();
  }
  // Back up the current topology.yaml into topology/.history (bounded), atomically write
  // `serialized`, then refresh topo_ + markers via loadTopology(). Call OFF the lock
  // (file I/O); the caller passes the serialized text gathered under the lock.
  void backupAndWriteTopology(const std::string& serialized) {
    writeFileWithHistory(makeMapFs(), topology_path_, topologyHistoryDir(), snapStamp(),
                         serialized, topology_history_keep_);
    loadTopology();  // refresh topo_ + republish gateway markers
  }

  bool reloadTopologySrv(std_srvs::Trigger::Request&,
                         std_srvs::Trigger::Response& res) {
    loadTopology();
    res.success = true;
    res.message = "topology reloaded";
    return true;
  }

  static g1_msgs::Gateway gatewayToMsg(const GatewayEdge& e) {
    g1_msgs::Gateway g;
    g.from_map = e.from;
    g.to_map = e.to;
    g.pose_in_from = toMsgPose(fromIso(e.pose_in_from));
    g.pose_in_to = toMsgPose(fromIso(e.pose_in_to));
    g.label = e.label;
    g.source = e.source.empty() ? "captured" : e.source;
    return g;
  }

  bool getTopologySrv(g1_msgs::GetTopology::Request&,
                      g1_msgs::GetTopology::Response& res) {
    std::lock_guard<std::mutex> lk(mu_);
    res.maps = topo_.maps();
    for (const GatewayEdge& e : topo_.edges()) res.edges.push_back(gatewayToMsg(e));
    return true;
  }

  bool getMapNeighborsSrv(g1_msgs::GetMapNeighbors::Request& req,
                          g1_msgs::GetMapNeighbors::Response& res) {
    std::lock_guard<std::mutex> lk(mu_);
    const std::string target = req.map_name.empty() ? active_map_ : req.map_name;
    for (const GatewayEdge& e : topo_.neighbors(target)) {
      res.neighbors.push_back(gatewayToMsg(e));
    }
    res.success = true;
    res.message = "neighbors derived from topology.yaml";
    return true;
  }

  // ---- gateway capture (begin/load/commit/nudge/yaw/save/discard/status) ----
  bool navBusyLocked() const { return as_ && as_->isActive(); }  // mu_ held

  // Build the response state string + staged edge + qualities (mu_ held).
  void fillCaptureResponseLocked(g1_msgs::GatewayCapture::Response& res) {
    const bool has_staged = gw_editor_.view().size() > gw_editor_.edges().size();
    if (has_staged) {
      res.state = "staged";
    } else {
      switch (capture_.state()) {
        case CaptureState::BEGUN: res.state = "begun"; break;
        case CaptureState::LOADED: res.state = "loaded"; break;
        default: res.state = "idle"; break;
      }
    }
    if (has_staged) res.staged = gatewayToMsg(gw_editor_.view().back());
    res.quality_from = qualToMsg(capture_.qualityFrom());
  }

  bool gatewayCaptureSrv(g1_msgs::GatewayCapture::Request& req,
                         g1_msgs::GatewayCapture::Response& res) {
    const std::string& cmd = req.command;

    if (cmd == "begin") {
      std::string from_map;
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (navBusyLocked()) {
          res.success = false; res.message = "navigation active";
          fillCaptureResponseLocked(res); return true;
        }
        if (mm_.mode() != Mode::LOCALIZATION) {
          res.success = false; res.message = "begin requires LOCALIZATION mode";
          fillCaptureResponseLocked(res); return true;
        }
        if (capture_.state() != CaptureState::IDLE ||
            gw_editor_.view().size() > gw_editor_.edges().size()) {
          res.success = false;
          res.message = "a capture or staged gateway is pending; save or discard first";
          fillCaptureResponseLocked(res); return true;
        }
        from_map = active_map_;
      }
      // The from-side pose comes from the live map->base_link TF; a TF failure means
      // the backend has no lock yet, which is the real "not localized" gate.
      Eigen::Isometry3d pose_from;
      std::string err;
      if (!captureCurrentPose(pose_from, err)) {
        res.success = false; res.message = err;
        std::lock_guard<std::mutex> lk(mu_); fillCaptureResponseLocked(res); return true;
      }
      RelocQual qf; qf.accepted = true;  // ongoing tracking trusted; best-effort quality
      std::lock_guard<std::mutex> lk(mu_);
      // (re)sync the editor's committed edges from the current topology before staging.
      gw_editor_.load(topo_.maps(), topo_.edges());
      res.success = capture_.begin(from_map, req.to_map, req.label, pose_from, qf,
                                   res.message);
      fillCaptureResponseLocked(res);
      return true;
    }

    if (cmd == "load") {
      std::string to_map;
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (capture_.state() != CaptureState::BEGUN) {
          res.success = false; res.message = "load requires a pending begin";
          fillCaptureResponseLocked(res); return true;
        }
        if (mm_.mode() != Mode::LOCALIZATION) {
          res.success = false; res.message = "load requires LOCALIZATION mode";
          fillCaptureResponseLocked(res); return true;
        }
        to_map = capture_.toMap();
      }
      std::string msg;
      if (!callLoadMap(map_store_->geometryDir(to_map), /*read_only=*/true, &msg)) {
        res.success = false; res.message = "load failed: " + msg;
        std::lock_guard<std::mutex> lk(mu_); fillCaptureResponseLocked(res); return true;
      }
      std::lock_guard<std::mutex> lk(mu_);
      active_map_ = to_map;
      current_label_ = map_store_->readLineage(to_map).label;
      localized_ = false;
      loadActiveNavPointsLocked();
      republishNavPointsLocked();
      res.success = capture_.load(res.message);
      publishStateLocked("gateway capture: loaded '" + to_map + "'");
      fillCaptureResponseLocked(res);
      return true;
    }

    if (cmd == "commit") {
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (capture_.state() != CaptureState::LOADED) {
          res.success = false; res.message = "commit requires a loaded target";
          fillCaptureResponseLocked(res); return true;
        }
      }
      Eigen::Isometry3d locked;
      RelocQual qt;
      std::string msg;
      const bool ok =
          callRelocalizeGlobalWithQuality(reloc_global_timeout_, locked, qt, msg);
      qt.accepted = ok;  // backend success == passed the inlier gate
      std::lock_guard<std::mutex> lk(mu_);
      res.quality_to = qualToMsg(qt);
      if (!ok) {
        res.success = false; res.message = "relocalize failed: " + msg;
        fillCaptureResponseLocked(res); return true;
      }
      GatewayEdge edge;
      if (!capture_.commit(locked, qt, edge, res.message)) {
        res.success = false; fillCaptureResponseLocked(res); return true;
      }
      // hand the committed edge to the editor for review/nudge/save.
      GatewayEditArgs a;
      a.from_map = edge.from; a.to_map = edge.to; a.label = edge.label;
      a.pose_in_from = edge.pose_in_from; a.pose_in_to = edge.pose_in_to;
      gw_editor_.apply("define", a, msg);
      publishGatewayMarkersLocked();
      res.success = true; res.message = "committed; review then save";
      fillCaptureResponseLocked(res);
      return true;
    }

    if (cmd == "nudge" || cmd == "yaw") {
      std::lock_guard<std::mutex> lk(mu_);
      GatewayEditArgs a;
      a.values.assign(req.values.begin(), req.values.end());
      res.success = gw_editor_.apply(cmd, a, res.message);
      publishGatewayMarkersLocked();
      fillCaptureResponseLocked(res);
      return true;
    }

    if (cmd == "save") {
      std::string serialized;
      {
        std::lock_guard<std::mutex> lk(mu_);
        std::string msg;
        if (!gw_editor_.apply("save", {}, msg) || !gw_editor_.dirty()) {
          res.success = false; res.message = msg.empty() ? "nothing to save" : msg;
          fillCaptureResponseLocked(res); return true;
        }
        serialized = gw_editor_.serialize();
        gw_editor_.clearDirty();
      }
      backupAndWriteTopology(serialized);  // file I/O + loadTopology off the lock
      std::lock_guard<std::mutex> lk(mu_);
      res.success = true; res.message = "saved gateway to topology.yaml";
      fillCaptureResponseLocked(res);
      return true;
    }

    if (cmd == "discard") {
      std::lock_guard<std::mutex> lk(mu_);
      if (capture_.state() != CaptureState::IDLE) {
        res.success = capture_.discard(res.message);
      } else {
        res.success = gw_editor_.apply("discard", {}, res.message);
      }
      publishGatewayMarkersLocked();
      fillCaptureResponseLocked(res);
      return true;
    }

    if (cmd == "status") {
      std::lock_guard<std::mutex> lk(mu_);
      res.success = true;
      res.message = "capture status";
      fillCaptureResponseLocked(res);
      return true;
    }

    res.success = false;
    res.message = "unknown command: " + cmd;
    std::lock_guard<std::mutex> lk(mu_);
    fillCaptureResponseLocked(res);
    return true;
  }

  // Gateway departure poses (current map frame) as arrows + labels for RViz.
  void publishGatewayMarkersLocked() {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int id = 0;
    for (const GatewayEdge& e : topo_.neighbors(active_map_)) {
      // Departure pose is in this (active) map frame: from_map if we leave via
      // from, else to_map's pose_in_to.
      const Eigen::Isometry3d depart =
          (e.from == active_map_) ? e.pose_in_from : e.pose_in_to;
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = map_frame_;
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "gateways";
      arrow.id = id++;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose = toMsgPose(fromIso(depart));
      arrow.scale.x = 0.6;
      arrow.scale.y = 0.1;
      arrow.scale.z = 0.1;
      arrow.color.b = 1.0f;
      arrow.color.a = 0.9f;
      markers.markers.push_back(arrow);
      visualization_msgs::Marker text = arrow;
      text.id = id++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.pose.position.z += 0.4;
      text.scale.z = 0.25;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.text = (e.from == active_map_ ? e.to : e.from) +
                  (e.label.empty() ? "" : " (" + e.label + ")");
      markers.markers.push_back(text);
    }
    if (gw_editor_.view().size() > gw_editor_.edges().size()) {
      const GatewayEdge& st = gw_editor_.view().back();   // the staged edge
      const Eigen::Isometry3d depart =
          (st.from == active_map_) ? st.pose_in_from : st.pose_in_to;
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = map_frame_;
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "gateways_staged";
      arrow.id = id++;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose = toMsgPose(fromIso(depart));
      arrow.scale.x = 0.6; arrow.scale.y = 0.1; arrow.scale.z = 0.1;
      arrow.color.r = 1.0f; arrow.color.g = 1.0f; arrow.color.a = 0.9f;  // yellow
      markers.markers.push_back(arrow);
    }
    gateway_markers_pub_.publish(markers);
  }

  double nowSec() const { return ros::Time::now().toSec(); }

  void publishTransition(bool t) {
    std::lock_guard<std::mutex> lk(mu_);
    if (transition_ != t) {
      transition_ = t;
      publishStateLocked(detail_);
    }
  }

  void legFeedbackCb(const g1_msgs::NavigateToFeedbackConstPtr& fb) {
    g1_msgs::NavigateToFeedback relayed = *fb;  // pass g1_nav progress straight through
    as_->publishFeedback(relayed);
  }

  // Send one GOAL_POSE leg to g1_nav (map frame) and wait, relaying feedback and
  // honoring a manager-level preempt. Returns true on g1_nav SUCCEEDED.
  bool sendLegAndWait(const geometry_msgs::Pose& pose, uint8_t* outcome) {
    *outcome = g1_msgs::NavigateToResult::OUTCOME_ABORTED_NO_PATH;
    if (!ac_->waitForServer(ros::Duration(5.0))) return false;
    g1_msgs::NavigateToGoal g;
    g.goal_type = g1_msgs::NavigateToGoal::GOAL_POSE;
    g.target_pose.header.frame_id = map_frame_;
    g.target_pose.header.stamp = ros::Time::now();
    g.target_pose.pose = pose;
    ac_->sendGoal(
        g, actionlib::SimpleActionClient<g1_msgs::NavigateToAction>::SimpleDoneCallback(),
        actionlib::SimpleActionClient<g1_msgs::NavigateToAction>::SimpleActiveCallback(),
        boost::bind(&MapManagerNode::legFeedbackCb, this, _1));
    ros::Rate r(20.0);
    while (ros::ok()) {
      if (as_->isPreemptRequested()) {
        ac_->cancelGoal();
        *outcome = g1_msgs::NavigateToResult::OUTCOME_PREEMPTED;
        return false;
      }
      if (ac_->getState().isDone()) break;
      r.sleep();
    }
    const auto result = ac_->getResult();
    if (result) *outcome = result->outcome;
    return ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  }

  bool callRelocalizeSeedUntil(const Eigen::Isometry3d& seed, double timeout) {
    g1_msgs::Relocalize s;
    s.request.use_guess = true;
    s.request.initial_guess = toMsgPose(fromIso(seed));
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout);
    do {
      if (reloc_client_.call(s) && s.response.success) {
        std::lock_guard<std::mutex> lk(mu_);
        localized_ = true;
        return true;
      }
      ros::Duration(0.2).sleep();
    } while (ros::ok() && ros::Time::now() < deadline);
    return false;
  }

  bool callRelocalizeOnce(bool use_guess) {
    g1_msgs::Relocalize s;
    s.request.use_guess = use_guess;
    if (reloc_client_.call(s) && s.response.success) {
      std::lock_guard<std::mutex> lk(mu_);
      localized_ = true;
      return true;
    }
    return false;
  }

  // Look up the current map->base_link pose (off the lock). Returns false on TF failure.
  bool captureCurrentPose(Eigen::Isometry3d& out, std::string& err) {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                      ros::Duration(0.5));
    } catch (const tf2::TransformException& e) {
      err = std::string("TF ") + map_frame_ + "->" + base_frame_ + " failed: " + e.what();
      return false;
    }
    geometry_msgs::Pose p;
    p.position.x = tf.transform.translation.x;
    p.position.y = tf.transform.translation.y;
    p.position.z = tf.transform.translation.z;
    p.orientation = tf.transform.rotation;
    out = toIso(fromMsgPose(p));
    return true;
  }

  // Global relocalize (no guess), retrying over `timeout`. On success sets localized_,
  // fills out_locked (map-frame robot pose) and out_q. Call OFF the lock.
  bool callRelocalizeGlobalWithQuality(double timeout, Eigen::Isometry3d& out_locked,
                                       RelocQual& out_q, std::string& msg) {
    g1_msgs::Relocalize s;
    s.request.use_guess = false;
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout);
    do {
      if (reloc_client_.call(s)) {
        out_q = qualFromMsg(s.response.quality);
        if (s.response.success) {
          out_locked = toIso(fromMsgPose(s.response.locked_pose));
          {
            std::lock_guard<std::mutex> lk(mu_);
            localized_ = true;
          }
          msg = "relocalized";
          return true;
        }
      }
      ros::Duration(0.2).sleep();
    } while (ros::ok() && ros::Time::now() < deadline);
    msg = s.response.message.empty() ? "relocalization timed out" : s.response.message;
    return false;
  }

  // Load next_map read-only, switch the manager's active-map context, then seed
  // relocalize. Returns true on a lock.
  bool switchToMap(const std::string& next_map, const Eigen::Isometry3d& seed) {
    std::string msg;
    if (!callLoadMap(map_store_->geometryDir(next_map), /*read_only=*/true, &msg)) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      active_map_ = next_map;
      current_label_ = map_store_->readLineage(next_map).label;
      localized_ = false;
      loadActiveNavPointsLocked();
      republishNavPointsLocked();
    }
    return callRelocalizeSeedUntil(seed, reloc_seed_timeout_);
  }

  // Resolve the final goal pose (target map frame). GOAL_NAV_POINT resolves against
  // the now-active target map's nav store + live keyframe poses.
  bool resolveFinalGoalPose(const g1_msgs::NavigateToGoalConstPtr& goal,
                            const std::string& target_map, geometry_msgs::Pose& out,
                            std::string& err) {
    if (goal->goal_type == g1_msgs::NavigateToGoal::GOAL_POSE) {
      out = goal->target_pose.pose;
      return true;
    }
    if (goal->goal_type == g1_msgs::NavigateToGoal::GOAL_NAV_POINT) {
      std::lock_guard<std::mutex> lk(mu_);
      if (target_map != active_map_) {
        err = "nav-point goal target map is not active";
        return false;
      }
      Pose3 world;
      if (!nav_store_.resolve(goal->nav_point_name, toKfRecs(kf_poses_), world)) {
        err = "nav point '" + goal->nav_point_name + "' not found in '" + target_map + "'";
        return false;
      }
      out = toMsgPose(world);
      return true;
    }
    err = "unsupported goal_type";
    return false;
  }

  void executeNav(const g1_msgs::NavigateToGoalConstPtr& goal) {
    g1_msgs::NavigateToResult result;
    std::string from_map, target_map;
    {
      std::lock_guard<std::mutex> lk(mu_);
      from_map = active_map_;
      target_map = goal->map_name.empty() ? active_map_ : goal->map_name;
    }
    if (from_map.empty()) {
      result.success = false;
      result.outcome = g1_msgs::NavigateToResult::OUTCOME_REJECTED;
      result.message = "no active map";
      as_->setAborted(result, result.message);
      return;
    }
    if (goal->goal_type == g1_msgs::NavigateToGoal::GOAL_OBJECT_ID) {
      result.success = false;
      result.outcome = g1_msgs::NavigateToResult::OUTCOME_REJECTED;
      result.message = "object-id goals are not supported";
      as_->setAborted(result, result.message);
      return;
    }

    std::vector<RouteLeg> route;
    {
      std::lock_guard<std::mutex> lk(mu_);
      route = topo_.route(from_map, target_map);
    }
    if (route.empty()) {
      result.success = false;
      result.outcome = g1_msgs::NavigateToResult::OUTCOME_REJECTED;
      result.message = "no route from '" + from_map + "' to '" + target_map + "'";
      as_->setAborted(result, result.message);
      return;
    }

    const double dwell =
        goal->transition_wait >= 0.0f ? goal->transition_wait : transition_wait_;
    mm_.setGlobalTimeout(reloc_global_timeout_);
    mm_.beginRoute(route, dwell, nowSec());

    uint8_t leg_fail_outcome = g1_msgs::NavigateToResult::OUTCOME_ABORTED_NO_PATH;
    ManagerObs obs;
    obs.now = nowSec();
    while (ros::ok()) {
      if (as_->isPreemptRequested()) {
        ac_->cancelGoal();
        publishTransition(false);
        result.success = false;
        result.outcome = g1_msgs::NavigateToResult::OUTCOME_PREEMPTED;
        result.message = "preempted";
        as_->setPreempted(result, result.message);
        return;
      }
      const XmDecision d = mm_.step(obs);
      publishTransition(d.transition);
      obs = ManagerObs{};
      obs.now = nowSec();
      switch (d.action) {
        case XmAction::SEND_LEG: {
          const RouteLeg& leg = route[d.leg_index];
          geometry_msgs::Pose pose;
          if (leg.is_final) {
            if (!resolveFinalGoalPose(goal, target_map, pose, result.message)) {
              publishTransition(false);
              result.success = false;
              result.outcome = g1_msgs::NavigateToResult::OUTCOME_REJECTED;
              as_->setAborted(result, result.message);
              return;
            }
          } else {
            pose = toMsgPose(fromIso(leg.depart_pose));
          }
          uint8_t outcome = 0;
          const bool ok = sendLegAndWait(pose, &outcome);
          if (!ok) leg_fail_outcome = outcome;
          obs.leg_succeeded = ok;
          obs.leg_aborted = !ok;
          obs.now = nowSec();
          break;
        }
        case XmAction::DWELL_GATEWAY:
          ros::Duration(0.1).sleep();
          obs.now = nowSec();
          break;
        case XmAction::SWITCH_MAP: {
          const RouteLeg& leg = route[d.leg_index];
          const bool locked = switchToMap(leg.next_map, leg.seed_pose);
          obs.reloc_locked = locked;
          obs.reloc_failed = !locked;
          obs.now = nowSec();
          break;
        }
        case XmAction::GLOBAL_FALLBACK: {
          const bool locked = callRelocalizeOnce(/*use_guess=*/false);
          obs.reloc_locked = locked;
          obs.reloc_failed = !locked;
          obs.now = nowSec();
          break;
        }
        case XmAction::FINISH_OK:
          publishTransition(false);
          result.success = true;
          result.outcome = g1_msgs::NavigateToResult::OUTCOME_SUCCEEDED;
          result.message = "reached goal";
          as_->setSucceeded(result, result.message);
          return;
        case XmAction::FINISH_FAIL:
          publishTransition(false);
          result.success = false;
          result.outcome = leg_fail_outcome;
          result.message = "navigation aborted";
          as_->setAborted(result, result.message);
          return;
        case XmAction::NONE:
        default:
          ros::Duration(0.02).sleep();
          obs.now = nowSec();
          break;
      }
    }
    result.success = false;
    result.outcome = g1_msgs::NavigateToResult::OUTCOME_ABORTED_NO_PATH;
    as_->setAborted(result, "node shutting down");
  }

  // ---- nav points ----
  std::string navPointsPath(const std::string& name) const {
    return map_store_->overlaysDir(name) + "/nav_points.yaml";
  }

  // (Re)load the active map's nav points; call when active_map_ changes (mu_ held).
  void loadActiveNavPointsLocked() {
    nav_store_.load(active_map_.empty() ? "" : readFile(navPointsPath(active_map_)));
    nav_store_.setMapName(active_map_);
  }

  // Build a g1_msgs/NavPoint for `rec`, resolving its current world pose.
  g1_msgs::NavPoint makeNavPointMsg(const NavPointRec& rec, const std::string& map,
                                    const std::vector<KeyframePoseRec>& kfs) const {
    g1_msgs::NavPoint np;
    np.name = rec.name;
    np.map_name = map;
    np.anchor_keyframe_id = rec.anchor_id;
    np.anchor_relative_pose = toMsgPose(fromIso(rec.rel));
    np.source = rec.source;
    Pose3 world;
    if (nav_store_.resolve(rec.name, kfs, world)) np.pose = toMsgPose(world);
    return np;
  }

  void republishNavPointsLocked() {
    const std::vector<KeyframePoseRec> kfs = toKfRecs(kf_poses_);
    g1_msgs::NavPointArray arr;
    arr.header.stamp = ros::Time::now();
    arr.header.frame_id = map_frame_;
    arr.map_name = active_map_;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int id = 0;
    for (const NavPointRec& rec : nav_store_.all()) {
      const g1_msgs::NavPoint np = makeNavPointMsg(rec, active_map_, kfs);
      arr.points.push_back(np);
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = map_frame_;
      sphere.header.stamp = arr.header.stamp;
      sphere.ns = "nav_points";
      sphere.id = id++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose = np.pose;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25;
      sphere.color.g = 1.0f;
      sphere.color.a = 0.9f;
      markers.markers.push_back(sphere);
      visualization_msgs::Marker text = sphere;
      text.id = id++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.pose.position.z += 0.35;
      text.scale.z = 0.25;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.text = np.name;
      markers.markers.push_back(text);
    }
    nav_points_pub_.publish(arr);
    nav_point_markers_pub_.publish(markers);
  }

  // Back up the current nav_points.yaml into overlays/.history (bounded) then write.
  void backupAndWriteNavPoints(const std::string& name) {
    writeFileWithHistory(makeMapFs(), navPointsPath(name), map_store_->historyDir(name),
                         snapStamp(), nav_store_.serialize(), overlay_history_keep_);
  }

  bool addNavPointSrv(g1_msgs::AddNavPoint::Request& req,
                      g1_msgs::AddNavPoint::Response& res) {
    // Resolve the world pose first (TF lookup runs off the lock).
    Pose3 world;
    if (req.use_current_pose) {
      geometry_msgs::TransformStamped tf;
      try {
        tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                        ros::Duration(0.5));
      } catch (const tf2::TransformException& e) {
        res.success = false;
        res.message = std::string("TF ") + map_frame_ + "->" + base_frame_ +
                      " failed: " + e.what();
        return true;
      }
      geometry_msgs::Pose p;
      p.position.x = tf.transform.translation.x;
      p.position.y = tf.transform.translation.y;
      p.position.z = tf.transform.translation.z;
      p.orientation = tf.transform.rotation;
      world = fromMsgPose(p);
    } else {
      world = fromMsgPose(req.pose);
    }

    std::lock_guard<std::mutex> lk(mu_);
    if (!mm_.canWriteMap()) {
      res.success = false;
      res.message = "map is read-only (localization); nav point refused";
      return true;
    }
    const std::string target = req.map_name.empty() ? active_map_ : req.map_name;
    if (target.empty() || target != active_map_) {
      res.success = false;
      res.message = "nav points can only be authored on the active map";
      return true;
    }
    const std::vector<KeyframePoseRec> kfs = toKfRecs(kf_poses_);
    NavPointRec rec;
    std::string err;
    if (!nav_store_.add(req.name, world, kfs, nearest_anchor_max_dist_, req.overwrite,
                        rec, err)) {
      res.success = false;
      res.message = err;
      return true;
    }
    backupAndWriteNavPoints(target);
    republishNavPointsLocked();
    res.success = true;
    res.message = "added";
    res.point = makeNavPointMsg(rec, target, kfs);
    return true;
  }

  bool queryNavPointSrv(g1_msgs::QueryNavPoint::Request& req,
                        g1_msgs::QueryNavPoint::Response& res) {
    std::lock_guard<std::mutex> lk(mu_);
    const std::string target = req.map_name.empty() ? active_map_ : req.map_name;
    // Active-map query resolves against the live cached keyframe poses; non-active
    // (cross-map) resolution from pose_graph.g2o is wired with the action server.
    if (target.empty() || target != active_map_) {
      res.found = false;
      return true;
    }
    const std::vector<KeyframePoseRec> kfs = toKfRecs(kf_poses_);
    Pose3 world;
    if (!nav_store_.resolve(req.query, kfs, world)) {
      res.found = false;
      return true;
    }
    for (const NavPointRec& rec : nav_store_.all()) {
      if (rec.name == req.query) {
        res.point = makeNavPointMsg(rec, target, kfs);
        break;
      }
    }
    res.found = true;
    return true;
  }

  void stateTimerCb(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lk(mu_);
    publishStateLocked(detail_);
  }

  // ---- state (mu_ held) ----
  g1_msgs::MapManagerState buildStateLocked(const std::string& detail) const {
    g1_msgs::MapManagerState s;
    s.header.stamp = ros::Time::now();
    s.active_map = active_map_;
    s.current_label = current_label_;
    s.mode = modeToMsg(mm_.mode());
    s.localized = localized_;
    s.transition = transition_;
    s.detail = detail;
    return s;
  }
  void publishStateLocked(const std::string& detail) {
    detail_ = detail;
    state_pub_.publish(buildStateLocked(detail));
  }

  std::mutex mu_;
  std::unique_ptr<MapStore> map_store_;
  ModeMachine mm_;
  NavPointStore nav_store_;
  TopologyGraph topo_;
  CaptureMachine capture_;     // gateway begin/load/commit ordering (guarded by mu_)
  GatewayEditor gw_editor_;    // post-commit staged edge + nudge/yaw/save (guarded by mu_)
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::unique_ptr<actionlib::SimpleActionServer<g1_msgs::NavigateToAction>> as_;
  std::unique_ptr<actionlib::SimpleActionClient<g1_msgs::NavigateToAction>> ac_;
  std::string maps_root_, state_topic_, kf_poses_topic_, topology_path_;
  std::string map_nav_action_name_, nav_action_name_;
  double transition_wait_ = 3.0;
  std::string map_frame_, base_frame_, nav_points_topic_, nav_point_markers_topic_,
      gateway_markers_topic_, gateway_capture_service_;
  std::string load_srv_name_, save_srv_name_, reloc_srv_name_, begin_incr_srv_name_,
      reset_srv_name_;
  double state_rate_ = 5.0, reloc_seed_timeout_ = 5.0, reloc_global_timeout_ = 15.0;
  double nearest_anchor_max_dist_ = 5.0;
  int overlay_history_keep_ = 5, topology_history_keep_ = 5, snapshot_auto_keep_ = 2;

  // Manager state (guarded by mu_).
  std::string active_map_, current_label_, pre_incr_snapshot_, detail_;
  bool localized_ = false;
  bool transition_ = false;
  g1_msgs::KeyframePoseArray::ConstPtr kf_poses_;

  ros::ServiceClient load_client_, save_client_, reloc_client_, begin_incr_client_,
      reset_client_;
  ros::Subscriber kf_poses_sub_;
  ros::Publisher state_pub_, nav_points_pub_, nav_point_markers_pub_,
      gateway_markers_pub_;
  ros::ServiceServer start_mapping_srv_, start_incremental_srv_, start_localization_srv_,
      stop_mapping_srv_, list_maps_srv_, get_active_srv_, snapshot_srv_,
      add_nav_srv_, query_nav_srv_, reload_topo_srv_, get_topology_srv_,
      get_neighbors_srv_, gateway_capture_srv_;
  ros::Timer state_timer_;
};

}  // namespace g1_map_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_map_manager");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_map_manager::MapManagerNode node(nh, pnh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
