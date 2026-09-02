#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g1_slam_backend/keyframe.h"
#include "g1_slam_backend/pose_graph.h"
#include "g1_slam_backend/scan_context.h"

namespace g1_slam_backend {

struct BackendConfig {
  // Keyframe selection (relative to the previous keyframe).
  double keyframe_dist = 1.0;     // m
  double keyframe_angle = 0.2;    // rad
  // Cloud downsampling (<= 0 disables).
  double keyframe_voxel = 0.3;    // m, stored keyframe clouds
  double map_voxel = 0.4;         // m, assembled map
  // Scan Context loop detection.
  ScanContextConfig sc;
  int sc_min_id_gap = 30;         // exclude this many recent keyframes from the loop search
  double sc_dist_thresh = 0.2;    // SC distance below which a candidate is considered
  int sc_knn = 10;
  // ICP geometric verification.
  double icp_max_corr_dist = 1.0;
  int icp_max_iter = 50;
  double icp_fitness_thresh = 0.3;  // accept the loop if the ICP fitness score is below this
  int icp_submap_neighbors = 8;     // +/- neighbors assembled around the candidate (loop + reloc); larger submap = more ICP correspondences, bounded by loop_voxel
  double loop_icp_voxel = 0.4;      // m; re-voxel the assembled loop TARGET submap before ICP (<=0 disables); NOT the single-keyframe source. Normalizes the submap density so point-to-point ICP is not biased by overlap, and bounds cost
  // Relocalization geometric acceptance (more discriminative than fitness alone):
  // try several SC candidates, accept the alignment whose inlier ratio (fraction
  // of live points within inlier_dist of the map) clears reloc_min_inlier.
  double reloc_min_inlier = 0.5;    // required inlier fraction to accept a relocalization
  double reloc_inlier_dist = 0.3;   // m; a live point within this of the map counts as an inlier
  int reloc_num_candidates = 5;     // SC candidates ICP-verified per acquire
  // Localization tracking observability: nearest neighbours used to estimate the
  // live-scan surface normals that build the point-to-plane info (degeneracy)
  // matrix returned by trackAgainstSubmap.
  int track_normal_k = 10;
  // Pose-graph noise (isotropic rotation [rad] / translation [m] sigmas). Loop
  // sigmas are deliberately LOOSER than odom: the Cauchy kernel (loop_robust_c)
  // provides the trust control, so a tight loop sigma would only amplify outliers.
  double prior_rot_sigma = 1e-4, prior_trans_sigma = 1e-4;
  double odom_rot_sigma = 0.05, odom_trans_sigma = 0.1;
  double loop_rot_sigma = 0.3, loop_trans_sigma = 0.5;
  int loop_extra_iters = 5;
  // Loop-closure robustness against false/degenerate matches (corridors and
  // repetitive rooms produce them; an accurate front end lets us reject them).
  double loop_robust_c = 1.0;       // Cauchy kernel param for loop factors (<=0 = plain Gaussian)
  // Consistency gate: a true loop's ICP measurement must agree with the
  // front-end odometry-chained relative pose within accumulated drift. Reject if
  // the disagreement exceeds (floor + rate * traveled path length between the pair).
  double loop_max_dt = 0.5;         // m;   translation-disagreement floor
  double loop_max_dt_rate = 0.005;  //      + fraction of traveled path
  double loop_max_dr = 0.21;        // rad (~12 deg); rotation-disagreement floor
  double loop_max_dr_rate = 0.0005; // rad per m of traveled path
};

// Backend orchestration: keyframe selection, the storage-3 cloud transform,
// odometry/loop factors, Scan Context -> ICP loop verification, iSAM2
// optimization, and the map->odom correction. ROS-free and std::thread-free so
// it is unit-testable; the ROS node (Task 5) owns the threads and the mutex,
// calling the fast stepIntake() on the odom+cloud sync and the slow
// runLoopClosureOnce() on a separate loop-closure thread.
class Backend {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  // A consistent, lock-free-assemblable view of one keyframe: its optimized
  // map-frame pose + an immutable handle to its base_link cloud (the cloud is
  // never mutated after creation, so reading it off the lock is safe).
  struct KeyframeView {
    Eigen::Isometry3d pose;  // T(map<-base_link_i), optimized
    Cloud::ConstPtr cloud;   // base_link_i frame
  };

  // A loop-closure edge and its ICP measurement rel = T(from<-to). The
  // measurement is retained (not just the id pair) so saveMap can write a
  // faithful, re-optimizable g2o pose graph.
  struct LoopEdge {
    std::uint64_t from = 0;
    std::uint64_t to = 0;
    Eigen::Isometry3d rel = Eigen::Isometry3d::Identity();  // T(from<-to), ICP
  };

  // Result of a relocalization attempt against the (loaded) map. The diagnostic
  // fields (fitness, inlier_ratio, sc_distance, match_id) are filled with the
  // BEST attempt even when `found` is false, so the node can log why it rejected.
  struct RelocResult {
    bool found = false;
    Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
    double fitness = 0.0;         // ICP fitness score (mean sq corr distance)
    double inlier_ratio = 0.0;    // fraction of source points within inlier_dist (the accept gate)
    double sc_distance = 1.0;     // Scan Context distance of the chosen candidate
    std::uint64_t match_id = 0;   // matched keyframe id (global acquire)
    // Registration observability at the converged pose: the point-to-plane
    // information (Hessian), ordered [trans(3); rot(3)] in the map frame about the
    // sensor position, mean-normalized over the live scan. Filled by
    // trackAgainstSubmap (localization tracking) and consumed by
    // computeTrackingCorrection; a near-zero eigenvalue marks an unconstrained DOF
    // (e.g. sliding across a dominant table plane). The Identity default means "no
    // observability info" and is treated as fully observable (no projection), so a
    // result from a path that does not compute it is safe to pass through.
    Eigen::Matrix<double, 6, 6> obs_info = Eigen::Matrix<double, 6, 6>::Identity();
  };

  explicit Backend(const BackendConfig& cfg);

  // FAST path. Decides whether this frame is a keyframe; if so stores it
  // (storage 3), adds the odometry factor (or the prior for the first), builds
  // its Scan Context, updates the graph, recomputes map->odom, and queues it for
  // a loop check. Returns true iff a keyframe was created.
  bool stepIntake(double stamp, const Eigen::Isometry3d& odom_pose,
                  const Cloud& cloud_world);

  // SLOW path. Pops one queued keyframe, runs an SC query + ICP verification,
  // and on success adds the loop factor, re-optimizes, and recomputes map->odom.
  // Returns true iff a loop was accepted. No-op (returns false) if nothing is
  // queued or no loop is found.
  bool runLoopClosureOnce();
  bool hasPendingLoopChecks() const { return !pending_loops_.empty(); }

  // Current correction T(map<-odom) (identity until the first keyframe).
  Eigen::Isometry3d mapToOdom() const { return map_to_odom_; }

  // Set the map->odom correction directly. Used in localization mode (frozen map):
  // the node applies a relocalization result here so the existing TF/publish
  // path stays the single source. In mapping mode stepIntake derives it.
  void setMapToOdom(const Eigen::Isometry3d& T) { map_to_odom_ = T; }

  // Global relocalization (kidnapped robot / acquire). Scan Context query of the
  // live scan against the WHOLE map, then ICP against the matched keyframe's
  // submap (yaw seeded from the SC column shift; both signs tried, best fit
  // kept). cloud_world is the live registered cloud (odom frame); odom_pose is
  // the paired P_now = T(odom<-base_link). Pure: returns the result without
  // mutating state (the node decides whether to apply it via setMapToOdom).
  RelocResult relocalizeGlobal(const Eigen::Isometry3d& odom_pose,
                               const Cloud& cloud_world) const;

  // Local relocalization (tracking). Scan-to-map ICP of the live scan against a
  // submap of keyframes within `radius` of the predicted map pose, seeded by
  // that prediction. Use once locked to refine map->odom and absorb odom drift.
  // Pure (see relocalizeGlobal).
  RelocResult relocalizeLocal(const Eigen::Isometry3d& odom_pose,
                              const Cloud& cloud_world,
                              const Eigen::Isometry3d& predicted_map_pose,
                              double radius) const;

  // Track against a prebuilt map-frame submap: ICP the live scan (seeded by the
  // predicted map pose) into `submap_map`, return the refined map->odom. Lets the
  // node cache the submap (via snapshotKeyframesNear + assembleFromSnapshot) and
  // run both the assembly and this ICP OFF the mutex, so the tf broadcast is not
  // blocked. Pure (reads only the passed submap + cfg).
  RelocResult trackAgainstSubmap(const Eigen::Isometry3d& odom_pose,
                                 const Cloud& cloud_world,
                                 const Eigen::Isometry3d& predicted_map_pose,
                                 const Cloud::ConstPtr& submap_map) const;

  // Optimized global map: union of P_i' * cloud_base_i, voxel-downsampled.
  // (= assembleFromSnapshot(snapshotKeyframes(), map_voxel); kept for tests.)
  Cloud::Ptr assembleMap() const;

  // Cheap, consistent snapshot the node takes UNDER the mutex so the heavy map
  // assembly can run OFF the mutex (no intake stall). Copies optimized poses +
  // bumps cloud shared-ptr refcounts only.
  std::vector<KeyframeView> snapshotKeyframes() const;
  // Like snapshotKeyframes but only keyframes whose optimized position is within
  // `radius` of `center` (map frame). For cached localization tracking: take it
  // under the lock, then assemble + match OFF the lock (the loaded map is frozen
  // during localization, so the cloud handles stay valid).
  std::vector<KeyframeView> snapshotKeyframesNear(const Eigen::Vector3d& center,
                                                  double radius) const;
  // Heavy assembly (transform + concat + voxel); static and lock-free.
  static Cloud::Ptr assembleFromSnapshot(const std::vector<KeyframeView>& kfs,
                                         double map_voxel);
  double mapVoxel() const { return cfg_.map_voxel; }

  std::size_t numKeyframes() const { return keyframes_.size(); }
  std::vector<Eigen::Isometry3d> optimizedPoses() const;  // ordered by id
  const std::vector<LoopEdge>& loopEdges() const { return loop_edges_; }

  // Persistence. saveMap writes an inspectable, re-optimizable map
  // directory (manifest.yaml + pose_graph.g2o + scan_context.bin +
  // keyframes/NNNNNN.pcd) atomically (temp dir + swap). loadMap rebuilds the
  // keyframes, Scan Context DB, and pose graph from such a directory (replacing
  // any current state), adopting the map's Scan Context config. loadMap leaves
  // map->odom unset (Identity): it is established only by relocalization, since
  // the live odom frame is unrelated to the loaded map until then. Returns false
  // and fills *message on failure. The loaded map is frozen (localization-only);
  // re-saving a loaded map is not supported.
  bool saveMap(const std::string& dir, std::string* message = nullptr) const;
  bool loadMap(const std::string& dir, std::string* message = nullptr);

  // Clear all state back to a fresh mapping backend (keyframes, pending loops,
  // loop edges, the iSAM2 graph, the Scan Context DB, and map->odom). Used by the
  // node's /slam/reset to start a brand-new mapping session.
  void reset();

  // Re-base each loaded keyframe's odom_pose into the LIVE odom frame so MAPPING
  // can resume on a loaded map after a relocalization lock. loadMap leaves each
  // odom_pose as a placeholder (the optimized map pose), which would make the
  // first new keyframe's bridge factor garbage. With map_to_odom = T (set by a
  // prior relocalization), this sets odom_pose := T^-1 * optimizedPose(id), so a
  // new live frame P_new lands at T * P_new in the map frame and stepIntake /
  // saveMap stay consistent. Precondition: map_to_odom_ established by a prior
  // relocalization (else the identity default treats the live odom frame as map).
  void beginIncremental();

 private:
  void updateMapToOdom();
  std::vector<Eigen::Vector3d> toEigenPoints(const Cloud& cloud) const;
  // Pull a live registered (odom-frame) cloud into base_link and voxel it (the
  // storage-3 transform, shared by stepIntake's intent and relocalization).
  Cloud::Ptr toBaseCloud(const Eigen::Isometry3d& odom_pose,
                         const Cloud& cloud_world) const;

  BackendConfig cfg_;
  PoseGraph graph_;
  ScanContextDB scdb_;
  std::vector<Keyframe> keyframes_;
  std::vector<std::uint64_t> pending_loops_;
  std::vector<LoopEdge> loop_edges_;
  Eigen::Isometry3d map_to_odom_ = Eigen::Isometry3d::Identity();
};

}  // namespace g1_slam_backend
