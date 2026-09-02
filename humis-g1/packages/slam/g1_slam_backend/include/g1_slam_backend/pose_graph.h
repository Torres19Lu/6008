#pragma once

#include <cstdint>
#include <map>
#include <memory>

#include <Eigen/Geometry>

namespace g1_slam_backend {

// Incremental keyframe pose graph: a thin wrapper over GTSAM iSAM2.
//
// The public interface is GTSAM-free (Eigen only), via the pimpl idiom, so the
// rest of the backend and the unit tests do not depend on GTSAM headers/types.
// Node poses are map-frame rigid transforms T(map<-node). Relative measurements
// follow the GTSAM BetweenFactor convention: rel = T(from<-to). Measurement
// noise is isotropic, given as rotation (rad) and translation (m) standard
// deviations.
class PoseGraph {
 public:
  using Key = std::uint64_t;

  PoseGraph();
  ~PoseGraph();

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  // Anchor the graph: a prior on `key` at `pose`. Use on the first keyframe to
  // fix the gauge. Also seeds the node's initial estimate.
  void addPrior(Key key, const Eigen::Isometry3d& pose,
                double rot_sigma, double trans_sigma);

  // Odometry edge to a NEW node `to`: measured relative pose rel = T(from<-to).
  // The new node's initial estimate is composed from the current estimate of
  // `from` (so open-loop drift accumulates exactly as the front end reports it).
  void addOdometry(Key from, Key to, const Eigen::Isometry3d& rel_pose,
                   double rot_sigma, double trans_sigma);

  // Loop-closure edge between two EXISTING nodes: rel = T(from<-to) from ICP.
  // Adds no new node or initial estimate. `cauchy_c` > 0 wraps the factor in a
  // Cauchy M-estimator (robust kernel) so an outlier loop that slips past the
  // acceptance gate is down-weighted instead of distorting the whole graph;
  // <= 0 keeps a plain Gaussian factor.
  void addLoop(Key from, Key to, const Eigen::Isometry3d& rel_pose,
               double rot_sigma, double trans_sigma, double cauchy_c);

  // Flush the factors/values queued since the last update through iSAM2.
  // `extra_iters` runs additional re-linearization passes; use >0 right after a
  // loop edge so the correction propagates around the cycle.
  void update(int extra_iters = 0);

  // Reset to an empty graph (drops all factors, values, and estimates). Used by
  // Backend::loadMap to rebuild the graph from a serialized map.
  void clear();

  // Current optimized estimate (falls back to the seeded init before the first
  // update touches a node).
  Eigen::Isometry3d optimizedPose(Key key) const;
  std::map<Key, Eigen::Isometry3d> allPoses() const;
  bool exists(Key key) const;
  std::size_t size() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace g1_slam_backend
