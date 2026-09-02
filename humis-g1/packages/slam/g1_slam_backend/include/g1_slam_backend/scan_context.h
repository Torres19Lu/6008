#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g1_slam_backend {

struct ScanContextConfig {
  int num_rings = 20;          // Ng: radial bins
  int num_sectors = 60;        // Ns: azimuthal bins
  double max_range = 80.0;     // m; points at/over this range are ignored
  double min_range = 0.5;      // m; points under this range are ignored (self/near returns)
  double height_offset = 2.0;  // m added to z so encoded heights are positive (0 = empty bin)
};

// Scan Context (Kim & Kim, IROS 2018) loop detection: a polar max-height
// descriptor + a rotation-invariant ring key for KNN pre-filtering, then a
// column-shift-aligned descriptor distance that also yields a relative-yaw
// estimate. Reimplemented from the published algorithm.
class ScanContextDB {
 public:
  using Key = std::uint64_t;
  using Descriptor = Eigen::MatrixXd;  // num_rings x num_sectors, max-Z per bin

  struct QueryResult {
    bool found = false;
    Key match_id = 0;
    double distance = 1.0;  // SC distance in [0,1]; 0 = identical
    double yaw = 0.0;       // estimated relative yaw (rad) from the best column shift
  };

  explicit ScanContextDB(const ScanContextConfig& cfg = ScanContextConfig());

  const ScanContextConfig& config() const { return cfg_; }
  std::size_t size() const { return entries_.size(); }

  // Build the descriptor for a cloud already in the SC frame (sensor-centered,
  // gravity-aligned: x-y horizontal, z up). Use toScanContextFrame() to obtain
  // such a cloud from a base_link cloud.
  Descriptor make(const std::vector<Eigen::Vector3d>& points) const;

  // Rotation-invariant ring key: one mean-height value per ring.
  Eigen::VectorXd ringKey(const Descriptor& desc) const;

  // Add a keyframe descriptor under integer id `id`.
  void add(Key id, const Descriptor& desc);

  // Ring-key KNN pre-filter (excluding ids within min_id_gap of query_id), then
  // column-aligned SC distance on the candidates. Returns the best match only if
  // its distance < dist_thresh.
  QueryResult query(Key query_id, const Descriptor& desc, int min_id_gap,
                    double dist_thresh, int knn = 10) const;

  // Relocalization retrieval: the up-to-`num` best candidates by column-aligned
  // SC distance (ring-key KNN pre-filter, `knn` examined), each with its yaw. No
  // id-gap exclusion and no distance gate - geometric verification (ICP inlier
  // ratio) downstream decides, so SC only has to propose a short list.
  std::vector<QueryResult> queryCandidates(const Descriptor& desc, int knn,
                                           int num) const;

  // Express a base_link-frame cloud in the SC frame: strip roll/pitch from the
  // base orientation (keep yaw) so z aligns with gravity. odom_R_base is the
  // keyframe's base_link orientation in the gravity-aligned odom frame. This is
  // the humanoid refinement: gait roll/pitch must not corrupt the height bins.
  static std::vector<Eigen::Vector3d> toScanContextFrame(
      const std::vector<Eigen::Vector3d>& points_base,
      const Eigen::Matrix3d& odom_R_base);

 private:
  struct Entry {
    Key id;
    Descriptor sc;
    Eigen::VectorXd ringkey;
  };

  // Minimum column-aligned distance between two descriptors and the column shift
  // that achieves it (the shift maps to the relative-yaw estimate).
  std::pair<double, int> alignedDistance(const Descriptor& a,
                                         const Descriptor& b) const;

  ScanContextConfig cfg_;
  std::vector<Entry> entries_;
};

}  // namespace g1_slam_backend
