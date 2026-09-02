#include "g1_slam_backend/scan_context.h"

#include <algorithm>
#include <cmath>

namespace g1_slam_backend {

ScanContextDB::ScanContextDB(const ScanContextConfig& cfg) : cfg_(cfg) {}

ScanContextDB::Descriptor ScanContextDB::make(
    const std::vector<Eigen::Vector3d>& points) const {
  Descriptor sc = Descriptor::Zero(cfg_.num_rings, cfg_.num_sectors);
  const double two_pi = 2.0 * M_PI;
  for (const Eigen::Vector3d& p : points) {
    const double r = std::hypot(p.x(), p.y());
    if (r < cfg_.min_range || r >= cfg_.max_range) continue;
    double theta = std::atan2(p.y(), p.x());
    if (theta < 0.0) theta += two_pi;

    int ring = static_cast<int>(r / cfg_.max_range * cfg_.num_rings);
    int sector = static_cast<int>(theta / two_pi * cfg_.num_sectors);
    if (ring >= cfg_.num_rings) ring = cfg_.num_rings - 1;
    if (sector >= cfg_.num_sectors) sector = cfg_.num_sectors - 1;

    const double h = p.z() + cfg_.height_offset;  // max-Z encoding; 0 == empty bin
    if (h > sc(ring, sector)) sc(ring, sector) = h;
  }
  return sc;
}

Eigen::VectorXd ScanContextDB::ringKey(const Descriptor& desc) const {
  Eigen::VectorXd key(desc.rows());
  for (int i = 0; i < desc.rows(); ++i) {
    key(i) = desc.row(i).mean();
  }
  return key;
}

void ScanContextDB::add(Key id, const Descriptor& desc) {
  entries_.push_back(Entry{id, desc, ringKey(desc)});
}

std::pair<double, int> ScanContextDB::alignedDistance(const Descriptor& a,
                                                      const Descriptor& b) const {
  const int ns = static_cast<int>(a.cols());
  double best = 1.0;
  int best_shift = 0;
  for (int s = 0; s < ns; ++s) {
    double sum = 0.0;
    int count = 0;
    for (int j = 0; j < ns; ++j) {
      const int k = (j + s) % ns;
      const double na = a.col(j).norm();
      const double nb = b.col(k).norm();
      if (na < 1e-9 || nb < 1e-9) continue;  // skip empty columns
      sum += a.col(j).dot(b.col(k)) / (na * nb);
      ++count;
    }
    if (count == 0) continue;
    const double dist = 1.0 - sum / count;
    if (dist < best) {
      best = dist;
      best_shift = s;
    }
  }
  return {best, best_shift};
}

ScanContextDB::QueryResult ScanContextDB::query(Key query_id,
                                                const Descriptor& desc,
                                                int min_id_gap,
                                                double dist_thresh,
                                                int knn) const {
  QueryResult res;
  if (entries_.empty()) return res;

  const Eigen::VectorXd qkey = ringKey(desc);

  // Ring-key pre-filter: collect eligible entries (recent ids excluded), keep
  // the KNN nearest by ring-key L2 distance. Linear scan is fine at our scale;
  // swap in nanoflann/a KD-tree if keyframe counts grow large.
  std::vector<std::pair<double, std::size_t>> cand;
  cand.reserve(entries_.size());
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const Key id = entries_[i].id;
    const long gap =
        (id > query_id) ? static_cast<long>(id - query_id)
                        : static_cast<long>(query_id - id);
    if (gap < min_id_gap) continue;  // skip recent keyframes
    cand.emplace_back((entries_[i].ringkey - qkey).norm(), i);
  }
  if (cand.empty()) return res;

  const int k = std::min<int>(knn, static_cast<int>(cand.size()));
  std::partial_sort(cand.begin(), cand.begin() + k, cand.end());

  double best_dist = dist_thresh;
  for (int c = 0; c < k; ++c) {
    const Entry& e = entries_[cand[c].second];
    const std::pair<double, int> da = alignedDistance(desc, e.sc);
    if (da.first < best_dist) {
      best_dist = da.first;
      res.found = true;
      res.match_id = e.id;
      res.distance = da.first;
      res.yaw = da.second * (2.0 * M_PI / cfg_.num_sectors);
    }
  }
  return res;
}

std::vector<ScanContextDB::QueryResult> ScanContextDB::queryCandidates(
    const Descriptor& desc, int knn, int num) const {
  std::vector<QueryResult> out;
  if (entries_.empty() || num <= 0) return out;

  const Eigen::VectorXd qkey = ringKey(desc);
  std::vector<std::pair<double, std::size_t>> cand;
  cand.reserve(entries_.size());
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    cand.emplace_back((entries_[i].ringkey - qkey).norm(), i);
  }
  const int k = std::min<int>(knn, static_cast<int>(cand.size()));
  std::partial_sort(cand.begin(), cand.begin() + k, cand.end());

  std::vector<QueryResult> scored;
  scored.reserve(k);
  for (int c = 0; c < k; ++c) {
    const Entry& e = entries_[cand[c].second];
    const std::pair<double, int> da = alignedDistance(desc, e.sc);
    QueryResult r;
    r.found = true;
    r.match_id = e.id;
    r.distance = da.first;
    r.yaw = da.second * (2.0 * M_PI / cfg_.num_sectors);
    scored.push_back(r);
  }
  std::sort(scored.begin(), scored.end(),
            [](const QueryResult& a, const QueryResult& b) {
              return a.distance < b.distance;
            });
  const int n = std::min<int>(num, static_cast<int>(scored.size()));
  out.assign(scored.begin(), scored.begin() + n);
  return out;
}

std::vector<Eigen::Vector3d> ScanContextDB::toScanContextFrame(
    const std::vector<Eigen::Vector3d>& points_base,
    const Eigen::Matrix3d& odom_R_base) {
  // Yaw about gravity (odom z) from the base orientation, then the residual
  // roll/pitch R_tilt = Rz(yaw)^T * odom_R_base. p_sc = R_tilt * p_base keeps the
  // heading but aligns z with gravity (so p_sc.z equals the true odom height).
  const double yaw = std::atan2(odom_R_base(1, 0), odom_R_base(0, 0));
  const Eigen::Matrix3d rz =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d r_tilt = rz.transpose() * odom_R_base;

  std::vector<Eigen::Vector3d> out;
  out.reserve(points_base.size());
  for (const Eigen::Vector3d& p : points_base) {
    out.push_back(r_tilt * p);
  }
  return out;
}

}  // namespace g1_slam_backend
