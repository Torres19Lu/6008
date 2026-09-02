#include "g1_slam_backend/backend.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

namespace g1_slam_backend {
namespace {

namespace fs = std::filesystem;

// ----- Map directory layout -----
constexpr int kMapFormatVersion = 1;
constexpr char kManifestName[] = "manifest.yaml";
constexpr char kGraphName[] = "pose_graph.g2o";
constexpr char kScName[] = "scan_context.bin";
constexpr char kKeyframesDir[] = "keyframes";

std::string keyframePcdName(std::uint64_t id) {
  std::ostringstream os;
  os << std::setw(6) << std::setfill('0') << id << ".pcd";
  return os.str();
}

// --- g2o text IO (own minimal VERTEX_SE3:QUAT / EDGE_SE3:QUAT, so the file is
// g2o_viewer-readable and GTSAM-version-independent; on load the edge noise comes
// from the config, not the stored information matrix). ---

void writeQuatPose(std::ostream& os, const Eigen::Isometry3d& T) {
  const Eigen::Vector3d t = T.translation();
  Eigen::Quaterniond q(T.rotation());
  q.normalize();
  os << t.x() << ' ' << t.y() << ' ' << t.z() << ' ' << q.x() << ' ' << q.y()
     << ' ' << q.z() << ' ' << q.w();
}

Eigen::Isometry3d readQuatPose(std::istream& is) {
  double tx, ty, tz, qx, qy, qz, qw;
  is >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();
  T.linear() = q.toRotationMatrix();
  T.translation() << tx, ty, tz;
  return T;
}

// Upper-triangular 6x6 information block (translation-first ordering, matching
// g2o EDGE_SE3:QUAT) from isotropic sigmas. Diagonal only.
void writeInfoUpperTri(std::ostream& os, double rot_sigma, double trans_sigma) {
  const double it = 1.0 / (trans_sigma * trans_sigma);
  const double ir = 1.0 / (rot_sigma * rot_sigma);
  const double diag[6] = {it, it, it, ir, ir, ir};
  for (int r = 0; r < 6; ++r) {
    for (int c = r; c < 6; ++c) {
      os << ' ' << (r == c ? diag[r] : 0.0);
    }
  }
}

// --- minimal key:value YAML (fixed schema, ASCII, no yaml-cpp dependency) ---
bool parseManifest(const fs::path& path, std::map<std::string, std::string>* kv) {
  std::ifstream in(path);
  if (!in) return false;
  std::string line;
  while (std::getline(in, line)) {
    const std::size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon);
    std::string val = line.substr(colon + 1);
    auto trim = [](std::string& s) {
      const char* ws = " \t\r\n";
      const std::size_t b = s.find_first_not_of(ws);
      const std::size_t e = s.find_last_not_of(ws);
      s = (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
    };
    trim(key);
    trim(val);
    if (!key.empty()) (*kv)[key] = val;
  }
  return true;
}

// Fraction of `aligned` points with a `target` point within `dist` (geometric
// overlap). More discriminative than ICP fitness for accepting a relocalization:
// a wrong alignment may have low mean distance but few true correspondences.
double inlierRatio(const pcl::PointCloud<pcl::PointXYZI>& aligned,
                   pcl::KdTreeFLANN<pcl::PointXYZI>& target_kd, double dist) {
  if (aligned.empty()) return 0.0;
  const double d2 = dist * dist;
  std::vector<int> idx(1);
  std::vector<float> sq(1);
  std::size_t in = 0;
  for (const pcl::PointXYZI& p : aligned) {
    if (target_kd.nearestKSearch(p, 1, idx, sq) > 0 && sq[0] <= d2) ++in;
  }
  return static_cast<double>(in) / static_cast<double>(aligned.size());
}

// Point-to-plane registration information (Hessian) of a base-frame scan at the
// converged map pose x_now, ordered [trans(3); rot(3)] in the map frame about the
// sensor position and mean-normalized. Each point contributes
//   J_i = [ n_i ; (p_i - c) x n_i ]   (n_i, p_i in map frame, c = sensor origin)
// so the translation block is sum n_i n_i^T: a dominant plane leaves the two
// in-plane translations near-zero (the unobservable slide directions).
Eigen::Matrix<double, 6, 6> pointToPlaneInfo(
    const pcl::PointCloud<pcl::PointXYZI>& cloud_base,
    const Eigen::Isometry3d& x_now, int normal_k) {
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  if (cloud_base.size() < static_cast<std::size_t>(std::max(3, normal_k))) return H;

  pcl::PointCloud<pcl::PointXYZI>::Ptr in(
      new pcl::PointCloud<pcl::PointXYZI>(cloud_base));
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(in);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setSearchMethod(tree);
  ne.setKSearch(std::max(3, normal_k));
  pcl::PointCloud<pcl::Normal> normals;
  ne.compute(normals);

  const Eigen::Matrix3d R = x_now.linear();
  const Eigen::Vector3d c = x_now.translation();
  std::size_t used = 0;
  for (std::size_t i = 0; i < in->size(); ++i) {
    const pcl::Normal& nrm = normals[i];
    if (!std::isfinite(nrm.normal_x) || !std::isfinite(nrm.normal_y) ||
        !std::isfinite(nrm.normal_z)) {
      continue;
    }
    const Eigen::Vector3d n_map =
        R * Eigen::Vector3d(nrm.normal_x, nrm.normal_y, nrm.normal_z);
    const Eigen::Vector3d p_map =
        x_now * Eigen::Vector3d((*in)[i].x, (*in)[i].y, (*in)[i].z);
    Eigen::Matrix<double, 6, 1> J;
    J.head<3>() = n_map;
    J.tail<3>() = (p_map - c).cross(n_map);
    H += J * J.transpose();
    ++used;
  }
  if (used > 0) H /= static_cast<double>(used);
  return H;
}

}  // namespace

Backend::Backend(const BackendConfig& cfg) : cfg_(cfg), scdb_(cfg.sc) {}

std::vector<Eigen::Vector3d> Backend::toEigenPoints(const Cloud& cloud) const {
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(cloud.size());
  for (const pcl::PointXYZI& p : cloud) {
    pts.emplace_back(p.x, p.y, p.z);
  }
  return pts;
}

void Backend::updateMapToOdom() {
  if (keyframes_.empty()) {
    map_to_odom_ = Eigen::Isometry3d::Identity();
    return;
  }
  const Keyframe& last = keyframes_.back();
  // map->odom = T(map<-base_last) * T(base_last<-odom) = X_last * P_last^-1.
  map_to_odom_ = graph_.optimizedPose(last.id) * last.odom_pose.inverse();
}

bool Backend::stepIntake(double stamp, const Eigen::Isometry3d& odom_pose,
                         const Cloud& cloud_world) {
  const bool first = keyframes_.empty();
  Eigen::Isometry3d rel = Eigen::Isometry3d::Identity();
  if (!first) {
    rel = keyframes_.back().odom_pose.inverse() * odom_pose;
    const double dt = rel.translation().norm();
    const double da = std::abs(Eigen::AngleAxisd(rel.rotation()).angle());
    if (dt < cfg_.keyframe_dist && da < cfg_.keyframe_angle) {
      return false;  // not enough motion since the last keyframe
    }
  }

  const std::uint64_t id = keyframes_.size();

  // Storage 3: pull the registered (odom-frame) cloud into base_link_id.
  Cloud::Ptr cloud_base = toBaseCloud(odom_pose, cloud_world);

  // Scan Context in the yaw-only gravity-aligned frame (gait roll/pitch removed).
  const std::vector<Eigen::Vector3d> sc_pts = ScanContextDB::toScanContextFrame(
      toEigenPoints(*cloud_base), odom_pose.rotation());
  const ScanContextDB::Descriptor sc = scdb_.make(sc_pts);

  if (first) {
    graph_.addPrior(id, odom_pose, cfg_.prior_rot_sigma, cfg_.prior_trans_sigma);
  } else {
    graph_.addOdometry(keyframes_.back().id, id, rel, cfg_.odom_rot_sigma,
                       cfg_.odom_trans_sigma);
  }

  Keyframe kf;
  kf.id = id;
  kf.stamp = stamp;
  kf.odom_pose = odom_pose;
  kf.cloud_base = cloud_base;
  kf.sc = sc;
  keyframes_.push_back(kf);
  scdb_.add(id, sc);
  pending_loops_.push_back(id);

  graph_.update();
  updateMapToOdom();
  return true;
}

bool Backend::runLoopClosureOnce() {
  if (pending_loops_.empty()) return false;
  const std::uint64_t id = pending_loops_.front();
  pending_loops_.erase(pending_loops_.begin());

  const Keyframe& kf = keyframes_[id];
  const ScanContextDB::QueryResult q = scdb_.query(
      id, kf.sc, cfg_.sc_min_id_gap, cfg_.sc_dist_thresh, cfg_.sc_knn);
  if (!q.found) return false;
  const std::uint64_t cand = q.match_id;

  // Target submap: the candidate (+/- neighbors) brought into the candidate's
  // base frame via current relative estimates T(cand<-k) = X_cand^-1 * X_k.
  Cloud::Ptr target(new Cloud());
  const Eigen::Isometry3d x_cand = graph_.optimizedPose(cand);
  const long n = static_cast<long>(keyframes_.size());
  for (long k = static_cast<long>(cand) - cfg_.icp_submap_neighbors;
       k <= static_cast<long>(cand) + cfg_.icp_submap_neighbors; ++k) {
    if (k < 0 || k >= n) continue;
    const Eigen::Isometry3d rel =
        x_cand.inverse() * graph_.optimizedPose(static_cast<std::uint64_t>(k));
    Cloud tmp;
    pcl::transformPointCloud(*keyframes_[k].cloud_base, tmp,
                             rel.matrix().cast<float>());
    *target += tmp;
  }
  if (target->empty() || kf.cloud_base->empty()) return false;

  // Re-voxel ONLY the assembled TARGET submap before ICP. The concatenated
  // keyframes overlap unevenly, and point-to-point ICP weights correspondences by
  // density, so an un-normalized submap biases the alignment toward the overlap
  // zones; the references downsample the submap here. The SOURCE is a single
  // keyframe (already uniform at keyframe_voxel), so it is NOT downsampled -
  // re-voxeling a single scan to a coarser leaf only thins it and starves ICP of
  // correspondences (seen as "Not enough correspondences" on drifted loops).
  if (cfg_.loop_icp_voxel > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(cfg_.loop_icp_voxel, cfg_.loop_icp_voxel, cfg_.loop_icp_voxel);
    Cloud::Ptr tv(new Cloud());
    vg.setInputCloud(target);
    vg.filter(*tv);
    target = tv;
  }

  // ICP: align the query keyframe (source) into the candidate frame. Initial
  // guess T(cand<-id) from the current estimates.
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(cfg_.icp_max_corr_dist);
  icp.setMaximumIterations(cfg_.icp_max_iter);
  icp.setInputSource(kf.cloud_base);
  icp.setInputTarget(target);
  const Eigen::Isometry3d guess = x_cand.inverse() * graph_.optimizedPose(id);
  Cloud aligned;
  icp.align(aligned, guess.matrix().cast<float>());
  if (!icp.hasConverged() || icp.getFitnessScore() > cfg_.icp_fitness_thresh) {
    return false;
  }

  // The ICP result is T(cand<-id): the loop measurement.
  const Eigen::Matrix4d m = icp.getFinalTransformation().cast<double>();
  Eigen::Isometry3d rel_loop = Eigen::Isometry3d::Identity();
  rel_loop.linear() = m.block<3, 3>(0, 0);
  rel_loop.translation() = m.block<3, 1>(0, 3);

  // Front-end consistency gate. The front end is accurate, so a TRUE loop's ICP
  // measurement must agree with the odometry-chained relative pose
  // T_odom(cand<-id) = P_cand^-1 * P_id within accumulated drift; a disagreement
  // beyond (floor + rate * traveled path) is a false/degenerate match (corridor
  // aliasing, the Scan Context yaw-invariance trap) and is rejected here so it
  // never enters the graph. Cheap insurance on top of the robust kernel.
  const Eigen::Isometry3d odom_rel =
      keyframes_[cand].odom_pose.inverse() * keyframes_[id].odom_pose;
  const Eigen::Isometry3d disc = rel_loop.inverse() * odom_rel;
  const double dt = disc.translation().norm();
  const double dr = Eigen::AngleAxisd(disc.rotation()).angle();
  double path_len = 0.0;
  for (std::uint64_t k = cand; k < id; ++k) {
    path_len += (keyframes_[k].odom_pose.inverse() *
                 keyframes_[k + 1].odom_pose).translation().norm();
  }
  if (dt > cfg_.loop_max_dt + cfg_.loop_max_dt_rate * path_len ||
      dr > cfg_.loop_max_dr + cfg_.loop_max_dr_rate * path_len) {
    return false;
  }

  graph_.addLoop(cand, id, rel_loop, cfg_.loop_rot_sigma, cfg_.loop_trans_sigma,
                 cfg_.loop_robust_c);
  loop_edges_.push_back(LoopEdge{cand, id, rel_loop});
  graph_.update(cfg_.loop_extra_iters);
  updateMapToOdom();
  return true;
}

std::vector<Backend::KeyframeView> Backend::snapshotKeyframes() const {
  std::vector<KeyframeView> out;
  out.reserve(keyframes_.size());
  for (const Keyframe& kf : keyframes_) {
    out.push_back(KeyframeView{graph_.optimizedPose(kf.id), kf.cloud_base});
  }
  return out;
}

Backend::Cloud::Ptr Backend::assembleFromSnapshot(
    const std::vector<KeyframeView>& kfs, double map_voxel) {
  Cloud::Ptr map(new Cloud());
  for (const KeyframeView& kf : kfs) {
    Cloud tmp;
    pcl::transformPointCloud(*kf.cloud, tmp, kf.pose.matrix().cast<float>());
    *map += tmp;
  }
  if (map_voxel > 0.0 && !map->empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(map);
    vg.setLeafSize(map_voxel, map_voxel, map_voxel);
    Cloud::Ptr ds(new Cloud());
    vg.filter(*ds);
    map = ds;
  }
  return map;
}

Backend::Cloud::Ptr Backend::assembleMap() const {
  return assembleFromSnapshot(snapshotKeyframes(), cfg_.map_voxel);
}

std::vector<Eigen::Isometry3d> Backend::optimizedPoses() const {
  std::vector<Eigen::Isometry3d> out;
  out.reserve(keyframes_.size());
  for (const Keyframe& kf : keyframes_) {
    out.push_back(graph_.optimizedPose(kf.id));
  }
  return out;
}

Backend::Cloud::Ptr Backend::toBaseCloud(const Eigen::Isometry3d& odom_pose,
                                         const Cloud& cloud_world) const {
  Cloud::Ptr c(new Cloud());
  pcl::transformPointCloud(cloud_world, *c,
                           odom_pose.inverse().matrix().cast<float>());
  if (cfg_.keyframe_voxel > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(c);
    vg.setLeafSize(cfg_.keyframe_voxel, cfg_.keyframe_voxel, cfg_.keyframe_voxel);
    Cloud::Ptr ds(new Cloud());
    vg.filter(*ds);
    c = ds;
  }
  return c;
}

Backend::RelocResult Backend::relocalizeGlobal(
    const Eigen::Isometry3d& odom_pose, const Cloud& cloud_world) const {
  RelocResult res;
  if (keyframes_.empty()) return res;

  const Cloud::Ptr c_now = toBaseCloud(odom_pose, cloud_world);
  if (c_now->empty()) return res;

  // Live descriptor in the same yaw-only gravity-aligned frame as the keyframes,
  // then a short list of SC candidates (SC only proposes; geometry decides).
  const ScanContextDB::Descriptor sc = scdb_.make(ScanContextDB::toScanContextFrame(
      toEigenPoints(*c_now), odom_pose.rotation()));
  const std::vector<ScanContextDB::QueryResult> cands =
      scdb_.queryCandidates(sc, cfg_.sc_knn, cfg_.reloc_num_candidates);
  if (cands.empty()) return res;

  const long n = static_cast<long>(keyframes_.size());
  double best_inlier = 0.0;
  Eigen::Isometry3d best_x_now = Eigen::Isometry3d::Identity();

  // ICP-verify each candidate; accept the alignment with the highest inlier
  // ratio (geometric overlap), not the lowest fitness. The SC column shift gives
  // the relative yaw up to a sign in our frame convention, so try both.
  for (const ScanContextDB::QueryResult& q : cands) {
    const std::uint64_t cand = q.match_id;
    const Eigen::Isometry3d x_cand = graph_.optimizedPose(cand);
    Cloud::Ptr target(new Cloud());
    for (long k = static_cast<long>(cand) - cfg_.icp_submap_neighbors;
         k <= static_cast<long>(cand) + cfg_.icp_submap_neighbors; ++k) {
      if (k < 0 || k >= n) continue;
      const Eigen::Isometry3d rel =
          x_cand.inverse() * graph_.optimizedPose(static_cast<std::uint64_t>(k));
      Cloud tmp;
      pcl::transformPointCloud(*keyframes_[k].cloud_base, tmp,
                               rel.matrix().cast<float>());
      *target += tmp;
    }
    if (target->empty()) continue;

    pcl::KdTreeFLANN<pcl::PointXYZI> target_kd;
    target_kd.setInputCloud(target);
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(cfg_.icp_max_corr_dist);
    icp.setMaximumIterations(cfg_.icp_max_iter);
    icp.setInputSource(c_now);
    icp.setInputTarget(target);
    for (const double yaw : {q.yaw, -q.yaw}) {
      Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
      guess.linear() =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      Cloud aligned;
      icp.align(aligned, guess.matrix().cast<float>());
      if (!icp.hasConverged()) continue;
      const double inl = inlierRatio(aligned, target_kd, cfg_.reloc_inlier_dist);
      if (inl > best_inlier) {
        best_inlier = inl;
        const Eigen::Matrix4d m = icp.getFinalTransformation().cast<double>();
        Eigen::Isometry3d cand_now = Eigen::Isometry3d::Identity();
        cand_now.linear() = m.block<3, 3>(0, 0);
        cand_now.translation() = m.block<3, 1>(0, 3);
        best_x_now = x_cand * cand_now;  // T(map<-base_now)
        res.match_id = cand;
        res.sc_distance = q.distance;
        res.inlier_ratio = inl;
        res.fitness = icp.getFitnessScore();
      }
    }
  }

  if (best_inlier >= cfg_.reloc_min_inlier) {
    res.found = true;
    res.map_to_odom = best_x_now * odom_pose.inverse();
  }
  return res;  // diagnostics (match_id/sc_distance/inlier_ratio/fitness) filled either way
}

Backend::RelocResult Backend::relocalizeLocal(
    const Eigen::Isometry3d& odom_pose, const Cloud& cloud_world,
    const Eigen::Isometry3d& predicted_map_pose, double radius) const {
  RelocResult res;
  if (keyframes_.empty()) return res;

  const Cloud::Ptr c_now = toBaseCloud(odom_pose, cloud_world);
  if (c_now->empty()) return res;

  // Local submap (keyframes within `radius` of the prediction) expressed in the
  // predicted base frame, so ICP runs on small coordinates (good float32) with
  // an identity initial guess.
  Cloud::Ptr target(new Cloud());
  const Eigen::Isometry3d pred_inv = predicted_map_pose.inverse();
  const Eigen::Vector3d center = predicted_map_pose.translation();
  for (const Keyframe& kf : keyframes_) {
    const Eigen::Isometry3d x_k = graph_.optimizedPose(kf.id);
    if ((x_k.translation() - center).norm() > radius) continue;
    const Eigen::Isometry3d rel = pred_inv * x_k;
    Cloud tmp;
    pcl::transformPointCloud(*kf.cloud_base, tmp, rel.matrix().cast<float>());
    *target += tmp;
  }
  if (target->empty()) return res;

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(cfg_.icp_max_corr_dist);
  icp.setMaximumIterations(cfg_.icp_max_iter);
  icp.setInputSource(c_now);
  icp.setInputTarget(target);
  Cloud aligned;
  icp.align(aligned);  // identity init: the source is already near the prediction
  if (!icp.hasConverged()) return res;
  pcl::KdTreeFLANN<pcl::PointXYZI> target_kd;
  target_kd.setInputCloud(target);
  res.inlier_ratio = inlierRatio(aligned, target_kd, cfg_.reloc_inlier_dist);
  res.fitness = icp.getFitnessScore();
  if (res.inlier_ratio < cfg_.reloc_min_inlier) return res;
  const Eigen::Matrix4d m = icp.getFinalTransformation().cast<double>();
  Eigen::Isometry3d pred_now = Eigen::Isometry3d::Identity();
  pred_now.linear() = m.block<3, 3>(0, 0);
  pred_now.translation() = m.block<3, 1>(0, 3);

  const Eigen::Isometry3d x_now = predicted_map_pose * pred_now;  // T(map<-now)
  res.found = true;
  res.map_to_odom = x_now * odom_pose.inverse();
  return res;
}

std::vector<Backend::KeyframeView> Backend::snapshotKeyframesNear(
    const Eigen::Vector3d& center, double radius) const {
  std::vector<KeyframeView> out;
  const double r2 = radius * radius;
  for (const Keyframe& kf : keyframes_) {
    const Eigen::Isometry3d x = graph_.optimizedPose(kf.id);
    if ((x.translation() - center).squaredNorm() > r2) continue;
    out.push_back(KeyframeView{x, kf.cloud_base});
  }
  return out;
}

Backend::RelocResult Backend::trackAgainstSubmap(
    const Eigen::Isometry3d& odom_pose, const Cloud& cloud_world,
    const Eigen::Isometry3d& predicted_map_pose,
    const Cloud::ConstPtr& submap_map) const {
  RelocResult res;
  if (!submap_map || submap_map->empty()) return res;
  const Cloud::Ptr c_now = toBaseCloud(odom_pose, cloud_world);
  if (c_now->empty()) return res;

  // ICP the live base_link cloud into the map-frame submap, seeded by the
  // predicted map pose; the result is T(map<-base_now).
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(cfg_.icp_max_corr_dist);
  icp.setMaximumIterations(cfg_.icp_max_iter);
  icp.setInputSource(c_now);
  icp.setInputTarget(submap_map);
  Cloud aligned;
  icp.align(aligned, predicted_map_pose.matrix().cast<float>());
  if (!icp.hasConverged()) return res;
  pcl::KdTreeFLANN<pcl::PointXYZI> target_kd;
  target_kd.setInputCloud(submap_map);
  res.inlier_ratio = inlierRatio(aligned, target_kd, cfg_.reloc_inlier_dist);
  res.fitness = icp.getFitnessScore();
  if (res.inlier_ratio < cfg_.reloc_min_inlier) return res;  // reject -> lost streak
  const Eigen::Matrix4d m = icp.getFinalTransformation().cast<double>();
  Eigen::Isometry3d x_now = Eigen::Isometry3d::Identity();
  x_now.linear() = m.block<3, 3>(0, 0);
  x_now.translation() = m.block<3, 1>(0, 3);
  res.found = true;
  res.map_to_odom = x_now * odom_pose.inverse();
  // Registration observability at the converged pose: the node uses it to drop
  // the correction along unconstrained DOFs (degeneracy projection).
  res.obs_info = pointToPlaneInfo(*c_now, x_now, cfg_.track_normal_k);
  return res;
}

bool Backend::saveMap(const std::string& dir, std::string* message) const {
  auto fail = [&](const std::string& m) {
    if (message) *message = m;
    return false;
  };
  if (keyframes_.empty()) return fail("no keyframes to save");

  try {
    const fs::path target(dir);
    fs::path parent = target.parent_path();
    if (parent.empty()) parent = ".";
    const fs::path tmp = parent / (target.filename().string() + ".tmp");
    std::error_code ec;
    fs::remove_all(tmp, ec);
    fs::create_directories(tmp / kKeyframesDir);

    {  // manifest (fixed-schema key:value YAML, ASCII, no yaml-cpp dependency)
      std::ofstream os(tmp / kManifestName);
      if (!os) return fail("cannot write manifest");
      os << "# g1_slam_backend map. Inspectable and re-optimizable.\n";
      os << "format_version: " << kMapFormatVersion << "\n";
      os << "num_keyframes: " << keyframes_.size() << "\n";
      os << "keyframe_voxel: " << cfg_.keyframe_voxel << "\n";
      os << "sc_num_rings: " << cfg_.sc.num_rings << "\n";
      os << "sc_num_sectors: " << cfg_.sc.num_sectors << "\n";
      os << "sc_max_range: " << cfg_.sc.max_range << "\n";
      os << "sc_min_range: " << cfg_.sc.min_range << "\n";
      os << "sc_height_offset: " << cfg_.sc.height_offset << "\n";
    }

    {  // pose graph (g2o): vertices = optimized poses; edges = odom + loops.
      std::ofstream os(tmp / kGraphName);
      if (!os) return fail("cannot write pose graph");
      os << std::setprecision(12);
      for (const Keyframe& kf : keyframes_) {
        os << "VERTEX_SE3:QUAT " << kf.id << ' ';
        writeQuatPose(os, graph_.optimizedPose(kf.id));
        os << '\n';
      }
      for (std::size_t i = 1; i < keyframes_.size(); ++i) {
        const Eigen::Isometry3d rel =
            keyframes_[i - 1].odom_pose.inverse() * keyframes_[i].odom_pose;
        os << "EDGE_SE3:QUAT " << keyframes_[i - 1].id << ' '
           << keyframes_[i].id << ' ';
        writeQuatPose(os, rel);
        writeInfoUpperTri(os, cfg_.odom_rot_sigma, cfg_.odom_trans_sigma);
        os << '\n';
      }
      for (const LoopEdge& e : loop_edges_) {
        os << "EDGE_SE3:QUAT " << e.from << ' ' << e.to << ' ';
        writeQuatPose(os, e.rel);
        writeInfoUpperTri(os, cfg_.loop_rot_sigma, cfg_.loop_trans_sigma);
        os << '\n';
      }
    }

    {  // Scan Context descriptors (count, rings, sectors, then id + row-major f64)
      std::ofstream os(tmp / kScName, std::ios::binary);
      if (!os) return fail("cannot write scan context");
      const std::uint64_t n = keyframes_.size();
      const std::int32_t rows = cfg_.sc.num_rings;
      const std::int32_t cols = cfg_.sc.num_sectors;
      os.write(reinterpret_cast<const char*>(&n), sizeof(n));
      os.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
      os.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
      for (const Keyframe& kf : keyframes_) {
        os.write(reinterpret_cast<const char*>(&kf.id), sizeof(kf.id));
        for (int r = 0; r < rows; ++r) {
          for (int c = 0; c < cols; ++c) {
            const double v = kf.sc(r, c);
            os.write(reinterpret_cast<const char*>(&v), sizeof(v));
          }
        }
      }
    }

    for (const Keyframe& kf : keyframes_) {  // per-keyframe base_link clouds
      const fs::path pcd = tmp / kKeyframesDir / keyframePcdName(kf.id);
      if (pcl::io::savePCDFileBinary(pcd.string(), *kf.cloud_base) != 0) {
        return fail("cannot write " + pcd.string());
      }
    }

    // Swap into place: move any existing target aside, rename tmp over it.
    const fs::path backup = parent / (target.filename().string() + ".bak");
    fs::remove_all(backup, ec);
    if (fs::exists(target)) fs::rename(target, backup);
    fs::rename(tmp, target);
    fs::remove_all(backup, ec);
  } catch (const std::exception& e) {
    return fail(std::string("exception: ") + e.what());
  }

  if (message) {
    *message = "saved " + std::to_string(keyframes_.size()) + " keyframes";
  }
  return true;
}

bool Backend::loadMap(const std::string& dir, std::string* message) {
  auto fail = [&](const std::string& m) {
    if (message) *message = m;
    return false;
  };

  try {
    const fs::path root(dir);
    if (!fs::is_directory(root)) return fail("not a directory: " + dir);

    std::map<std::string, std::string> kv;
    if (!parseManifest(root / kManifestName, &kv)) {
      return fail("cannot read manifest");
    }
    auto it_n = kv.find("num_keyframes");
    if (it_n == kv.end()) return fail("manifest missing num_keyframes");
    const std::size_t n = static_cast<std::size_t>(std::stoull(it_n->second));
    if (n == 0) return fail("manifest num_keyframes = 0");

    // Adopt the map's Scan Context config (descriptors were built with it).
    ScanContextConfig sc = cfg_.sc;
    auto getd = [&](const char* k, double* out) {
      auto it = kv.find(k);
      if (it != kv.end()) *out = std::stod(it->second);
    };
    auto geti = [&](const char* k, int* out) {
      auto it = kv.find(k);
      if (it != kv.end()) *out = std::stoi(it->second);
    };
    geti("sc_num_rings", &sc.num_rings);
    geti("sc_num_sectors", &sc.num_sectors);
    getd("sc_max_range", &sc.max_range);
    getd("sc_min_range", &sc.min_range);
    getd("sc_height_offset", &sc.height_offset);
    double keyframe_voxel = cfg_.keyframe_voxel;
    getd("keyframe_voxel", &keyframe_voxel);

    // Scan Context descriptors.
    std::map<std::uint64_t, ScanContextDB::Descriptor> descs;
    {
      std::ifstream is(root / kScName, std::ios::binary);
      if (!is) return fail("cannot read scan context");
      std::uint64_t cnt = 0;
      std::int32_t rows = 0, cols = 0;
      is.read(reinterpret_cast<char*>(&cnt), sizeof(cnt));
      is.read(reinterpret_cast<char*>(&rows), sizeof(rows));
      is.read(reinterpret_cast<char*>(&cols), sizeof(cols));
      if (!is) return fail("corrupt scan context header");
      if (rows != sc.num_rings || cols != sc.num_sectors) {
        return fail("scan context dims do not match manifest");
      }
      if (cnt != n) return fail("scan context count != num_keyframes");
      for (std::uint64_t i = 0; i < cnt; ++i) {
        std::uint64_t id = 0;
        is.read(reinterpret_cast<char*>(&id), sizeof(id));
        ScanContextDB::Descriptor d(rows, cols);
        for (int r = 0; r < rows; ++r) {
          for (int c = 0; c < cols; ++c) {
            double v = 0.0;
            is.read(reinterpret_cast<char*>(&v), sizeof(v));
            d(r, c) = v;
          }
        }
        if (!is) return fail("corrupt scan context body");
        descs[id] = d;
      }
    }

    // Pose graph (g2o): vertices + edges (odom = consecutive ids, else loop).
    std::map<std::uint64_t, Eigen::Isometry3d> verts;
    struct Edge {
      std::uint64_t from, to;
      Eigen::Isometry3d rel;
    };
    std::vector<Edge> odom_edges, loop_in;
    {
      std::ifstream is(root / kGraphName);
      if (!is) return fail("cannot read pose graph");
      std::string tag;
      while (is >> tag) {
        if (tag == "VERTEX_SE3:QUAT") {
          std::uint64_t id = 0;
          is >> id;
          verts[id] = readQuatPose(is);
        } else if (tag == "EDGE_SE3:QUAT") {
          std::uint64_t from = 0, to = 0;
          is >> from >> to;
          const Eigen::Isometry3d rel = readQuatPose(is);
          double dump = 0.0;
          for (int i = 0; i < 21 && (is >> dump); ++i) {
          }  // skip the 6x6 upper-tri info block (noise comes from config on load)
          if (to == from + 1) {
            odom_edges.push_back({from, to, rel});
          } else {
            loop_in.push_back({from, to, rel});
          }
        } else {
          std::getline(is, tag);  // skip an unrecognized line
        }
      }
    }
    if (verts.size() != n) return fail("g2o vertex count != num_keyframes");

    // Commit: replace any current state.
    cfg_.sc = sc;
    cfg_.keyframe_voxel = keyframe_voxel;
    scdb_ = ScanContextDB(sc);
    graph_.clear();
    keyframes_.clear();
    pending_loops_.clear();
    loop_edges_.clear();
    map_to_odom_ = Eigen::Isometry3d::Identity();

    // Keyframes (ids must be contiguous 0..n-1): clouds + descriptors.
    for (std::uint64_t id = 0; id < n; ++id) {
      if (verts.find(id) == verts.end()) return fail("g2o missing a vertex id");
      if (descs.find(id) == descs.end()) return fail("missing a descriptor id");
      Cloud::Ptr cloud(new Cloud());
      const fs::path pcd = root / kKeyframesDir / keyframePcdName(id);
      if (pcl::io::loadPCDFile(pcd.string(), *cloud) != 0) {
        return fail("cannot read " + pcd.string());
      }
      Keyframe kf;
      kf.id = id;
      kf.stamp = 0.0;
      kf.odom_pose = verts[id];  // placeholder; unused in localization mode
      kf.cloud_base = cloud;
      kf.sc = descs[id];
      keyframes_.push_back(kf);
      scdb_.add(id, kf.sc);
    }

    // Rebuild the graph: prior on id 0, odom edges in order, then loop edges.
    graph_.addPrior(0, verts[0], cfg_.prior_rot_sigma, cfg_.prior_trans_sigma);
    std::sort(odom_edges.begin(), odom_edges.end(),
              [](const Edge& a, const Edge& b) { return a.to < b.to; });
    for (const Edge& e : odom_edges) {
      graph_.addOdometry(e.from, e.to, e.rel, cfg_.odom_rot_sigma,
                         cfg_.odom_trans_sigma);
    }
    for (const Edge& e : loop_in) {
      graph_.addLoop(e.from, e.to, e.rel, cfg_.loop_rot_sigma,
                     cfg_.loop_trans_sigma, cfg_.loop_robust_c);
      loop_edges_.push_back(LoopEdge{e.from, e.to, e.rel});
    }
    graph_.update(cfg_.loop_extra_iters);
  } catch (const std::exception& e) {
    return fail(std::string("exception: ") + e.what());
  }

  if (message) {
    *message = "loaded " + std::to_string(keyframes_.size()) + " keyframes";
  }
  return true;
}

void Backend::reset() {
  graph_.clear();
  scdb_ = ScanContextDB(cfg_.sc);
  keyframes_.clear();
  pending_loops_.clear();
  loop_edges_.clear();
  map_to_odom_ = Eigen::Isometry3d::Identity();
}

void Backend::beginIncremental() {
  // map->odom = T (relocalization lock). Each keyframe's optimized map pose is
  // X_id; in the live odom frame its base_link sits at P_id = T^-1 * X_id. Setting
  // odom_pose to that makes the next bridge factor (kf_last.odom_pose^-1 * P_new)
  // place the new keyframe at T * P_new in the map frame, and keeps updateMapToOdom
  // (X_last * P_last^-1) consistent at T.
  const Eigen::Isometry3d odom_to_map = map_to_odom_.inverse();
  for (Keyframe& kf : keyframes_) {
    kf.odom_pose = odom_to_map * graph_.optimizedPose(kf.id);
  }
}

}  // namespace g1_slam_backend
