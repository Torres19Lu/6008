#include "g1_slam_backend/pose_graph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace g1_slam_backend {
namespace {

gtsam::Pose3 toGtsam(const Eigen::Isometry3d& T) {
  return gtsam::Pose3(gtsam::Rot3(Eigen::Matrix3d(T.rotation())),
                      gtsam::Point3(Eigen::Vector3d(T.translation())));
}

Eigen::Isometry3d toEigen(const gtsam::Pose3& p) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = p.rotation().matrix();
  T.translation() = p.translation();
  return T;
}

// GTSAM Pose3 tangent ordering is [rotation(3), translation(3)].
gtsam::SharedNoiseModel diagNoise(double rot_sigma, double trans_sigma) {
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << rot_sigma, rot_sigma, rot_sigma,
                           trans_sigma, trans_sigma, trans_sigma)
          .finished());
}

// Loop noise, optionally wrapped in a Cauchy robust kernel (cauchy_c > 0). A
// robust kernel down-weights an outlier loop by its residual, so a false loop
// that passes the acceptance gate cannot drag the whole map.
gtsam::SharedNoiseModel loopNoise(double rot_sigma, double trans_sigma,
                                  double cauchy_c) {
  gtsam::SharedNoiseModel base = diagNoise(rot_sigma, trans_sigma);
  if (cauchy_c <= 0.0) return base;
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(cauchy_c), base);
}

// iSAM2 tuning. The GTSAM defaults (relinearizeSkip=10) defer relinearization,
// so a loop correction stays partly on its pre-loop linearization for several
// updates ("the loop does not fully take"). relinearizeSkip=1 relinearizes every
// update, so a loop propagates immediately. threshold=0.1 is a moderate threshold
// (a tighter 0.01 increases accuracy at higher CPU cost per update).
gtsam::ISAM2Params isamParams() {
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = 0.1;
  p.relinearizeSkip = 1;
  return p;
}

}  // namespace

struct PoseGraph::Impl {
  gtsam::ISAM2 isam{isamParams()};
  gtsam::NonlinearFactorGraph pending_graph;
  gtsam::Values pending_values;
  std::map<Key, Eigen::Isometry3d> current;  // latest estimate (init or optimized)
};

PoseGraph::PoseGraph() : impl_(std::make_unique<Impl>()) {}
PoseGraph::~PoseGraph() = default;

void PoseGraph::addPrior(Key key, const Eigen::Isometry3d& pose,
                         double rot_sigma, double trans_sigma) {
  impl_->pending_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key, toGtsam(pose), diagNoise(rot_sigma, trans_sigma));
  if (!impl_->pending_values.exists(key)) {
    impl_->pending_values.insert(key, toGtsam(pose));
  }
  impl_->current[key] = pose;
}

void PoseGraph::addOdometry(Key from, Key to, const Eigen::Isometry3d& rel_pose,
                            double rot_sigma, double trans_sigma) {
  impl_->pending_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      from, to, toGtsam(rel_pose), diagNoise(rot_sigma, trans_sigma));
  const Eigen::Isometry3d init = impl_->current.at(from) * rel_pose;
  impl_->pending_values.insert(to, toGtsam(init));
  impl_->current[to] = init;
}

void PoseGraph::addLoop(Key from, Key to, const Eigen::Isometry3d& rel_pose,
                        double rot_sigma, double trans_sigma, double cauchy_c) {
  impl_->pending_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      from, to, toGtsam(rel_pose), loopNoise(rot_sigma, trans_sigma, cauchy_c));
}

void PoseGraph::update(int extra_iters) {
  impl_->isam.update(impl_->pending_graph, impl_->pending_values);
  for (int i = 0; i < extra_iters; ++i) {
    impl_->isam.update();
  }
  impl_->pending_graph = gtsam::NonlinearFactorGraph();
  impl_->pending_values.clear();

  const gtsam::Values est = impl_->isam.calculateEstimate();
  for (const gtsam::Key key : est.keys()) {
    impl_->current[key] = toEigen(est.at<gtsam::Pose3>(key));
  }
}

void PoseGraph::clear() { impl_ = std::make_unique<Impl>(); }

Eigen::Isometry3d PoseGraph::optimizedPose(Key key) const {
  return impl_->current.at(key);
}

std::map<PoseGraph::Key, Eigen::Isometry3d> PoseGraph::allPoses() const {
  return impl_->current;
}

bool PoseGraph::exists(Key key) const {
  return impl_->current.find(key) != impl_->current.end();
}

std::size_t PoseGraph::size() const { return impl_->current.size(); }

}  // namespace g1_slam_backend
