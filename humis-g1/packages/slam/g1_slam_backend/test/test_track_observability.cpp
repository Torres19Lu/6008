// Observability (point-to-plane Hessian) computed by the localization tracking
// ICP. The TRACKING correction must know which DOFs the live scan actually
// constrains: a scene dominated by ONE planar surface (a table / monitor the
// robot faces) leaves the two in-plane translations unobservable, so the raw ICP
// can slide there. trackAgainstSubmap fills RelocResult.obs_info; this test
// asserts the geometry is reflected: a single vertical plane yields a rank-
// deficient translation block (near-zero eigenvalues), a closed box does not.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g1_slam_backend/backend.h"

using g1_slam_backend::Backend;
using g1_slam_backend::BackendConfig;

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

void addPoint(Cloud& c, double x, double y, double z) {
  pcl::PointXYZI p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = static_cast<float>(z);
  p.intensity = 1.0f;
  c.push_back(p);
}

// A single vertical plane at x = d (normal along x), spanning y,z. The only
// constrained translation is x; y and z are free slides.
Cloud::Ptr verticalPlane(double d) {
  Cloud::Ptr c(new Cloud());
  for (double y = -3.0; y <= 3.0; y += 0.1)
    for (double z = 0.0; z <= 2.0; z += 0.1) addPoint(*c, d, y, z);
  return c;
}

// A closed box interior (4 walls + floor): normals span x, y, z -> all three
// translations observable.
Cloud::Ptr boxScene() {
  Cloud::Ptr c(new Cloud());
  for (double y = -3.0; y <= 3.0; y += 0.15)
    for (double z = 0.0; z <= 2.0; z += 0.15) {
      addPoint(*c, 4.0, y, z);   // +x wall
      addPoint(*c, -4.0, y, z);  // -x wall
    }
  for (double x = -4.0; x <= 4.0; x += 0.15)
    for (double z = 0.0; z <= 2.0; z += 0.15) {
      addPoint(*c, x, 3.0, z);   // +y wall
      addPoint(*c, x, -3.0, z);  // -y wall
    }
  for (double x = -4.0; x <= 4.0; x += 0.15)
    for (double y = -3.0; y <= 3.0; y += 0.15) addPoint(*c, x, y, 0.0);  // floor
  return c;
}

// Smallest eigenvalue of the 3x3 translation block of the observability info.
double minTransEig(const Backend::RelocResult& r) {
  const Eigen::Matrix3d T = r.obs_info.topLeftCorner<3, 3>();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(T);
  return es.eigenvalues()(0);
}

BackendConfig cfg() {
  BackendConfig c;
  c.keyframe_voxel = 0.0;  // keep the synthetic clouds intact
  c.reloc_inlier_dist = 0.3;
  c.reloc_min_inlier = 0.5;
  c.icp_max_corr_dist = 1.0;
  return c;
}

}  // namespace

// A single dominant plane: the in-plane translations are unobservable, so the
// translation-info block is rank-deficient (>=2 near-zero eigenvalues).
TEST(TrackObservability, DominantPlaneIsDegenerate) {
  Backend backend(cfg());
  const Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d pred = Eigen::Isometry3d::Identity();
  Cloud::Ptr scene = verticalPlane(5.0);

  const Backend::RelocResult r =
      backend.trackAgainstSubmap(odom, *scene, pred, scene);

  ASSERT_TRUE(r.found);
  EXPECT_LT(minTransEig(r), 0.05);  // a free slide direction exists
}

// A closed box constrains all three translations: no near-zero eigenvalue.
TEST(TrackObservability, BoxIsWellConditioned) {
  Backend backend(cfg());
  const Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d pred = Eigen::Isometry3d::Identity();
  Cloud::Ptr scene = boxScene();

  const Backend::RelocResult r =
      backend.trackAgainstSubmap(odom, *scene, pred, scene);

  ASSERT_TRUE(r.found);
  // obs_info was actually computed (not left at the Identity default).
  EXPECT_FALSE(r.obs_info.isApprox(Eigen::Matrix<double, 6, 6>::Identity()));
  EXPECT_GT(minTransEig(r), 0.10);  // every translation is constrained
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
