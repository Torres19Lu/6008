// Keyframe overlay filter tests: suppressed keyframe dropped, deleted voxel removes
// nearby points, point outside a crop dropped, empty overlay passes through.

#include <gtest/gtest.h>

#include <array>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "g1_map_manager/core/overlay.h"
#include "g1_map_manager/keyframe_filter.h"

using namespace g1_map_manager;

namespace {

g1_msgs::KeyframeCloud makeKf(std::uint64_t id,
                              const std::vector<std::array<float, 3>>& pts) {
  g1_msgs::KeyframeCloud kf;
  kf.id = id;
  pcl::PointCloud<pcl::PointXYZI> c;
  for (const auto& p : pts) {
    pcl::PointXYZI pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    pt.intensity = 1.0f;
    c.push_back(pt);
  }
  pcl::toROSMsg(c, kf.cloud);
  return kf;
}

std::size_t cloudSize(const g1_msgs::KeyframeCloud& kf) {
  pcl::PointCloud<pcl::PointXYZI> c;
  pcl::fromROSMsg(kf.cloud, c);
  return c.size();
}

}  // namespace

TEST(KeyframeFilter, SuppressedKeyframeDropped) {
  g1_msgs::KeyframeCloudArray in;
  in.keyframes.push_back(makeKf(0, {{0, 0, 0}, {1, 1, 0}}));
  in.keyframes.push_back(makeKf(1, {{2, 2, 0}}));
  Overlay ov;
  ov.suppressed_keyframes = {1};
  const auto out = filterKeyframes(in, ov, 0.2);
  ASSERT_EQ(out.keyframes.size(), 1u);
  EXPECT_EQ(out.keyframes[0].id, 0u);
}

TEST(KeyframeFilter, DeletedVoxelRemovesNearbyPoints) {
  g1_msgs::KeyframeCloudArray in;
  in.keyframes.push_back(makeKf(0, {{0, 0, 0}, {5, 5, 0}}));
  Overlay ov;
  ov.deleted_voxels = {{0.0, 0.0, 0.0}};
  const auto out = filterKeyframes(in, ov, 0.2);
  ASSERT_EQ(out.keyframes.size(), 1u);
  EXPECT_EQ(cloudSize(out.keyframes[0]), 1u);  // only (5,5,0) survives
}

TEST(KeyframeFilter, OutsideCropDropped) {
  g1_msgs::KeyframeCloudArray in;
  in.keyframes.push_back(makeKf(0, {{1, 1, 0}, {5, 5, 0}}));
  Overlay ov;
  ov.crops = {{{0, 0}, {2, 0}, {2, 2}, {0, 2}}};  // keep-inside unit-ish square
  const auto out = filterKeyframes(in, ov, 0.2);
  ASSERT_EQ(out.keyframes.size(), 1u);
  EXPECT_EQ(cloudSize(out.keyframes[0]), 1u);  // (1,1) inside kept, (5,5) dropped
}

TEST(KeyframeFilter, EmptyOverlayPassesThrough) {
  g1_msgs::KeyframeCloudArray in;
  in.keyframes.push_back(makeKf(0, {{0, 0, 0}, {1, 1, 0}}));
  in.keyframes.push_back(makeKf(1, {{2, 2, 0}}));
  const auto out = filterKeyframes(in, Overlay{}, 0.2);
  ASSERT_EQ(out.keyframes.size(), 2u);
  EXPECT_EQ(cloudSize(out.keyframes[0]), 2u);
  EXPECT_EQ(cloudSize(out.keyframes[1]), 1u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
