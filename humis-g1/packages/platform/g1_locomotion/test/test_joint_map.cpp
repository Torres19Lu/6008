#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "g1_locomotion/joint_map.hpp"

using namespace g1_locomotion;

TEST(JointMap, MapsIndicesToPositions) {
  std::array<double, kLowStateMotorCount> q{};
  for (std::size_t i = 0; i < q.size(); ++i) q[i] = static_cast<double>(i);
  std::vector<JointMapEntry> m = {
      {"left_hip_pitch_joint", 0},
      {"waist_yaw_joint", 12},
      {"right_wrist_roll_joint", 26},
  };
  JointStateData d = buildJointState(q, m);
  ASSERT_EQ(d.names.size(), 3u);
  ASSERT_EQ(d.positions.size(), 3u);
  EXPECT_EQ(d.names[0], "left_hip_pitch_joint");
  EXPECT_DOUBLE_EQ(d.positions[0], 0.0);
  EXPECT_EQ(d.names[1], "waist_yaw_joint");
  EXPECT_DOUBLE_EQ(d.positions[1], 12.0);
  EXPECT_EQ(d.names[2], "right_wrist_roll_joint");
  EXPECT_DOUBLE_EQ(d.positions[2], 26.0);
}

TEST(JointMap, SkipsOutOfRange) {
  std::array<double, kLowStateMotorCount> q{};
  q[5] = 1.23;
  std::vector<JointMapEntry> m = {{"bad", -1}, {"also_bad", 99}, {"ok", 5}};
  JointStateData d = buildJointState(q, m);
  ASSERT_EQ(d.names.size(), 1u);
  EXPECT_EQ(d.names[0], "ok");
  EXPECT_DOUBLE_EQ(d.positions[0], 1.23);
}

TEST(JointMap, PreservesMapOrder) {
  std::array<double, kLowStateMotorCount> q{};
  q[0] = 0.0;
  q[1] = 1.0;
  q[2] = 2.0;
  std::vector<JointMapEntry> m = {{"a", 2}, {"b", 1}, {"c", 0}};
  JointStateData d = buildJointState(q, m);
  ASSERT_EQ(d.names.size(), 3u);
  EXPECT_EQ(d.names[0], "a");
  EXPECT_DOUBLE_EQ(d.positions[0], 2.0);
  EXPECT_EQ(d.names[2], "c");
  EXPECT_DOUBLE_EQ(d.positions[2], 0.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
