// Unit tests for cropCentered (local_window.h / local_window.cpp).
// All tests are ROS-free; link g1_costmap_core only.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "g1_costmap/core/cost_values.h"
#include "g1_costmap/core/costmap_grid.h"
#include "g1_costmap/core/local_window.h"

using g1_costmap::CostmapGrid;
using g1_costmap::NO_INFORMATION;
using g1_costmap::cropCentered;

// Build a src grid filled with unique values (cell value = mx * 100 + my) so we
// can verify exactly which src cells ended up in the crop.
static CostmapGrid makeGrid(double res, double ox, double oy,
                            unsigned int sx, unsigned int sy) {
  CostmapGrid g(res, ox, oy, sx, sy, g1_costmap::FREE_SPACE);
  for (unsigned int y = 0; y < sy; ++y) {
    for (unsigned int x = 0; x < sx; ++x) {
      // value wraps into uint8; we keep values small to avoid overflow
      g.setCost(x, y, static_cast<std::uint8_t>((x * 10 + y) % 200));
    }
  }
  return g;
}

// Test 1: crop fully inside the src grid.
// Src: 20x20 @ 0.1 m/cell, origin (0,0).
// Crop center = world (1.0, 1.0) -> src cell (10, 10).
// size_m = 0.6 -> n = round(0.6 / 0.1) = 6. ll = 10 - 3 = 7.
TEST(LocalWindow, CropInterior) {
  const double res = 0.1;
  CostmapGrid src = makeGrid(res, 0.0, 0.0, 20, 20);

  const double center_x = 1.0;
  const double center_y = 1.0;
  const double size_m = 0.6;
  const int n = static_cast<int>(std::lround(size_m / res));  // 6
  ASSERT_EQ(n, 6);

  CostmapGrid crop = cropCentered(src, center_x, center_y, size_m);
  ASSERT_EQ(crop.cells(), static_cast<std::size_t>(n * n));
  EXPECT_EQ(crop.sizeX(), static_cast<unsigned int>(n));
  EXPECT_EQ(crop.sizeY(), static_cast<unsigned int>(n));
  EXPECT_NEAR(crop.resolution(), res, 1e-12);

  // ll_x = floor((1.0-0.0)/0.1) - 6/2 = 10 - 3 = 7
  const int ll = 10 - n / 2;
  EXPECT_NEAR(crop.originX(), src.originX() + ll * res, 1e-9);
  EXPECT_NEAR(crop.originY(), src.originY() + ll * res, 1e-9);

  // Every window cell must match its corresponding src cell.
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      const int sx = ll + i;
      const int sy = ll + j;
      EXPECT_EQ(crop.at(static_cast<unsigned int>(i), static_cast<unsigned int>(j)),
                src.at(static_cast<unsigned int>(sx), static_cast<unsigned int>(sy)))
          << " at window (" << i << "," << j << ") src (" << sx << "," << sy << ")";
    }
  }
}

// Test 2: crop near the src edge -> some cells out-of-bounds.
// Src: 10x10 @ 0.1. Center near corner -> half the window falls outside.
TEST(LocalWindow, CropAtEdge) {
  const double res = 0.1;
  CostmapGrid src = makeGrid(res, 0.0, 0.0, 10, 10);

  // Center at world (0.05, 0.05) -> src cell (0, 0). ll = 0 - 3 = -3.
  const double center_x = 0.05;
  const double center_y = 0.05;
  const double size_m = 0.6;
  const int n = static_cast<int>(std::lround(size_m / res));  // 6
  const int ll = 0 - n / 2;  // = -3

  CostmapGrid crop = cropCentered(src, center_x, center_y, size_m);
  ASSERT_EQ(crop.cells(), static_cast<std::size_t>(n * n));

  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      const int sx = ll + i;
      const int sy = ll + j;
      const std::uint8_t cv =
          crop.at(static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      if (sx < 0 || sx >= 10 || sy < 0 || sy >= 10) {
        EXPECT_EQ(cv, NO_INFORMATION)
            << "out-of-bounds window (" << i << "," << j << ") should be NO_INFORMATION";
      } else {
        EXPECT_EQ(cv, src.at(static_cast<unsigned int>(sx),
                              static_cast<unsigned int>(sy)))
            << "in-bounds window (" << i << "," << j << ")";
      }
    }
  }
}

// Test 3: crop fully off the src -> all cells NO_INFORMATION.
TEST(LocalWindow, CropFullyOutside) {
  const double res = 0.1;
  CostmapGrid src = makeGrid(res, 0.0, 0.0, 10, 10);  // spans [0,1) x [0,1)

  // Center at world (100.0, 100.0) -> completely outside the src.
  CostmapGrid crop = cropCentered(src, 100.0, 100.0, 0.6);
  ASSERT_GT(crop.cells(), 0u);  // result is non-empty (6x6 window)
  for (unsigned int j = 0; j < crop.sizeY(); ++j) {
    for (unsigned int i = 0; i < crop.sizeX(); ++i) {
      EXPECT_EQ(crop.at(i, j), NO_INFORMATION)
          << "cell (" << i << "," << j << ")";
    }
  }
}

// Test 4: n computed from size_m / resolution; for size_m=6.0, res=0.1, n=60.
// Even-n centering: center cell sits at ll + n/2 in src indices.
TEST(LocalWindow, CellCountAndEvenCentering) {
  const double res = 0.1;
  const double size_m = 6.0;
  const int n = static_cast<int>(std::lround(size_m / res));  // 60
  ASSERT_EQ(n, 60);

  // Src large enough that the crop is fully interior.
  CostmapGrid src = makeGrid(res, -5.0, -5.0, 200, 200);

  // Center at world (0.0, 0.0) -> src cell = floor((0-(-5))/0.1) = 50.
  const double center_x = 0.0;
  const double center_y = 0.0;
  CostmapGrid crop = cropCentered(src, center_x, center_y, size_m);
  ASSERT_EQ(crop.cells(), static_cast<std::size_t>(n * n));

  // ll = 50 - 60/2 = 20.  Center cell in window = n/2 = 30 -> src cell 50.
  const int ll = 50 - n / 2;
  const int center_src_x = ll + n / 2;
  EXPECT_EQ(center_src_x, 50);

  // All cells interior -> value matches src.
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      EXPECT_EQ(crop.at(static_cast<unsigned int>(i), static_cast<unsigned int>(j)),
                src.at(static_cast<unsigned int>(ll + i),
                       static_cast<unsigned int>(ll + j)));
    }
  }
}

// Test 5: empty src -> empty result.
TEST(LocalWindow, EmptySrcReturnsEmpty) {
  CostmapGrid empty_src;
  CostmapGrid crop = cropCentered(empty_src, 0.0, 0.0, 6.0);
  EXPECT_EQ(crop.cells(), 0u);
}

// Test 6: size_m <= 0 -> empty result.
TEST(LocalWindow, NegativeSizeMReturnsEmpty) {
  CostmapGrid src = makeGrid(0.1, 0.0, 0.0, 10, 10);
  EXPECT_EQ(cropCentered(src, 0.5, 0.5, 0.0).cells(), 0u);
  EXPECT_EQ(cropCentered(src, 0.5, 0.5, -1.0).cells(), 0u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
