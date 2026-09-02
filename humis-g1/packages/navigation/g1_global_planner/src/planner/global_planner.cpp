#include "g1_global_planner/planner/global_planner.h"

#include <cassert>
#include <cmath>
#include <queue>
#include <vector>

#include "g1_global_planner/core/cost_model.h"
#include "g1_global_planner/planner/astar.h"
#include "g1_global_planner/planner/path_smoother.h"

namespace g1_global_planner {

GlobalPlanner::GlobalPlanner(const PlannerConfig& cfg) : cfg_(cfg) {}

void GlobalPlanner::setConfig(const PlannerConfig& cfg) { cfg_ = cfg; }

const PlannerConfig& GlobalPlanner::config() const { return cfg_; }

PlannerGrid GlobalPlanner::relaxAroundGoal(const PlannerGrid& grid,
                                           double gx, double gy) const {
  PlannerGrid out = grid;  // copy (PlannerGrid owns its data buffer)
  if (!cfg_.goal_relax_enable || cfg_.goal_relax_radius <= 0.0 ||
      grid.resolution() <= 0.0) {
    return out;
  }
  int cgx = 0, cgy = 0;
  if (!grid.worldToMap(gx, gy, cgx, cgy)) return out;  // goal off-grid: no relax
  const int r =
      static_cast<int>(std::round(cfg_.goal_relax_radius / grid.resolution()));
  const long long r2 = static_cast<long long>(r) * static_cast<long long>(r);
  const int thr = cfg_.lethal_threshold;  // 99
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (static_cast<long long>(dx) * dx + static_cast<long long>(dy) * dy > r2)
        continue;
      const int cx = cgx + dx, cy = cgy + dy;
      if (!out.inBounds(cx, cy)) continue;
      const int v = out.at(cx, cy);
      // Relax blocked-but-not-true-lethal (inscribed) to the top traversable value.
      if (v >= thr && v < 100) out.setAt(cx, cy, static_cast<int8_t>(thr - 1));
    }
  }
  return out;
}

namespace {

// BFS-collect search: find the Euclidean-nearest free cell to (ox, oy) within
// `max_dist_cells` cells (Chebyshev radius for BFS expansion). Returns true and
// sets (out_x, out_y) on success.
//
// All candidate free cells within the Chebyshev radius are collected first,
// then the one with smallest squared Euclidean distance to (ox, oy) is chosen.
// Ties are broken deterministically by (my*W + mx) index (lowest wins).
// max_dist_cells <= 0 means snapping is off; blocked endpoint fails immediately.
bool snapToFree(const PlannerGrid& grid, const CostModel& cost,
                int ox, int oy, int max_dist_cells, int& out_x, int& out_y) {
  if (max_dist_cells <= 0) return false;

  const int W = grid.width();
  const int H = grid.height();
  const std::size_t N = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);

  std::vector<bool> visited(N, false);
  std::queue<int>   q;

  auto enqueue = [&](int x, int y) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    int idx = y * W + x;
    if (visited[static_cast<std::size_t>(idx)]) return;
    visited[static_cast<std::size_t>(idx)] = true;
    q.push(idx);
  };

  enqueue(ox, oy);

  // Collect all reachable free cells within the Chebyshev radius.
  int best_idx   = -1;
  long long best_dist2 = -1;

  while (!q.empty()) {
    int idx = q.front(); q.pop();
    int my = idx / W;
    int mx = idx - my * W;
    int cheb = std::max(std::abs(mx - ox), std::abs(my - oy));
    if (cheb > max_dist_cells) continue;

    if (!cost.blocked(grid.at(mx, my))) {
      long long dx = mx - ox;
      long long dy = my - oy;
      long long dist2 = dx * dx + dy * dy;
      if (best_idx < 0 || dist2 < best_dist2 ||
          (dist2 == best_dist2 && idx < best_idx)) {
        best_dist2 = dist2;
        best_idx   = idx;
      }
    }

    // Expand 8-connected neighbors.
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        enqueue(mx + dx, my + dy);
      }
    }
  }

  if (best_idx < 0) return false;
  out_y = best_idx / W;
  out_x = best_idx - out_y * W;
  return true;
}

}  // namespace

PlanResult GlobalPlanner::plan(const PlannerGrid& in_grid,
                               const Pose2D&      start,
                               const Pose2D&      goal) const {
  // Relax the inscribed band around the goal so a near-obstacle goal is reachable;
  // the rest of plan() operates on this working copy (snap, A*, smooth).
  const PlannerGrid grid = relaxAroundGoal(in_grid, goal.x, goal.y);
  CostModel cost(cfg_);

  int sx, sy, gx, gy;

  // Convert start.
  bool start_in = grid.worldToMap(start.x, start.y, sx, sy);

  // Convert goal.
  bool goal_in = grid.worldToMap(goal.x, goal.y, gx, gy);

  const int snap_cells = (cfg_.goal_snap_radius > 0.0 && grid.resolution() > 0.0)
                             ? static_cast<int>(std::round(cfg_.goal_snap_radius / grid.resolution()))
                             : 0;

  // Snap start if out-of-bounds or blocked.
  if (!start_in || cost.blocked(grid.at(sx, sy))) {
    int cand_x = start_in ? sx : 0;
    int cand_y = start_in ? sy : 0;
    if (!start_in) {
      // Clamp to boundary for snap search origin.
      int tx, ty;
      tx = static_cast<int>(std::floor((start.x - grid.originX()) / grid.resolution()));
      ty = static_cast<int>(std::floor((start.y - grid.originY()) / grid.resolution()));
      cand_x = std::max(0, std::min(grid.width()  - 1, tx));
      cand_y = std::max(0, std::min(grid.height() - 1, ty));
    }
    if (!snapToFree(grid, cost, cand_x, cand_y, snap_cells, sx, sy)) {
      return {PlanStatus::BAD_START, {}, 0, 0.0};
    }
  }

  // Snap goal if out-of-bounds or blocked.
  if (!goal_in || cost.blocked(grid.at(gx, gy))) {
    int cand_x = goal_in ? gx : 0;
    int cand_y = goal_in ? gy : 0;
    if (!goal_in) {
      int tx, ty;
      tx = static_cast<int>(std::floor((goal.x - grid.originX()) / grid.resolution()));
      ty = static_cast<int>(std::floor((goal.y - grid.originY()) / grid.resolution()));
      cand_x = std::max(0, std::min(grid.width()  - 1, tx));
      cand_y = std::max(0, std::min(grid.height() - 1, ty));
    }
    if (!snapToFree(grid, cost, cand_x, cand_y, snap_cells, gx, gy)) {
      return {PlanStatus::BAD_GOAL, {}, 0, 0.0};
    }
  }

  // Run A*.
  AStarResult ar = astar(grid, cost, cfg_, {sx, sy}, {gx, gy});
  if (ar.status != PlanStatus::SUCCESS) {
    return {ar.status, {}, ar.expansions, 0.0};
  }

  // Smooth the cell path.
  std::vector<Cell> smooth = smoothPath(grid, cost, cfg_, ar.path);

  // Convert to world Pose2D with yaw.
  std::vector<Pose2D> world_path;
  world_path.reserve(smooth.size());
  for (const Cell& c : smooth) {
    double wx, wy;
    grid.mapToWorld(c.x, c.y, wx, wy);
    world_path.push_back({wx, wy, 0.0});
  }

  // Assign yaw: each waypoint points toward the next; last uses goal.yaw.
  for (std::size_t i = 0; i + 1 < world_path.size(); ++i) {
    double dy_seg = world_path[i + 1].y - world_path[i].y;
    double dx_seg = world_path[i + 1].x - world_path[i].x;
    world_path[i].yaw = std::atan2(dy_seg, dx_seg);
  }
  world_path.back().yaw = goal.yaw;

  // Compute path length.
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < world_path.size(); ++i) {
    double dx_seg = world_path[i + 1].x - world_path[i].x;
    double dy_seg = world_path[i + 1].y - world_path[i].y;
    length += std::sqrt(dx_seg * dx_seg + dy_seg * dy_seg);
  }

  return {PlanStatus::SUCCESS, std::move(world_path), ar.expansions, length};
}

bool GlobalPlanner::pathStillValid(const PlannerGrid&        in_grid,
                                   const std::vector<Pose2D>& path) const {
  if (path.size() <= 1) return true;

  // Relax around the path endpoint so the inscribed cells a legitimate near-goal
  // path runs through are not flagged invalid (would force a perpetual replan).
  const PlannerGrid grid = relaxAroundGoal(in_grid, path.back().x, path.back().y);
  CostModel cost(cfg_);

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    int ax, ay, bx, by;
    if (!grid.worldToMap(path[i].x, path[i].y, ax, ay)) return false;
    if (!grid.worldToMap(path[i + 1].x, path[i + 1].y, bx, by)) return false;
    if (!lineOfSight(grid, cost, {ax, ay}, {bx, by})) return false;
  }
  return true;
}

}  // namespace g1_global_planner
