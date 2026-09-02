#include "g1_global_planner/planner/astar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace g1_global_planner {

namespace {

// Octile distance heuristic (admissible for 8-connected grid, step cost >= 1).
inline double octile(int dx, int dy) {
  int mn = std::min(dx, dy);
  int mx = std::max(dx, dy);
  return static_cast<double>(mx) + (std::sqrt(2.0) - 1.0) * static_cast<double>(mn);
}

struct Node {
  double f;
  double g;
  int    idx; // my * width + mx

  // Priority queue is a max-heap; we want the smallest f on top.
  // Tie-break: prefer larger g (deeper nodes), then smaller idx.
  bool operator>(const Node& o) const {
    if (f != o.f) return f > o.f;
    if (g != o.g) return g < o.g;  // larger g -> higher priority -> comes first
    return idx > o.idx;             // smaller idx -> higher priority
  }
};

}  // namespace

AStarResult astar(const PlannerGrid& grid,
                  const CostModel&   cost,
                  const PlannerConfig& cfg,
                  Cell start,
                  Cell goal) {
  const int W = grid.width();
  const int H = grid.height();

  // Validate start and goal.
  if (!grid.inBounds(start.x, start.y) || cost.blocked(grid.at(start.x, start.y))) {
    return {PlanStatus::BAD_START, {}, 0};
  }
  if (!grid.inBounds(goal.x, goal.y) || cost.blocked(grid.at(goal.x, goal.y))) {
    return {PlanStatus::BAD_GOAL, {}, 0};
  }

  const int start_idx = start.y * W + start.x;
  const int goal_idx  = goal.y  * W + goal.x;

  if (start_idx == goal_idx) {
    return {PlanStatus::SUCCESS, {start}, 0};
  }

  // Per-cell state arrays.
  const int N = W * H;
  std::vector<double> g_val(N, std::numeric_limits<double>::infinity());
  std::vector<int>    parent(N, -1);
  std::vector<bool>   closed(N, false);

  g_val[start_idx] = 0.0;

  const int gdx = std::abs(goal.x - start.x);
  const int gdy = std::abs(goal.y - start.y);
  double h0 = cfg.heuristic_weight * octile(gdx, gdy);

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  open.push({h0, 0.0, start_idx});

  // 8-connected moves: (dx, dy, is_diagonal).
  static const int DX[8] = {1, -1, 0,  0, 1,  1, -1, -1};
  static const int DY[8] = {0,  0, 1, -1, 1, -1,  1, -1};
  static const bool DIAG[8] = {false, false, false, false, true, true, true, true};
  const double SQRT2 = std::sqrt(2.0);

  int expansions = 0;

  while (!open.empty()) {
    Node cur = open.top();
    open.pop();

    if (closed[cur.idx]) continue;
    closed[cur.idx] = true;
    ++expansions;

    if (cfg.max_expansions > 0 && expansions > cfg.max_expansions) {
      return {PlanStatus::CAPPED, {}, expansions};
    }

    if (cur.idx == goal_idx) {
      // Reconstruct path.
      std::vector<Cell> path;
      int idx = goal_idx;
      while (idx != -1) {
        int my = idx / W;
        int mx = idx - my * W;
        path.push_back({mx, my});
        idx = parent[idx];
      }
      std::reverse(path.begin(), path.end());
      return {PlanStatus::SUCCESS, std::move(path), expansions};
    }

    int cx = cur.idx % W;
    int cy = cur.idx / W;
    double g_cur = g_val[cur.idx];

    for (int d = 0; d < 8; ++d) {
      int nx = cx + DX[d];
      int ny = cy + DY[d];

      if (!grid.inBounds(nx, ny)) continue;

      int8_t nval = grid.at(nx, ny);
      if (cost.blocked(nval)) continue;

      // Corner-cutting check for diagonal moves.
      if (DIAG[d] && !cfg.allow_corner_cutting) {
        int8_t s1 = grid.at(cx + DX[d], cy);
        int8_t s2 = grid.at(cx, cy + DY[d]);
        if (cost.blocked(s1) || cost.blocked(s2)) continue;
      }

      int nidx = ny * W + nx;
      if (closed[nidx]) continue;

      double move_dist = DIAG[d] ? SQRT2 : 1.0;
      double edge_cost = move_dist * cost.stepCost(nval);
      double g_new = g_cur + edge_cost;

      if (g_new < g_val[nidx]) {
        g_val[nidx]  = g_new;
        parent[nidx] = cur.idx;
        int hdx = std::abs(nx - goal.x);
        int hdy = std::abs(ny - goal.y);
        double h = cfg.heuristic_weight * octile(hdx, hdy);
        open.push({g_new + h, g_new, nidx});
      }
    }
  }

  return {PlanStatus::NO_PATH, {}, expansions};
}

}  // namespace g1_global_planner
