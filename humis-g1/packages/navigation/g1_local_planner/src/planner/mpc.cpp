#include "g1_local_planner/planner/mpc.h"

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <OsqpEigen/OsqpEigen.h>

#include "g1_local_planner/core/kinematic_model.h"

namespace g1_local_planner {

namespace {

// State / control dimensions of the kinematic model.
constexpr int kNx = 3;  // s = [x, y, yaw]
constexpr int kNu = 3;  // u = [vx, vy, w]

// OSQP solver tolerances / iteration cap. These are SOLVER numerics, not user
// tuning, so they live as clearly named local constants (per the no-hardcoded
// rule for LOGIC values; tuning knobs all come from MpcConfig). The horizon is
// small (N=20 -> 120 vars) so the defaults below converge well inside the 50 ms
// control budget.
constexpr int    kOsqpMaxIter   = 4000;
constexpr double kOsqpAbsTol    = 1e-4;
constexpr double kOsqpRelTol    = 1e-4;

// Sequentially unwrap reference yaws so the tracking quadratic (yaw_k -
// yaw_ref_k)^2 cannot wrap (it would otherwise spin the long way across +-pi).
// Each reference yaw is shifted by a multiple of 2*pi to lie within pi of the
// PREVIOUS unwrapped target, starting from theta0. The result is a continuous
// target so the QP always rotates the short way.
std::vector<double> unwrapReferenceYaws(const std::vector<Pose2D>& refs,
                                        double theta0) {
  std::vector<double> out(refs.size());
  double prev = theta0;
  for (size_t k = 0; k < refs.size(); ++k) {
    const double delta = wrapToPi(refs[k].yaw - prev);
    out[k] = prev + delta;
    prev = out[k];
  }
  return out;
}

}  // namespace

Mpc::Mpc(const MpcConfig& cfg) : cfg_(cfg) {}

void Mpc::setConfig(const MpcConfig& cfg) { cfg_ = cfg; }

MpcResult Mpc::solve(const Pose2D& current, double theta0,
                     const std::vector<Pose2D>& refs,
                     const Twist2D& last_cmd,
                     const CostmapView& costmap,
                     const std::vector<Pose2D>* obstacle_nominal,
                     double obstacle_weight_scale) {
  MpcResult result;
  result.status = ControllerState::TRACKING;

  const int N = cfg_.horizon;
  const double dt = cfg_.dt;
  // Effective soft-obstacle weight for this solve (facade scales it toward 0 on the
  // final approach to a reachable goal; see mpc.h).
  const double w_obs = cfg_.w_obstacle * obstacle_weight_scale;

  // Clamp last_cmd componentwise into the velocity box BEFORE it seeds the k=0
  // slew rows (review Minor 3). The k=0 slew constraint is
  //   last_cmd[c] - acc*dt <= u_0[c] <= last_cmd[c] + acc*dt
  // intersected with the box. If a stale last_cmd lies outside the box (e.g. a
  // live config change shrank max_vx below the previous applied vx) the slew
  // window can sit entirely outside the box, leaving an EMPTY feasible set for
  // u_0[c] and an infeasible QP. Clamping last_cmd into the box first guarantees
  // the slew window always overlaps the box, so u_0 stays feasible.
  auto clampBox = [](double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  };
  const Twist2D last_cmd_clamped{
      clampBox(last_cmd.vx, cfg_.min_vx, cfg_.max_vx),
      clampBox(last_cmd.vy, -cfg_.max_vy, cfg_.max_vy),
      clampBox(last_cmd.w, -cfg_.max_wz, cfg_.max_wz)};

  // Helper: build a zero-rollout fallback result (used on any failure / bail).
  auto fail = [&]() -> MpcResult {
    MpcResult r;
    r.status = ControllerState::TRACKING;
    r.solved = false;
    r.applied = Twist2D{};
    if (N > 0) {
      const std::vector<Twist2D> zeros(static_cast<size_t>(N), Twist2D{});
      r.predicted = rolloutLinear(current, zeros, theta0, dt);
    }
    r.cost = 0.0;
    return r;
  };

  // Reference must match the horizon exactly (validate refs.size()).
  if (N <= 0 || static_cast<int>(refs.size()) != N) {
    return fail();
  }

  // ---- Dimensions -------------------------------------------------------
  const int nz = (kNx + kNu) * N;       // [s_1..s_N , u_0..u_{N-1}]
  const int state_base = 0;             // s_k block starts here
  const int ctrl_base  = kNx * N;       // u_j block starts here

  // z-offset of state s_k (k = 1..N) and control u_j (j = 0..N-1).
  auto sIdx = [&](int k, int comp) { return state_base + (k - 1) * kNx + comp; };
  auto uIdx = [&](int j, int comp) { return ctrl_base + j * kNu + comp; };

  // ---- Fixed LTI input matrix B(theta0, dt) -----------------------------
  const Eigen::Matrix3d B = controlMatrix(theta0, dt);

  // ---- Unwrapped reference yaws (critical, see unwrapReferenceYaws) ------
  const std::vector<double> yaw_ref = unwrapReferenceYaws(refs, theta0);

  // ---- Obstacle linearisation nominal (warm start, see mpc.h) ------------
  // Sample the obstacle gradient at obstacle_nominal[k-1] (last cycle's predicted
  // state) when a valid warm-start nominal is supplied; otherwise fall back to
  // the reference points. This re-linearises the obstacle term about the
  // previous predicted path so avoidance is self-reinforcing across cycles.
  const bool use_nominal =
      (obstacle_nominal != nullptr) &&
      (static_cast<int>(obstacle_nominal->size()) == N);
  const std::vector<Pose2D>& obs_nom = use_nominal ? *obstacle_nominal : refs;

  // ============================================================
  //  Hessian P  (OSQP form: minimise 0.5 z^T P z + q^T z)
  //  Built full-symmetric as triplets, then the UPPER triangle is extracted
  //  into the sparse matrix handed to OSQP (OSQP requires upper-triangular).
  // ============================================================
  std::vector<Eigen::Triplet<double>> p_full;   // full symmetric P
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nz);

  // accumulate P(i,j) += v on the FULL matrix (both off-diagonal entries).
  auto addP = [&](int i, int j, double v) {
    if (v == 0.0) return;
    p_full.emplace_back(i, j, v);
  };

  // --- State costs: tracking + obstacle (per step k = 1..N) ---
  for (int k = 1; k <= N; ++k) {
    const bool terminal = (k == N);
    const double w_pos = terminal ? cfg_.w_pos_terminal : cfg_.w_pos;
    const double w_yaw = terminal ? cfg_.w_yaw_terminal : cfg_.w_yaw;

    const double px_ref = refs[k - 1].x;
    const double py_ref = refs[k - 1].y;
    const double yw_ref = yaw_ref[k - 1];

    // tracking position: w_pos * (x-px_ref)^2 + (y-py_ref)^2
    //   -> P += 2*w_pos on x,x and y,y ; q += -2*w_pos*p_ref
    const int ix = sIdx(k, 0);
    const int iy = sIdx(k, 1);
    const int iw = sIdx(k, 2);
    addP(ix, ix, 2.0 * w_pos);
    addP(iy, iy, 2.0 * w_pos);
    q(ix) += -2.0 * w_pos * px_ref;
    q(iy) += -2.0 * w_pos * py_ref;

    // tracking yaw: w_yaw * (yaw - yw_ref)^2
    addP(iw, iw, 2.0 * w_yaw);
    q(iw) += -2.0 * w_yaw * yw_ref;

    // Obstacle term: linearised about the nominal point obs_nom[k-1]. The
    // nominal is the reference point by default, or last cycle's predicted state
    // when a warm-start nominal is supplied (real-time-iteration SQP; see mpc.h
    // for the rationale and the head-on-symmetry caveat). The linear term
    //   w_obstacle * g_k . p_k
    // pushes p_k DOWN-gradient (gradient() points toward INCREASING cost), so
    // the path shifts off obstacles by ~ (w_obstacle/w_pos)*g, balanced by the
    // tracking quadratic. The constant -g_k.p_nom_k drops out of argmin.
    if (w_obs != 0.0 && costmap.valid()) {
      double gx = 0.0, gy = 0.0;
      if (has_footprint_) {
        // Sum the costmap gradient over the rotated footprint circle centers at
        // the nominal pose for this step, so the WHOLE body is pushed off
        // obstacles. worldSamples() appends to the buffer; clear it first.
        std::vector<std::pair<double, double>> pts;
        footprint_.worldSamples(obs_nom[k - 1].x, obs_nom[k - 1].y,
                                obs_nom[k - 1].yaw, &pts);
        for (const auto& p : pts) {
          const std::pair<double, double> g = costmap.gradient(p.first, p.second);
          gx += g.first;
          gy += g.second;
        }
        if (!pts.empty()) {
          gx /= static_cast<double>(pts.size());
          gy /= static_cast<double>(pts.size());
        }
      } else {
        const std::pair<double, double> g =
            costmap.gradient(obs_nom[k - 1].x, obs_nom[k - 1].y);
        gx = g.first;
        gy = g.second;
      }
      q(ix) += w_obs * gx;
      q(iy) += w_obs * gy;
    }
  }

  // --- Control costs: effort + strafe + rate (per step j = 0..N-1) ---
  // effort : w_effort * (vx^2 + vy^2 + w^2)
  // strafe : w_lateral * vy^2
  // rate   : w_rate * ||u_j - u_{j-1}||^2 , u_{-1} = last_cmd (constant)
  const Eigen::Vector3d u_prev0(
      last_cmd_clamped.vx, last_cmd_clamped.vy, last_cmd_clamped.w);
  for (int j = 0; j < N; ++j) {
    const int jvx = uIdx(j, 0);
    const int jvy = uIdx(j, 1);
    const int jw  = uIdx(j, 2);

    // effort
    addP(jvx, jvx, 2.0 * cfg_.w_effort);
    addP(jvy, jvy, 2.0 * cfg_.w_effort);
    addP(jw,  jw,  2.0 * cfg_.w_effort);

    // strafe (vy only)
    addP(jvy, jvy, 2.0 * cfg_.w_lateral);

    // rate term: w_rate * sum_c (u_j[c] - u_{j-1}[c])^2
    // For j == 0, u_{-1} = last_cmd is constant -> contributes to diagonal of
    // u_0 plus a linear term. For j >= 1, couples u_j and u_{j-1}.
    if (cfg_.w_rate != 0.0) {
      const int comp_idx[3] = {jvx, jvy, jw};
      if (j == 0) {
        for (int c = 0; c < kNu; ++c) {
          addP(comp_idx[c], comp_idx[c], 2.0 * cfg_.w_rate);
          q(comp_idx[c]) += -2.0 * cfg_.w_rate * u_prev0(c);
        }
      } else {
        const int prev_idx[3] = {uIdx(j - 1, 0), uIdx(j - 1, 1), uIdx(j - 1, 2)};
        for (int c = 0; c < kNu; ++c) {
          // d/d.. of (u_j - u_{j-1})^2:
          //   +2*w on u_j,u_j and u_{j-1},u_{j-1}; -2*w on the cross terms.
          addP(comp_idx[c], comp_idx[c], 2.0 * cfg_.w_rate);
          addP(prev_idx[c], prev_idx[c], 2.0 * cfg_.w_rate);
          addP(comp_idx[c], prev_idx[c], -2.0 * cfg_.w_rate);
          addP(prev_idx[c], comp_idx[c], -2.0 * cfg_.w_rate);
        }
      }
    }
  }

  // Build the full symmetric P, then take the upper triangle for OSQP.
  Eigen::SparseMatrix<double> P_full(nz, nz);
  P_full.setFromTriplets(p_full.begin(), p_full.end());
  Eigen::SparseMatrix<double> hessian =
      P_full.triangularView<Eigen::Upper>();
  hessian.makeCompressed();

  // ============================================================
  //  Constraints  l <= A z <= u
  //    rows [0,          3N)  : dynamics equality   (l == u)
  //    rows [3N,         6N)  : box on u
  //    rows [6N,         9N)  : slew on u
  // ============================================================
  const int row_dyn  = 0;
  const int row_box  = kNx * N;
  const int row_slew = row_box + kNu * N;
  const int nc = row_slew + kNu * N;  // 9N

  std::vector<Eigen::Triplet<double>> a_trip;
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(nc);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(nc);

  // --- Dynamics equality ---
  //   k = 1:  s_1 - B u_0 = s_0      (s_0 = current, on the RHS)
  //   k>=2 :  s_k - s_{k-1} - B u_{k-1} = 0
  for (int k = 1; k <= N; ++k) {
    const int base = row_dyn + (k - 1) * kNx;
    // + I on s_k
    for (int c = 0; c < kNx; ++c) {
      a_trip.emplace_back(base + c, sIdx(k, c), 1.0);
    }
    // - I on s_{k-1} (only for k >= 2; for k == 1, s_0 is the constant RHS)
    if (k >= 2) {
      for (int c = 0; c < kNx; ++c) {
        a_trip.emplace_back(base + c, sIdx(k - 1, c), -1.0);
      }
    }
    // - B on u_{k-1}
    for (int r = 0; r < kNx; ++r) {
      for (int cc = 0; cc < kNu; ++cc) {
        const double v = B(r, cc);
        if (v != 0.0) a_trip.emplace_back(base + r, uIdx(k - 1, cc), -v);
      }
    }
    // RHS: k == 1 -> s_0 ; else 0 (equality -> lower == upper)
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    if (k == 1) {
      rhs << current.x, current.y, current.yaw;
    }
    for (int c = 0; c < kNx; ++c) {
      lower(base + c) = rhs(c);
      upper(base + c) = rhs(c);
    }
  }

  // --- Box on u ---
  //   min_vx <= vx <= max_vx ; -max_vy <= vy <= max_vy ; -max_wz <= w <= max_wz
  for (int j = 0; j < N; ++j) {
    const int base = row_box + j * kNu;
    a_trip.emplace_back(base + 0, uIdx(j, 0), 1.0);
    a_trip.emplace_back(base + 1, uIdx(j, 1), 1.0);
    a_trip.emplace_back(base + 2, uIdx(j, 2), 1.0);
    lower(base + 0) = cfg_.min_vx;   upper(base + 0) = cfg_.max_vx;
    lower(base + 1) = -cfg_.max_vy;  upper(base + 1) = cfg_.max_vy;
    lower(base + 2) = -cfg_.max_wz;  upper(base + 2) = cfg_.max_wz;
  }

  // --- Slew on u ---
  //   |u_j[c] - u_{j-1}[c]| <= acc_lim[c]*dt
  //   j == 0: u_{-1} = last_cmd (constant) -> box around last_cmd:
  //           last_cmd[c] - acc*dt <= u_0[c] <= last_cmd[c] + acc*dt
  //   j >= 1: -acc*dt <= u_j[c] - u_{j-1}[c] <= acc*dt
  const double acc[3] = {cfg_.acc_lim_x, cfg_.acc_lim_y, cfg_.acc_lim_theta};
  const double u_prev0_arr[3] = {last_cmd_clamped.vx, last_cmd_clamped.vy,
                                 last_cmd_clamped.w};
  for (int j = 0; j < N; ++j) {
    const int base = row_slew + j * kNu;
    for (int c = 0; c < kNu; ++c) {
      const double d = acc[c] * dt;
      a_trip.emplace_back(base + c, uIdx(j, c), 1.0);
      if (j == 0) {
        lower(base + c) = u_prev0_arr[c] - d;
        upper(base + c) = u_prev0_arr[c] + d;
      } else {
        a_trip.emplace_back(base + c, uIdx(j - 1, c), -1.0);
        lower(base + c) = -d;
        upper(base + c) = d;
      }
    }
  }

  Eigen::SparseMatrix<double> A(nc, nz);
  A.setFromTriplets(a_trip.begin(), a_trip.end());
  A.makeCompressed();

  // ============================================================
  //  Solve with OSQP via osqp-eigen. Any failure path returns the zero-rollout
  //  fallback and never throws (soft penalty -> the QP is feasible, but a
  //  numerical / setup failure is still handled gracefully).
  // ============================================================
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.settings()->setMaxIteration(kOsqpMaxIter);
  solver.settings()->setAbsoluteTolerance(kOsqpAbsTol);
  solver.settings()->setRelativeTolerance(kOsqpRelTol);

  solver.data()->setNumberOfVariables(nz);
  solver.data()->setNumberOfConstraints(nc);

  if (!solver.data()->setHessianMatrix(hessian) ||
      !solver.data()->setGradient(q) ||
      !solver.data()->setLinearConstraintsMatrix(A) ||
      !solver.data()->setLowerBound(lower) ||
      !solver.data()->setUpperBound(upper)) {
    return fail();
  }

  if (!solver.initSolver()) {
    return fail();
  }

  const OsqpEigen::ErrorExitFlag flag = solver.solveProblem();
  if (flag != OsqpEigen::ErrorExitFlag::NoError) {
    return fail();
  }

  const OsqpEigen::Status status = solver.getStatus();
  const bool ok = (status == OsqpEigen::Status::Solved) ||
                  (status == OsqpEigen::Status::SolvedInaccurate);
  if (!ok) {
    return fail();
  }

  const Eigen::VectorXd z = solver.getSolution();
  if (z.size() != nz) {
    return fail();
  }

  // ---- Extract the control sequence and predicted states from z ----
  result.solved = true;
  result.cost = solver.getObjValue();

  result.controls.reserve(static_cast<size_t>(N));
  for (int j = 0; j < N; ++j) {
    Twist2D u;
    u.vx = z(uIdx(j, 0));
    u.vy = z(uIdx(j, 1));
    u.w  = z(uIdx(j, 2));
    result.controls.push_back(u);
  }
  result.applied = result.controls.front();

  // Read s_1..s_N straight out of z (these match a rolloutLinear of the solved
  // controls because the dynamics equality is enforced; reading from z avoids
  // re-accumulating floating-point drift).
  result.predicted.reserve(static_cast<size_t>(N));
  for (int k = 1; k <= N; ++k) {
    Pose2D s;
    s.x   = z(sIdx(k, 0));
    s.y   = z(sIdx(k, 1));
    s.yaw = z(sIdx(k, 2));
    result.predicted.push_back(s);
  }

  return result;
}

}  // namespace g1_local_planner
