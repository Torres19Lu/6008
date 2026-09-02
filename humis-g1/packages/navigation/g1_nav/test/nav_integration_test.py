#!/usr/bin/env python3
"""End-to-end rostest integration harness for g1_nav.

A SINGLE rospy node plays three roles at once:

  1. Mock upstreams -- everything the real g1_nav_node talks to:
       * a static map->base_link TF (so the node always has a pose; the .test
         sets a large stuck_window so this static pose never trips the
         no-progress watchdog),
       * the four services the node calls, each recording its invocation:
         /g1_local_planner/enable (SetBool), /g1_costmap/clear_costmap (Empty),
         /g1/arm (Trigger), /g1/halt (Trigger),
       * a scripted /g1_local_planner/state (std_msgs/Int8) publisher driven by
         a `local_state` variable the scenarios set,
       * a /nav/cmd_vel_track (geometry_msgs/Twist) publisher emitting a small
         constant forward command so RELAY_TRACK has something fresh to relay,
       * an informational /g1/loco_status publisher.
  2. Action client -- a SimpleActionClient of the navigate_to action.
  3. Verifier -- subscribes /nav/state (records the NavState sequence), /nav/goal
     (the event that drives the scenarios when the node forwards a goal), and
     /cmd_vel.

The harness is EVENT-DRIVEN: it reacts to the node's /nav/goal forwards and to
the clear_costmap service call, so the four scenarios are deterministic rather
than sleep-fragile. Waits are poll-with-timeout, never fixed sleeps.

Run by rostest via nav_integration.test.
"""
import threading
import unittest

import rospy
import actionlib
import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import Int8
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import Trigger, TriggerResponse

from g1_msgs.msg import (
    NavigateToAction,
    NavigateToGoal,
    NavigateToResult,
    LocoStatus,
)

PKG = "g1_nav"

# LocalState wire constants (mirror g1_nav::LocalState / the local planner enum).
LS_IDLE = 0
LS_TRACKING = 1
LS_GOAL_REACHED = 2
LS_NO_PATH = 3
LS_NO_GOAL = 4
LS_NO_COSTMAP = 5
LS_STUCK = 6

# NavState wire constants (mirror g1_nav::NavState; published on /nav/state).
NS_IDLE = 0
NS_PLAN = 1
NS_TRACK = 2
NS_REPLAN = 3
NS_RECOVER = 4
NS_SUCCEEDED = 5
NS_ABORTED = 6
NS_RETRY = 7

NS_NAME = {
    NS_IDLE: "IDLE",
    NS_PLAN: "PLAN",
    NS_TRACK: "TRACK",
    NS_REPLAN: "REPLAN",
    NS_RECOVER: "RECOVER",
    NS_SUCCEEDED: "SUCCEEDED",
    NS_ABORTED: "ABORTED",
    NS_RETRY: "RETRY",
}

# Action outcome constants (mirror NavigateTo.action Result OUTCOME_*).
OUTCOME_SUCCEEDED = NavigateToResult.OUTCOME_SUCCEEDED
ABORTED_OUTCOMES = {
    NavigateToResult.OUTCOME_ABORTED_NO_PATH,
    NavigateToResult.OUTCOME_ABORTED_STUCK,
    NavigateToResult.OUTCOME_ABORTED_TIMEOUT,
    NavigateToResult.OUTCOME_ABORTED_RECOVERY_EXHAUSTED,
    NavigateToResult.OUTCOME_ABORTED_BLOCKED_TIMEOUT,
}


class NavMock(object):
    """The mock upstreams + verifier, shared by every test method."""

    def __init__(self):
        self.lock = threading.Lock()

        # Scripted local-planner state the node reads back on /g1_local_planner/state.
        self.local_state = LS_NO_GOAL

        # Verifier records: the ordered sequence of distinct NavState values seen,
        # plus the set for quick membership checks.
        self.state_seq = []
        self.state_set = set()

        # Service-call flags (the asserts read these).
        self.enable_last = None  # last SetBool request.data
        self.enable_count = 0
        self.clear_called = False
        self.clear_count = 0
        self.arm_called = False
        self.arm_count = 0
        self.halt_called = False
        self.halt_count = 0

        # Goal-forward event: the node publishing /nav/goal drives the scenarios.
        self.goal_forward_count = 0

        # cmd_vel sightings (not asserted on value; scenario 4 queries the master).
        self.cmd_vel_count = 0

        # ---- Static TF map->base_link (identity at a fixed origin). ----
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

        # ---- Services the node calls (each records the call). ----
        self.enable_srv = rospy.Service(
            "/g1_local_planner/enable", SetBool, self._on_enable)
        self.clear_srv = rospy.Service(
            "/g1_costmap/clear_costmap", Empty, self._on_clear)
        self.arm_srv = rospy.Service("/g1/arm", Trigger, self._on_arm)
        self.halt_srv = rospy.Service("/g1/halt", Trigger, self._on_halt)

        # ---- Publishers the node reads. ----
        self.state_pub = rospy.Publisher(
            "/g1_local_planner/state", Int8, queue_size=1)
        self.track_pub = rospy.Publisher(
            "/nav/cmd_vel_track", Twist, queue_size=1)
        self.loco_pub = rospy.Publisher(
            "/g1/loco_status", LocoStatus, queue_size=1)

        # ---- Subscribers (the verifier). ----
        self.state_sub = rospy.Subscriber(
            "/nav/state", Int8, self._on_nav_state, queue_size=10)
        self.goal_sub = rospy.Subscriber(
            "/nav/goal", PoseStamped, self._on_nav_goal, queue_size=10)
        self.cmd_sub = rospy.Subscriber(
            "/cmd_vel", Twist, self._on_cmd_vel, queue_size=10)

        # ---- 20 Hz timer: publish the scripted local_state + a fresh track cmd. ----
        self.timer = rospy.Timer(rospy.Duration(1.0 / 20.0), self._tick)

        # ---- Action client. ----
        self.client = actionlib.SimpleActionClient(
            "navigate_to", NavigateToAction)

        # A callback hook the scenarios install to react to a /nav/goal forward
        # or a clear_costmap call without polling. Called WITHOUT the lock held.
        self._goal_hook = None
        self._clear_hook = None

    # ---- Service handlers ----
    def _on_enable(self, req):
        with self.lock:
            self.enable_last = req.data
            self.enable_count += 1
        return SetBoolResponse(success=True, message="mock enable")

    def _on_clear(self, req):
        hook = None
        with self.lock:
            self.clear_called = True
            self.clear_count += 1
            hook = self._clear_hook
        if hook is not None:
            hook()
        return EmptyResponse()

    def _on_arm(self, req):
        with self.lock:
            self.arm_called = True
            self.arm_count += 1
        return TriggerResponse(success=True, message="mock arm")

    def _on_halt(self, req):
        with self.lock:
            self.halt_called = True
            self.halt_count += 1
        return TriggerResponse(success=True, message="mock halt")

    # ---- Verifier callbacks ----
    def _on_nav_state(self, msg):
        with self.lock:
            s = int(msg.data)
            self.state_set.add(s)
            if not self.state_seq or self.state_seq[-1] != s:
                self.state_seq.append(s)

    def _on_nav_goal(self, msg):
        hook = None
        with self.lock:
            self.goal_forward_count += 1
            hook = self._goal_hook
        if hook is not None:
            hook()

    def _on_cmd_vel(self, msg):
        with self.lock:
            self.cmd_vel_count += 1

    # ---- 20 Hz publish tick ----
    def _tick(self, _event):
        with self.lock:
            ls = self.local_state
        s = Int8()
        s.data = ls
        self.state_pub.publish(s)
        # A small constant forward command so RELAY_TRACK relays something fresh.
        t = Twist()
        t.linear.x = 0.1
        self.track_pub.publish(t)
        ls_msg = LocoStatus()
        ls_msg.halted = False
        self.loco_pub.publish(ls_msg)

    # ---- Scenario helpers ----
    def reset_scenario(self):
        """Clear per-scenario verifier + service-call state and the hooks."""
        with self.lock:
            self.local_state = LS_NO_GOAL
            self.state_seq = []
            self.state_set = set()
            self.enable_last = None
            self.enable_count = 0
            self.clear_called = False
            self.clear_count = 0
            self.arm_called = False
            self.arm_count = 0
            self.halt_called = False
            self.halt_count = 0
            self.goal_forward_count = 0
            self.cmd_vel_count = 0
            self._goal_hook = None
            self._clear_hook = None

    def set_local_state(self, value):
        with self.lock:
            self.local_state = value

    def set_goal_hook(self, fn):
        with self.lock:
            self._goal_hook = fn

    def set_clear_hook(self, fn):
        with self.lock:
            self._clear_hook = fn

    def seq_snapshot(self):
        with self.lock:
            return list(self.state_seq)

    def make_goal(self, x=1.0, y=0.0, yaw=0.0):
        g = NavigateToGoal()
        g.goal_type = NavigateToGoal.GOAL_POSE
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        g.target_pose = ps
        return g

    # ---- poll-with-timeout primitives ----
    def wait_for(self, pred, timeout, hz=50.0):
        """Poll pred() until true or timeout (seconds). Returns the bool result."""
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(hz)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            with self.lock:
                ok = pred()
            if ok:
                return True
            rate.sleep()
        with self.lock:
            return pred()

    def wait_state_seen(self, state, timeout):
        return self.wait_for(lambda: state in self.state_set, timeout)


class TestNavIntegration(unittest.TestCase):
    """The four end-to-end scenarios, each a method on the shared mock."""

    mock = None

    @classmethod
    def setUpClass(cls):
        rospy.init_node("nav_integration_test", anonymous=False)
        cls.mock = NavMock()
        # Wait for the node's action server and its service clients to be wired.
        if not cls.mock.client.wait_for_server(rospy.Duration(30.0)):
            raise AssertionError(
                "navigate_to action server did not come up within 30 s")
        # Let the static TF + the first scripted state publishes propagate so the
        # node has a pose and a cached local_state before the first goal.
        cls.mock.wait_for(lambda: True, 0.5)

    # ---------------- Scenario 1: clean run -> SUCCEEDED ----------------
    def test_1_clean_run_succeeds(self):
        m = self.mock
        m.reset_scenario()

        # On the first /nav/goal forward, the node has accepted the goal and
        # entered PLAN; make the local planner report TRACKING so PLAN->TRACK.
        def on_goal():
            m.set_local_state(LS_TRACKING)
        m.set_goal_hook(on_goal)

        m.client.send_goal(m.make_goal())

        self.assertTrue(
            m.wait_state_seen(NS_PLAN, 10.0),
            "never saw PLAN; seq=%s" % self._names(m.seq_snapshot()))
        self.assertTrue(
            m.wait_state_seen(NS_TRACK, 10.0),
            "never saw TRACK; seq=%s" % self._names(m.seq_snapshot()))

        # Once tracking is observed, the local planner reports the goal reached.
        m.set_local_state(LS_GOAL_REACHED)

        finished = m.client.wait_for_result(rospy.Duration(15.0))
        self.assertTrue(
            finished, "action did not finish; seq=%s"
            % self._names(m.seq_snapshot()))
        res = m.client.get_result()
        self.assertIsNotNone(res, "no result object")
        self.assertTrue(res.success, "result.success false; msg=%s" % res.message)
        self.assertEqual(res.outcome, OUTCOME_SUCCEEDED,
                         "outcome=%d not SUCCEEDED" % res.outcome)

        self.assertTrue(
            m.wait_state_seen(NS_SUCCEEDED, 5.0),
            "never saw SUCCEEDED; seq=%s" % self._names(m.seq_snapshot()))

        seq = m.seq_snapshot()
        self._assert_order(seq, [NS_PLAN, NS_TRACK, NS_SUCCEEDED])
        with m.lock:
            self.assertTrue(m.arm_called, "arm was not called on accept")
            self.assertTrue(m.halt_called, "halt was not called on terminal")

    # ---- Scenario 2: blocked -> REPLAN -> RETRY -> path clears -> SUCCEEDED ----
    def test_2_blocked_retries_then_resumes(self):
        m = self.mock
        m.reset_scenario()

        # First goal forward -> tracking (PLAN->TRACK). Later goal forwards (the
        # REPLAN re-nudge, the RETRY re-forwards) must NOT reset us to
        # tracking, so the hook only acts on the FIRST forward.
        started = {"v": False}

        def on_goal():
            if not started["v"]:
                started["v"] = True
                m.set_local_state(LS_TRACKING)
        m.set_goal_hook(on_goal)

        m.client.send_goal(m.make_goal())

        self.assertTrue(
            m.wait_state_seen(NS_TRACK, 10.0),
            "never saw TRACK; seq=%s" % self._names(m.seq_snapshot()))

        # Block the path: persistent NO_PATH drives TRACK->REPLAN->RETRY (no
        # active recovery for a pure no-path block).
        m.set_local_state(LS_NO_PATH)

        self.assertTrue(
            m.wait_state_seen(NS_REPLAN, 10.0),
            "never saw REPLAN; seq=%s" % self._names(m.seq_snapshot()))
        self.assertTrue(
            m.wait_state_seen(NS_RETRY, 10.0),
            "never saw RETRY; seq=%s" % self._names(m.seq_snapshot()))

        # The road clears: report TRACKING. RETRY resumes to TRACK on the next
        # fresh path (proving RETRY is not a dead end). state_seq must show a
        # TRACK AFTER the RETRY, so check the last recorded state flips to TRACK.
        m.set_local_state(LS_TRACKING)
        self.assertTrue(
            m.wait_for(
                lambda: m.state_seq and m.state_seq[-1] == NS_TRACK, 10.0),
            "did not resume to TRACK after the block cleared; seq=%s"
            % self._names(m.seq_snapshot()))
        m.set_local_state(LS_GOAL_REACHED)

        finished = m.client.wait_for_result(rospy.Duration(15.0))
        self.assertTrue(
            finished, "action did not finish; seq=%s"
            % self._names(m.seq_snapshot()))
        res = m.client.get_result()
        self.assertTrue(res.success, "result.success false; msg=%s" % res.message)
        self.assertEqual(res.outcome, OUTCOME_SUCCEEDED,
                         "outcome=%d not SUCCEEDED" % res.outcome)

        seq = m.seq_snapshot()
        self.assertIn(NS_REPLAN, seq,
                      "REPLAN not in seq=%s" % self._names(seq))
        self.assertIn(NS_RETRY, seq,
                      "RETRY not in seq=%s" % self._names(seq))
        self.assertIn(NS_SUCCEEDED, seq,
                      "SUCCEEDED not in seq=%s" % self._names(seq))

    # ---------------- Scenario 3: unreachable -> ABORTED ----------------
    def test_3_unreachable_aborts(self):
        m = self.mock
        m.reset_scenario()

        started = {"v": False}

        def on_goal():
            if not started["v"]:
                started["v"] = True
                m.set_local_state(LS_TRACKING)
        m.set_goal_hook(on_goal)
        # No path ever appears: the goal stays blocked.

        m.client.send_goal(m.make_goal())

        self.assertTrue(
            m.wait_state_seen(NS_TRACK, 10.0),
            "never saw TRACK; seq=%s" % self._names(m.seq_snapshot()))

        # Hard block forever: STUCK drives TRACK->RETRY directly (hold + replan, no
        # blind recovery). With the short retry_budget that times out ->
        # ABORTED_BLOCKED_TIMEOUT. RETRY never clears (no path ever appears).
        m.set_local_state(LS_STUCK)

        self.assertTrue(
            m.wait_state_seen(NS_RETRY, 10.0),
            "never saw RETRY; seq=%s" % self._names(m.seq_snapshot()))

        finished = m.client.wait_for_result(rospy.Duration(20.0))
        self.assertTrue(
            finished, "action did not finish; seq=%s"
            % self._names(m.seq_snapshot()))
        res = m.client.get_result()
        self.assertFalse(res.success, "result.success true on an unreachable goal")
        self.assertIn(
            res.outcome, ABORTED_OUTCOMES,
            "outcome=%d not an ABORTED_* code" % res.outcome)
        self.assertEqual(
            res.outcome, NavigateToResult.OUTCOME_ABORTED_BLOCKED_TIMEOUT,
            "outcome=%d not ABORTED_BLOCKED_TIMEOUT" % res.outcome)

        self.assertTrue(
            m.wait_state_seen(NS_ABORTED, 5.0),
            "never saw ABORTED; seq=%s" % self._names(m.seq_snapshot()))
        with m.lock:
            self.assertTrue(m.halt_called, "halt was not called on abort")

    # ---------------- Scenario 4: single /cmd_vel writer ----------------
    def test_4_single_cmd_vel_writer(self):
        m = self.mock
        # Query the ROS master for the publishers of /cmd_vel; exactly one (the
        # g1_nav node) must publish it. The mock must NOT publish /cmd_vel.
        master = rospy.get_master()
        code, _msg, state = master.getSystemState()
        self.assertEqual(code, 1, "master getSystemState failed")
        publishers = state[0]  # [ [topic, [pubs...]], ... ]
        pubs = []
        for topic, node_list in publishers:
            if topic == "/cmd_vel":
                pubs = node_list
                break
        self.assertEqual(
            len(pubs), 1,
            "/cmd_vel must have exactly one publisher (the g1_nav node); got %s"
            % pubs)
        self.assertTrue(
            any("g1_nav" in p for p in pubs),
            "/cmd_vel publisher is not g1_nav; got %s" % pubs)

    # ---- helpers ----
    @staticmethod
    def _names(seq):
        return [NS_NAME.get(s, str(s)) for s in seq]

    def _assert_order(self, seq, required):
        """Assert that `required` appears as a subsequence of `seq` (in order)."""
        it = iter(seq)
        for want in required:
            found = False
            for s in it:
                if s == want:
                    found = True
                    break
            self.assertTrue(
                found,
                "state %s missing or out of order in seq=%s (want order %s)"
                % (NS_NAME.get(want, want), self._names(seq),
                   self._names(required)))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, "nav_integration_test", TestNavIntegration)
