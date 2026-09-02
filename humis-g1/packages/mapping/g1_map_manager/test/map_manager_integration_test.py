#!/usr/bin/env python3
"""Integration rostest for g1_map_manager: the real map_manager_node against a single
mock node that plays the SLAM backend (services + latched /slam/keyframe_poses + a
static map->base_link TF) AND g1_nav (a /navigate_to action server). Asserts the map
lifecycle on the geometry/+overlays/+snapshots/ layout, nav-point authoring/resolution
+ bounded overlay history, the read-only localization guarantee, snapshot/restore,
same-map delegation, and cross-map switching (seed hit / global fallback / abort).
"""
import os
import shutil
import unittest

import rospy
import rostest
import actionlib
import tf2_ros
import yaml

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger, TriggerResponse
from g1_msgs.srv import (LoadMap, LoadMapResponse, SaveMap, SaveMapResponse,
                         Relocalize, RelocalizeResponse, StartMapping,
                         StartLocalization, StartIncremental, StopMapping,
                         GetActiveMap, MapSnapshot, MapSnapshotRequest, ListMaps,
                         AddNavPoint, QueryNavPoint, GatewayCapture,
                         GatewayCaptureRequest, MapEditCommand, MapEditCommandRequest,
                         GetTopology, GetMapNeighbors)
from g1_msgs.msg import (KeyframePoseArray, KeyframePose, MapManagerState,
                         NavPointArray, NavigateToAction, NavigateToGoal,
                         NavigateToResult)

PKG = 'g1_map_manager'


class MapManagerIntegration(unittest.TestCase):
    def setUp(self):
        rospy.init_node('map_manager_integration_test')
        self.maps_root = rospy.get_param('~maps_root')
        shutil.rmtree(self.maps_root, ignore_errors=True)
        os.makedirs(self.maps_root, exist_ok=True)
        shutil.rmtree(self._topo_dir(), ignore_errors=True)

        self.calls = {'reset': 0, 'load': [], 'reloc': [], 'begin': 0, 'save': []}
        self.legs = []
        self.nav_ok = True
        self.reloc_mode = 'always'  # 'always' | 'fallback' (fail seed) | 'never'
        rospy.Service('/slam/reset', Trigger, self._reset)
        rospy.Service('/slam/load_map', LoadMap, self._load)
        rospy.Service('/slam/relocalize', Relocalize, self._reloc)
        rospy.Service('/slam/begin_incremental', Trigger, self._begin)
        rospy.Service('/slam/save_map', SaveMap, self._save)

        self._nav = actionlib.SimpleActionServer(
            '/navigate_to', NavigateToAction, execute_cb=self._nav_exec,
            auto_start=False)
        self._nav.start()

        self._kf_pub = rospy.Publisher('/slam/keyframe_poses', KeyframePoseArray,
                                       queue_size=1, latch=True)
        self._static_tf = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 1.0
        t.transform.rotation.w = 1.0
        self._static_tf.sendTransform(t)

        for name in ('start_mapping', 'stop_mapping', 'start_localization',
                     'start_incremental', 'get_active_map', 'snapshot',
                     'list_maps', 'add_nav_point', 'query_nav_point',
                     'reload_topology', 'get_topology', 'get_map_neighbors'):
            rospy.wait_for_service('/map_manager/' + name, timeout=20.0)
        rospy.wait_for_service('/map_manager/gateway_capture', timeout=20.0)
        rospy.wait_for_service('/map_edit/command', timeout=20.0)

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while (self._kf_pub.get_num_connections() == 0
               and rospy.Time.now() < deadline):
            rospy.sleep(0.05)
        self._publish_kf(0.0, 0.0)
        rospy.sleep(0.4)

    # ---- mock backend handlers ----
    def _reset(self, _req):
        self.calls['reset'] += 1
        return TriggerResponse(success=True, message='reset')

    def _load(self, req):
        self.calls['load'].append((req.path, req.read_only))
        return LoadMapResponse(success=True, message='loaded')

    def _reloc(self, req):
        self.calls['reloc'].append(req.use_guess)
        if self.reloc_mode == 'always':
            ok = True
        elif self.reloc_mode == 'fallback':
            ok = not req.use_guess  # seed fails, global acquire succeeds
        else:
            ok = False
        r = RelocalizeResponse()
        r.success = ok
        r.message = 'relocalized' if ok else 'rejected'
        r.locked_pose.orientation.w = 1.0
        r.locked_pose.position.x = 7.0          # the to-side pose a commit reads back
        r.quality.inlier_ratio = 0.9 if ok else 0.1
        r.quality.accepted = ok
        return r

    def _begin(self, _req):
        self.calls['begin'] += 1
        return TriggerResponse(success=True, message='incremental')

    def _save(self, req):
        # The manager passes geometry.tmp/; write a backend-style manifest there (the
        # backend OWNS manifest.yaml; the manager adds lineage.yaml then commits).
        self.calls['save'].append(req.path)
        os.makedirs(os.path.join(req.path, 'keyframes'), exist_ok=True)
        with open(os.path.join(req.path, 'manifest.yaml'), 'w') as fh:
            fh.write('format_version: 1\nnum_keyframes: 1\nkeyframe_voxel: 0.25\n')
        return SaveMapResponse(success=True, message='saved')

    def _nav_exec(self, goal):
        self.legs.append(goal.target_pose.pose)
        result = NavigateToResult()
        if self.nav_ok:
            result.success = True
            result.outcome = NavigateToResult.OUTCOME_SUCCEEDED
            self._nav.set_succeeded(result)
        else:
            result.success = False
            result.outcome = NavigateToResult.OUTCOME_ABORTED_NO_PATH
            self._nav.set_aborted(result)

    # ---- helpers ----
    def _publish_kf(self, x, y, z=0.0):
        arr = KeyframePoseArray()
        arr.header.frame_id = 'map'
        kp = KeyframePose()
        kp.id = 0
        kp.pose.position.x = x
        kp.pose.position.y = y
        kp.pose.position.z = z
        kp.pose.orientation.w = 1.0
        arr.poses.append(kp)
        self._kf_pub.publish(arr)

    def _state(self):
        return rospy.ServiceProxy('/map_manager/get_active_map', GetActiveMap)().state

    def _topo_dir(self):
        return os.path.join(os.path.dirname(self.maps_root), 'topology')

    def _topo_path(self):
        return os.path.join(self._topo_dir(), 'topology.yaml')

    def _geometry_dir(self, name):
        return os.path.join(self.maps_root, name, 'geometry')

    def _lineage(self, name):
        with open(os.path.join(self._geometry_dir(name), 'lineage.yaml')) as fh:
            return yaml.safe_load(fh)

    def _snapshots(self, name):
        d = os.path.join(self.maps_root, name, 'snapshots')
        return sorted(os.listdir(d)) if os.path.isdir(d) else []

    def _nav_points(self):
        return rospy.wait_for_message('/map_manager/nav_points', NavPointArray,
                                      timeout=5.0)

    def _read_nav_yaml(self, name):
        with open(os.path.join(self.maps_root, name, 'overlays',
                               'nav_points.yaml')) as fh:
            return fh.read()

    def _seed_map(self, name):
        # A minimal valid map in the new layout: geometry/ with a backend manifest +
        # a manager lineage sidecar.
        g = self._geometry_dir(name)
        os.makedirs(os.path.join(g, 'keyframes'), exist_ok=True)
        with open(os.path.join(g, 'manifest.yaml'), 'w') as fh:
            fh.write('format_version: 1\nnum_keyframes: 1\n')
        with open(os.path.join(g, 'lineage.yaml'), 'w') as fh:
            fh.write('name: {}\nreason: imported\nparent: ""\nlabel: seed\n'.format(name))

    def _seed_topology_m1_m2(self):
        # pre-seed pose_in_to.x=99.0 (distinct from the mock's locked 7.0) so a later
        # capture+save is provably what replaced it.
        topo = {'version': 1, 'maps': ['m1', 'm2'], 'gateways': [{
            'from_map': 'm1', 'to_map': 'm2',
            'pose_in_from': {'position': {'x': 3.0, 'y': 0.0, 'z': 0.0},
                             'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1.0}},
            'pose_in_to': {'position': {'x': 99.0, 'y': 0.0, 'z': 0.0},
                           'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1.0}},
            'label': 'door', 'source': 'captured'}]}
        os.makedirs(self._topo_dir(), exist_ok=True)
        with open(self._topo_path(), 'w') as fh:
            yaml.safe_dump(topo, fh)
        rospy.ServiceProxy('/map_manager/reload_topology', Trigger)()

    def _navigate(self, **kw):
        client = self._client
        g = NavigateToGoal()
        g.goal_type = kw.get('goal_type', NavigateToGoal.GOAL_POSE)
        g.map_name = kw.get('map_name', '')
        g.nav_point_name = kw.get('nav_point_name', '')
        g.transition_wait = kw.get('transition_wait', -1.0)
        g.target_pose.header.frame_id = 'map'
        g.target_pose.pose.position.x = kw.get('x', 0.0)
        g.target_pose.pose.position.y = kw.get('y', 0.0)
        g.target_pose.pose.orientation.w = 1.0
        client.send_goal_and_wait(g, rospy.Duration(kw.get('timeout', 20.0)))
        return client.get_state()

    # ---- lifecycle + nav points + navigation, in order ----
    def test_everything(self):
        # 1-2. mapping + author a nav point at the current pose (x~1.0).
        self.assertTrue(
            rospy.ServiceProxy('/map_manager/start_mapping', StartMapping)('m1').success)
        add = rospy.ServiceProxy('/map_manager/add_nav_point', AddNavPoint)
        res = add(name='base', map_name='', use_current_pose=True, overwrite=False,
                  source=1)
        self.assertTrue(res.success, res.message)
        self.assertEqual(res.point.anchor_keyframe_id, 0)
        self.assertAlmostEqual(res.point.pose.position.x, 1.0, places=5)

        q = rospy.ServiceProxy('/map_manager/query_nav_point', QueryNavPoint)
        self.assertTrue(q(query='base', map_name='').found)

        # a second authoring write rolls the prior nav_points.yaml into overlays/.history.
        self.assertTrue(add(name='base2', map_name='', use_current_pose=True,
                            overwrite=False, source=1).success)
        hist = os.path.join(self.maps_root, 'm1', 'overlays', '.history')
        self.assertTrue(os.path.isdir(hist))
        self.assertTrue(any(f.startswith('nav_points.') for f in os.listdir(hist)))

        # re-optimization tracking.
        self._publish_kf(5.0, 5.0)
        rospy.sleep(0.3)
        self.assertAlmostEqual(q(query='base', map_name='').point.pose.position.x, 6.0,
                               places=5)
        nav_yaml_before = self._read_nav_yaml('m1')

        # stop + save -> geometry/ + lineage(reason=initial).
        res = rospy.ServiceProxy('/map_manager/stop_mapping', StopMapping)(True, '', '')
        self.assertTrue(res.success, res.message)
        self.assertEqual(res.saved_label, 'initial')
        self.assertTrue(os.path.isfile(
            os.path.join(self._geometry_dir('m1'), 'manifest.yaml')))
        self.assertEqual(self._lineage('m1')['reason'], 'initial')

        # content editing (active=m1): set_metadata writes overlays/metadata.yaml; the
        # backend-owned geometry/manifest.yaml stays byte-identical (non-destructive).
        manifest = os.path.join(self._geometry_dir('m1'), 'manifest.yaml')
        with open(manifest, 'w') as fh:
            fh.write('format_version: 1\nnum_keyframes: 1\n')
        manifest_before = open(manifest).read()
        rospy.sleep(0.3)  # let the editor see active_map=m1 via /map_manager/state
        me = rospy.ServiceProxy('/map_edit/command', MapEditCommand)
        self.assertTrue(me(MapEditCommandRequest(
            command='set_metadata', map_name='m1', key='description',
            value='north wing')).success)
        self.assertTrue(me(MapEditCommandRequest(command='save',
                                                 map_name='m1')).success)
        self.assertTrue(os.path.exists(
            os.path.join(self.maps_root, 'm1', 'overlays', 'metadata.yaml')))
        self.assertEqual(open(manifest).read(), manifest_before)

        # localization read-only: authoring refused, file untouched.
        self.assertTrue(rospy.ServiceProxy('/map_manager/start_localization',
                                           StartLocalization)('m1').success)
        res = add(name='base3', map_name='', use_current_pose=True, overwrite=False,
                  source=1)
        self.assertFalse(res.success)
        self.assertIn('read-only', res.message)
        self.assertEqual(self._read_nav_yaml('m1'), nav_yaml_before)

        # incremental: auto pre-incremental snapshot, then begin.
        self.assertTrue(rospy.ServiceProxy('/map_manager/start_incremental',
                                           StartIncremental)('m1').success)
        self.assertEqual(self.calls['begin'], 1)
        self.assertEqual(self._state().mode, MapManagerState.MODE_INCREMENTAL)
        self.assertTrue(any('pre-incremental' in s for s in self._snapshots('m1')))

        # incremental save -> new geometry/, lineage(reason=incremental, parent set).
        res = rospy.ServiceProxy('/map_manager/stop_mapping', StopMapping)(True, '', '')
        self.assertTrue(res.success, res.message)
        self.assertEqual(res.saved_label, 'increment')
        lin = self._lineage('m1')
        self.assertEqual(lin['reason'], 'incremental')
        self.assertTrue(lin['parent'])  # the pre-incremental snapshot id

        # ---- snapshot service: create / list / restore / delete ----
        snap = rospy.ServiceProxy('/map_manager/snapshot', MapSnapshot)
        r = snap(MapSnapshotRequest(name='m1', op='create', label='milestone one'))
        self.assertTrue(r.success, r.message)
        mid = r.snapshots[0].id
        r = snap(MapSnapshotRequest(name='m1', op='list'))
        self.assertTrue(any(s.id == mid for s in r.snapshots))
        r = snap(MapSnapshotRequest(name='m1', op='restore', snapshot_id=mid))
        self.assertTrue(r.success, r.message)
        r = snap(MapSnapshotRequest(name='m1', op='delete', snapshot_id=mid))
        self.assertTrue(r.success, r.message)
        r = snap(MapSnapshotRequest(name='m1', op='list'))
        self.assertFalse(any(s.id == mid for s in r.snapshots))

        # ---- same-map navigation ----
        self._client = actionlib.SimpleActionClient('/map_nav/navigate_to',
                                                    NavigateToAction)
        self.assertTrue(self._client.wait_for_server(rospy.Duration(10.0)))

        self.legs = []
        self.assertEqual(self._navigate(x=2.0), GoalStatus.SUCCEEDED)
        self.assertEqual(len(self.legs), 1)  # one leg, no gateway
        self.assertAlmostEqual(self.legs[0].position.x, 2.0, places=5)

        # GOAL_NAV_POINT resolves "base" (x=6.0 after the shift) then delegates.
        self.legs = []
        self.assertEqual(
            self._navigate(goal_type=NavigateToGoal.GOAL_NAV_POINT,
                           nav_point_name='base'), GoalStatus.SUCCEEDED)
        self.assertEqual(len(self.legs), 1)
        self.assertAlmostEqual(self.legs[0].position.x, 6.0, places=5)

        # ---- cross-map navigation ----
        self._seed_map('m2')
        # seed the m1 <-> m2 gateway directly (cross-map nav cases below need an edge).
        self._seed_topology_m1_m2()

        self.assertTrue(os.path.exists(self._topo_path()))
        gt = rospy.ServiceProxy('/map_manager/get_topology', GetTopology)()
        self.assertIn('m1', gt.maps)
        self.assertEqual(len(gt.edges), 1)
        nb = rospy.ServiceProxy('/map_manager/get_map_neighbors',
                                GetMapNeighbors)('m1')
        self.assertTrue(nb.success)
        self.assertEqual(len(nb.neighbors), 1)
        self.assertEqual(nb.neighbors[0].to_map, 'm2')

        # (a) seed hit: m1 -> m2, dwell then switch (load m2 read-only + seed reloc).
        self.legs, self.calls['load'], self.calls['reloc'] = [], [], []
        self.reloc_mode = 'always'
        self.assertEqual(
            self._navigate(map_name='m2', x=9.0, transition_wait=0.3),
            GoalStatus.SUCCEEDED)
        self.assertEqual(len(self.legs), 2)  # crossing leg (m1) + final leg (m2)
        self.assertAlmostEqual(self.legs[0].position.x, 3.0, places=5)  # depart pose
        self.assertAlmostEqual(self.legs[1].position.x, 9.0, places=5)  # user goal
        self.assertTrue(any(ro for (_p, ro) in self.calls['load']))     # m2 read-only
        self.assertGreaterEqual(len(self.calls['reloc']), 1)
        self.assertEqual(self._state().active_map, 'm2')

        # (b) seed miss -> global fallback locks. Now m2 -> m1.
        self.legs, self.calls['reloc'] = [], []
        self.reloc_mode = 'fallback'
        self.assertEqual(
            self._navigate(map_name='m1', x=0.5, transition_wait=0.2),
            GoalStatus.SUCCEEDED)
        self.assertTrue(any(ug for ug in self.calls['reloc']))       # seed tried
        self.assertTrue(any(not ug for ug in self.calls['reloc']))   # global fallback
        self.assertEqual(self._state().active_map, 'm1')

        # (c) reloc never locks -> safe abort. m1 -> m2.
        self.legs, self.calls['reloc'] = [], []
        self.reloc_mode = 'never'
        self.assertEqual(
            self._navigate(map_name='m2', x=9.0, transition_wait=0.2, timeout=25.0),
            GoalStatus.ABORTED)

        # ---- automatic gateway capture (begin in m1, load m2, commit, save) ----
        self.reloc_mode = 'always'
        self.assertTrue(rospy.ServiceProxy('/map_manager/start_localization',
                                           StartLocalization)('m1').success)
        gcap = rospy.ServiceProxy('/map_manager/gateway_capture', GatewayCapture)
        self.calls['load'] = []
        r = gcap(GatewayCaptureRequest(command='begin', to_map='m2', label='door'))
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.state, 'begun')
        r = gcap(GatewayCaptureRequest(command='load'))
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.state, 'loaded')
        self.assertTrue(any(ro for (_p, ro) in self.calls['load']))   # m2 loaded read-only
        r = gcap(GatewayCaptureRequest(command='commit'))
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.state, 'staged')
        self.assertTrue(r.quality_to.accepted)
        self.assertAlmostEqual(r.staged.pose_in_to.position.x, 7.0, places=5)  # captured
        r = gcap(GatewayCaptureRequest(command='save'))
        self.assertTrue(r.success, r.message)

        # the captured edge REPLACED the pre-seed (dedup): still one edge, now x=7.0.
        with open(self._topo_path()) as fh:
            topo = yaml.safe_load(fh)
        self.assertEqual(len(topo['gateways']), 1)
        self.assertEqual(topo['gateways'][0]['source'], 'captured')
        self.assertAlmostEqual(topo['gateways'][0]['pose_in_to']['position']['x'], 7.0,
                               places=5)

        # discard clears a pending capture; begin is rejected outside LOCALIZATION.
        self.assertTrue(rospy.ServiceProxy('/map_manager/start_localization',
                                           StartLocalization)('m1').success)
        self.assertTrue(gcap(GatewayCaptureRequest(command='begin', to_map='m2')).success)
        rd = gcap(GatewayCaptureRequest(command='discard'))
        self.assertTrue(rd.success)
        self.assertEqual(rd.state, 'idle')

        self.assertTrue(rospy.ServiceProxy('/map_manager/start_mapping',
                                           StartMapping)('m3').success)  # MAPPING mode
        rej = gcap(GatewayCaptureRequest(command='begin', to_map='m2'))
        self.assertFalse(rej.success)                                    # rejected: not LOCALIZATION


if __name__ == '__main__':
    rostest.rosrun(PKG, 'map_manager_integration', MapManagerIntegration)
