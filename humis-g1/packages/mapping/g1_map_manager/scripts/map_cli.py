#!/usr/bin/env python3
"""Thin CLI over the g1_map_manager services. No logic; just service calls.

Usage:
  map_cli.py list
  map_cli.py active | status
  map_cli.py start-mapping NAME
  map_cli.py stop [--save]
  map_cli.py start-loc NAME
  map_cli.py start-inc NAME
  map_cli.py add-point NAME
  map_cli.py snapshot {create|restore|list|delete} [--name N] [--label L] [--id ID]
  map_cli.py history NAME                   # readable lineage: current + snapshots
  map_cli.py {undo|redo|discard|save}      # content editor (map_edit/command)
  map_cli.py gateway-begin TO_MAP [--label L]   # capture from-side pose in the current map
  map_cli.py gateway-load                       # load TO_MAP read-only (operator-timed)
  map_cli.py gateway-commit                     # global relocalize + stage the captured edge
  map_cli.py gateway-nudge DX DY | gateway-yaw DYAW   # adjust the staged seed
  map_cli.py {gateway-save|gateway-discard|gateway-status}
  map_cli.py gateway-capture TO_MAP [--label L] # one-stop: begin -> load -> commit
"""
import argparse
import sys

import rospy
from g1_msgs.srv import (ListMaps, GetActiveMap, StartMapping, StopMapping,
                         StartLocalization, StartIncremental, AddNavPoint,
                         AddNavPointRequest, MapEditCommand, MapEditCommandRequest,
                         GatewayCapture, GatewayCaptureRequest,
                         MapSnapshot, MapSnapshotRequest)

M = "/map_manager/"


def call(name, srv_type, *args):
    rospy.wait_for_service(name, timeout=5.0)
    return rospy.ServiceProxy(name, srv_type)(*args)


def gateway(command, **kw):
    req = GatewayCaptureRequest(command=command, to_map=kw.get('to_map', ''),
                                label=kw.get('label', ''), values=kw.get('values', []))
    r = call(M + "gateway_capture", GatewayCapture, req)
    qt = r.quality_to
    print(f"[{r.state}] {r.message}"
          + (f" | to-side inlier={qt.inlier_ratio:.2f} accepted={qt.accepted}"
             if command == 'commit' else ""))
    return r


def build_parser():
    p = argparse.ArgumentParser(description="g1_map_manager CLI")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    sub.add_parser("active")
    sub.add_parser("status")
    sub.add_parser("start-mapping").add_argument("name")
    sub.add_parser("stop").add_argument("--save", action="store_true")
    sub.add_parser("start-loc").add_argument("name")
    sub.add_parser("start-inc").add_argument("name")
    sub.add_parser("add-point").add_argument("name")
    snp = sub.add_parser("snapshot")
    snp.add_argument("op", choices=["create", "restore", "list", "delete"])
    snp.add_argument("--name", default="")
    snp.add_argument("--label", default="")
    snp.add_argument("--id", default="", dest="snapshot_id")
    sub.add_parser("history").add_argument("name")
    for verb in ("undo", "redo", "discard", "save"):
        sub.add_parser(verb)
    gb = sub.add_parser("gateway-begin")
    gb.add_argument("to_map")
    gb.add_argument("--label", default="")
    sub.add_parser("gateway-load")
    sub.add_parser("gateway-commit")
    gn = sub.add_parser("gateway-nudge")
    gn.add_argument("dx", type=float)
    gn.add_argument("dy", type=float)
    gy = sub.add_parser("gateway-yaw")
    gy.add_argument("dyaw", type=float)
    sub.add_parser("gateway-save")
    sub.add_parser("gateway-discard")
    sub.add_parser("gateway-status")
    gc = sub.add_parser("gateway-capture")
    gc.add_argument("to_map")
    gc.add_argument("--label", default="")
    return p


def main():
    args = build_parser().parse_args()
    rospy.init_node("map_cli", anonymous=True, disable_signals=True)

    if args.cmd == "list":
        for m in call(M + "list_maps", ListMaps).maps:
            print(f"{m.name}  label='{m.label}'  {m.num_keyframes}kf  "
                  f"snapshots={m.num_snapshots}")
    elif args.cmd in ("active", "status"):
        s = call(M + "get_active_map", GetActiveMap).state
        print(f"mode={s.mode} map='{s.active_map}' label='{s.current_label}' "
              f"localized={s.localized} transition={s.transition} :: {s.detail}")
    elif args.cmd == "start-mapping":
        print(call(M + "start_mapping", StartMapping, args.name).message)
    elif args.cmd == "stop":
        r = call(M + "stop_mapping", StopMapping, args.save, "", "")
        print(r.message, r.saved_label)
    elif args.cmd == "snapshot":
        req = MapSnapshotRequest(name=args.name, op=args.op, label=args.label,
                                 snapshot_id=args.snapshot_id)
        r = call(M + "snapshot", MapSnapshot, req)
        print(r.message)
        for s in r.snapshots:
            print(f"  {s.id}  {s.num_keyframes}kf  reason={s.reason} "
                  f"parent={s.parent or '-'}  {s.label}")
    elif args.cmd == "history":
        cur = next((m for m in call(M + "list_maps", ListMaps).maps
                    if m.name == args.name), None)
        if cur:
            print(f"current   {cur.num_keyframes}kf  {cur.created_at}  "
                  f"\"{cur.label}\"  ({cur.num_snapshots} snapshots)")
        snaps = call(M + "snapshot", MapSnapshot,
                     MapSnapshotRequest(name=args.name, op="list")).snapshots
        for s in sorted(snaps, key=lambda e: e.id, reverse=True):
            print(f"snapshot  {s.num_keyframes}kf  {s.created_at}  "
                  f"\"{s.label}\"  reason={s.reason} parent={s.parent or '-'}  [{s.id}]")
    elif args.cmd == "start-loc":
        print(call(M + "start_localization", StartLocalization, args.name).message)
    elif args.cmd == "start-inc":
        print(call(M + "start_incremental", StartIncremental, args.name).message)
    elif args.cmd == "add-point":
        req = AddNavPointRequest(name=args.name, use_current_pose=True, source=1)
        print(call(M + "add_nav_point", AddNavPoint, req).message)
    elif args.cmd in ("undo", "redo", "discard", "save"):
        req = MapEditCommandRequest(command=args.cmd)
        print(call("/map_edit/command", MapEditCommand, req).message)
    elif args.cmd == "gateway-begin":
        gateway("begin", to_map=args.to_map, label=args.label)
    elif args.cmd == "gateway-load":
        gateway("load")
    elif args.cmd == "gateway-commit":
        gateway("commit")
    elif args.cmd == "gateway-nudge":
        gateway("nudge", values=[args.dx, args.dy])
    elif args.cmd == "gateway-yaw":
        gateway("yaw", values=[args.dyaw])
    elif args.cmd in ("gateway-save", "gateway-discard", "gateway-status"):
        gateway(args.cmd.split("-", 1)[1])
    elif args.cmd == "gateway-capture":   # one-stop convenience: begin -> load -> commit
        gateway("begin", to_map=args.to_map, label=args.label)
        gateway("load")
        gateway("commit")
        print("review in RViz, then: map_cli.py gateway-save (or gateway-discard)")
    else:
        print("unknown command", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
