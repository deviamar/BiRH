"""``morpholab run`` -- one grasp trial, with its full result row."""

from __future__ import annotations

import argparse
import json


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--hand", default="birh_v1", help="preset name or morphology JSON path")
    parser.add_argument("--object", dest="obj", default="sphere_medium")
    parser.add_argument("--task", default="lift")
    parser.add_argument("--controller", default="ramp")
    parser.add_argument("--trial", type=int, default=0)
    parser.add_argument("--robustness", action="store_true",
                        help="also run the disturbance battery (several times the cost)")
    parser.add_argument("--trace", metavar="PATH", help="save the episode trace")
    parser.add_argument("--video", metavar="PATH", help="render the trial to mp4/gif")
    parser.add_argument("--json", action="store_true", help="print the raw result row")


def run(args: argparse.Namespace) -> int:
    from ..control import get_controller
    from ..metrics import MetricContext, base_row, evaluate_metrics
    from ..objects import get_object
    from ..sim import Rollout, build_scene
    from ..tasks import get_task
    from ..experiments.runner import _resolve_morphology

    hand = _resolve_morphology(args.hand, None)
    obj = get_object(args.obj)
    task = get_task(args.task)
    scene = build_scene(hand, obj, approach=task.approach, standoff=task.standoff)
    rollout = Rollout(scene, task, get_controller(args.controller))
    trace = rollout.run()

    ctx = MetricContext(trace=trace, hand=hand, obj=obj, task=task, scene=scene,
                        rollout=rollout if args.robustness else None)
    row = {**base_row(ctx), **evaluate_metrics(ctx, include_sim=args.robustness)}

    if args.json:
        print(json.dumps(row, indent=2, sort_keys=True, default=str))
        return 0 if row["success"] else 1

    print(f"{hand.name} x {obj.name} x {task.name} ({args.controller})")
    print(f"  outcome     {row['outcome']}" + (f"  [{row['failure_mode']}]" if not row["success"] else ""))
    if row.get("failure_detail"):
        print(f"              {row['failure_detail']}")
    print(f"  approach    palm z={scene.initial_wrist.pos[2] * 100:.1f} cm, "
          f"preshape={scene.meta.get('preshape', 0):.2f}, "
          f"grasp depth={scene.meta.get('approach_plan', {}).get('grasp_depth', 0) * 100:.1f} cm")
    print(f"  grasp       {row['contact_count']} contacts on {row['contact_digits']} digits, "
          f"spread={row['contact_spread']:.2f}, opposition={row['opposition']:.2f}")
    print(f"  quality     epsilon={row['grasp_epsilon']:.3f}, volume={row['grasp_volume']:.3f}, "
          f"closure={row['force_closure']}, isotropy={row['grasp_isotropy']:.3f}")
    print(f"  outcome     lift={row['lift_height'] * 100:.1f} cm, slip={row['slip_distance'] * 100:.2f} cm, "
          f"rotation={row['slip_rotation']:.3f} rad")
    print(f"  effort      peak torque={row['peak_torque']:.3f} N.m, energy={row['energy']:.3f} J, "
          f"peak force={row['peak_normal_force']:.1f} N")
    if args.robustness and "disturbance_worst" in row:
        print(f"  robustness  worst={row['disturbance_worst']:.2f} N, mean={row['disturbance_mean']:.2f} N, "
              f"anisotropy={row['disturbance_anisotropy']:.2f}, gravity x{row['gravity_multiplier']:.1f}")
    print(f"  complexity  {row['complexity']:.2f} ({row['n_actuators']} actuators, {row['n_joints']} joints)")
    print(f"  cost        {trace.steps} steps, {trace.wall_time * 1000:.0f} ms")
    if row.get("force_implausible"):
        print("  WARNING     contact forces exceed what the actuators could produce; the object "
              "is probably reacting against the support or wedged")

    if args.trace:
        print(f"  trace       {trace.save(args.trace)}")
    if args.video:
        from ..viz.video import record_episode

        print(f"  video       {record_episode(rollout, task, path=args.video)}")
    return 0 if row["success"] else 1
