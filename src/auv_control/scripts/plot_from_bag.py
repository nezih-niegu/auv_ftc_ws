#!/usr/bin/env python3
# ============================================================================
#  plot_from_bag.py
#
#  Reads a rosbag2 recording produced during a scenario run and draws
#  paper-style figures mirroring Figures 5-11 of Zhang et al., Sensors 2024,
#  24, 3029.
#
#  Record the topics first:
#      ros2 bag record -o run1  /auv/virtual_u /auv/tau_des \
#                               /auv/tau_actual /auv/fault_status
#
#  Then generate the plots:
#      ros2 run auv_control plot_from_bag.py --bag run1 --out figs/
#
#  Requires:  rosbag2_py (ROS 2 standard), matplotlib, numpy.
# ============================================================================
import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray
except ImportError as exc:
    print(f"[plot_from_bag] missing dependency: {exc}", file=sys.stderr)
    sys.exit(1)


TOPICS = {
    "/auv/virtual_u":     "virtual_u",
    "/auv/tau_des":       "tau_des",
    "/auv/tau_actual":    "tau_actual",
    "/auv/fault_status":  "fault_status",
}


def read_bag(bag_path):
    opts = StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv = ConverterOptions("cdr", "cdr")
    reader = SequentialReader()
    reader.open(opts, conv)

    # topic -> (t_ns list, N-element vector list)
    data = {name: ([], []) for name in TOPICS.values()}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        msg = deserialize_message(raw, Float64MultiArray)
        key = TOPICS[topic]
        data[key][0].append(t_ns)
        data[key][1].append(list(msg.data))

    # Convert to numpy arrays with t in seconds, origin at first sample.
    out = {}
    t0 = None
    for key, (ts, vs) in data.items():
        if not ts:
            continue
        t0 = ts[0] if t0 is None else min(t0, ts[0])

    for key, (ts, vs) in data.items():
        if not ts:
            out[key] = (np.array([]), np.zeros((0, 0)))
            continue
        t = (np.array(ts, dtype=float) - t0) * 1e-9
        v = np.array(vs, dtype=float)
        out[key] = (t, v)
    return out


def plot_fig5(data, outdir):
    """Figure 5 — four virtual input channels u1..u4."""
    if "virtual_u" not in data or data["virtual_u"][1].size == 0:
        print("[plot_from_bag] no virtual_u data — skipping Fig 5")
        return
    t, U = data["virtual_u"]
    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, U[:, i], lw=1.2)
        ax.set_ylabel(f"$u_{{{i+1}}}$")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Simulation time [s]")
    fig.suptitle("Fig 5 — Input signal for fuzzy controller")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig05_virtual_u.png"), dpi=140)
    plt.close(fig)


def plot_fig67(data, outdir, which):
    """Figures 6-7 — desired vs. actual wrench components.

    which='force'   -> plot tau_x  (surge force, Fig 6)
    which='moment'  -> plot tau_n  (yaw moment, Fig 7)
    """
    if "tau_des" not in data or data["tau_des"][1].size == 0:
        return
    td, TD = data["tau_des"]
    ta, TA = data["tau_actual"]
    idx    = 0 if which == "force" else 4
    label  = r"$\tau_x$ (surge force)" if which == "force" else r"$\tau_n$ (yaw moment)"
    name   = "fig06_force" if which == "force" else "fig07_moment"

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(td, TD[:, idx], "b-",  lw=1.2, label="desired (fault-tolerant)")
    ax.plot(ta, TA[:, idx], "r--", lw=1.2, label="actual")
    # Mark the fault injection time used by scenario_runner.
    ax.axvline(150.0, color="gray", ls=":", lw=1.0, alpha=0.8, label="fault @150s")
    ax.set_xlabel("Simulation time [s]")
    ax.set_ylabel(label)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(f"Fig {'6' if which == 'force' else '7'} — {label}")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"{name}.png"), dpi=140)
    plt.close(fig)


def plot_fault_status(data, outdir):
    if "fault_status" not in data or data["fault_status"][1].size == 0:
        return
    t, F = data["fault_status"]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for i in range(F.shape[1]):
        ax.plot(t, F[:, i], lw=1.5, label=f"$f_{{{i+1}}}$")
    ax.set_xlabel("Simulation time [s]")
    ax.set_ylabel("fault factor $f_i$")
    ax.set_ylim(-0.05, 1.1)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title("Fault factors $f_i$ over the run")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "fig_fault_status.png"), dpi=140)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True, help="rosbag2 directory")
    p.add_argument("--out", default="figs", help="output directory for PNGs")
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print(f"[plot_from_bag] reading {args.bag}")
    data = read_bag(args.bag)
    for key, (t, v) in data.items():
        print(f"  {key}: {len(t)} samples, shape {v.shape}")

    plot_fig5(data, args.out)
    plot_fig67(data, args.out, "force")
    plot_fig67(data, args.out, "moment")
    plot_fault_status(data, args.out)
    print(f"[plot_from_bag] figures written to {args.out}/")


if __name__ == "__main__":
    main()
