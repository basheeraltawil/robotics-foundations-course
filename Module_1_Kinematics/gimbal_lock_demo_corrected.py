"""
Module 1 - Kinematics: corrected gimbal-lock demo (companion to kinematics_lab.ipynb, Section 2)

Why this file exists
--------------------
The Section 2 cell in kinematics_lab.ipynb sweeps pitch from 60 to 120 deg with roll = yaw = 0.
That is a rotation about ONE fixed axis, so interpolating the Euler angles and quaternion SLERP
produce exactly the same motion. The "spikes" in figures/section2_gimbal_lock.png are
floating-point noise of about 2.5e-14 rad/step, and both printed variabilities are 0.000.

To make the difference real, roll and yaw must change too. Near pitch = 90 deg, roll and yaw
rotate about the same physical axis, so their changes partly cancel there and add up elsewhere.
Interpolating the three Euler angles linearly then gives an uneven angular speed and a longer
path, while SLERP takes the shortest path at constant speed.

Run:  python gimbal_lock_demo_corrected.py
Needs: numpy, matplotlib. Saves figures/section2_gimbal_lock_corrected.png
The helper functions below are copied unchanged from the notebook so this file runs on its own.
"""
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# ---- helpers, identical to kinematics_lab.ipynb Section 2 ----
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quaternion_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def slerp(q_start, q_end, t):
    dot = np.dot(q_start, q_end)
    if dot < 0:
        q_end, dot = -q_end, -dot
    dot = np.clip(dot, -1, 1)
    theta = np.arccos(dot) * t
    q_perp = q_end - q_start * dot
    q_perp /= (np.linalg.norm(q_perp) + 1e-12)
    return q_start * np.cos(theta) + q_perp * np.sin(theta)


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def angular_speed(orientations, dt=1.0):
    speeds = []
    for i in range(1, len(orientations)):
        q1, q2 = orientations[i-1], orientations[i]
        relative = quaternion_multiply(quaternion_conjugate(q1), q2)
        relative = relative / np.linalg.norm(relative)
        angle = 2 * np.arccos(np.clip(abs(relative[0]), -1, 1))
        speeds.append(angle / dt)
    return np.array(speeds)


# ---- the corrected experiment ----
def compare(rpy_start_deg, rpy_end_deg, n_steps=60):
    """Interpolate start -> end two ways and return the per-step angular speed of each."""
    start = np.radians(np.array(rpy_start_deg, dtype=float))
    end = np.radians(np.array(rpy_end_deg, dtype=float))
    ts = np.linspace(0, 1, n_steps)
    euler_path = [euler_to_quaternion(*(start + t * (end - start))) for t in ts]
    slerp_path = [slerp(euler_path[0], euler_path[-1], t) for t in ts]
    return angular_speed(euler_path), angular_speed(slerp_path)


def report(name, euler_speed, slerp_speed):
    print(f"\n{name}")
    print(f"  Euler speed variability (std/mean): {euler_speed.std() / euler_speed.mean():.3f}")
    print(f"  SLERP speed variability (std/mean): {slerp_speed.std() / slerp_speed.mean():.3f}")
    print(f"  Euler fastest / slowest step:       {euler_speed.max() / euler_speed.min():.2f}x")
    print(f"  Total rotation travelled:  Euler {np.degrees(euler_speed.sum()):.1f} deg"
          f"  vs  SLERP {np.degrees(slerp_speed.sum()):.1f} deg")


# Case A: same pitch sweep as the notebook (60 -> 120 deg, straight through 90),
#         but roll and yaw each turn 0 -> 90 deg as well.
euler_A, slerp_A = compare((0, 60, 0), (90, 120, 90))
report("Case A: rpy (0, 60, 0) -> (90, 120, 90) deg, through pitch = 90", euler_A, slerp_A)

# Case B: the project scenario's motion, gripper pointing forward -> straight down.
euler_B, slerp_B = compare((0, 0, 0), (90, 90, 90))
report("Case B: rpy (0, 0, 0) -> (90, 90, 90) deg, forward -> straight down", euler_B, slerp_B)

# Control: the notebook's original pure-pitch sweep. Both methods are identical here.
euler_0, slerp_0 = compare((0, 60, 0), (0, 120, 0))
print(f"\nControl (pitch only, as in the notebook): largest Euler-vs-SLERP speed difference = "
      f"{np.abs(euler_0 - slerp_0).max():.1e} rad/step  (numerical noise)")

assert euler_A.std() / euler_A.mean() > 0.05 and slerp_A.std() / slerp_A.mean() < 1e-6
assert euler_B.std() / euler_B.mean() > 0.05 and slerp_B.std() / slerp_B.mean() < 1e-6
assert euler_A.sum() > slerp_A.sum() and euler_B.sum() > slerp_B.sum()
print("\nCONFIRMED: once roll and yaw change too, Euler-angle interpolation moves at an uneven speed")
print("and travels further than SLERP, which keeps a constant angular speed along the shortest path.")

fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=False)
for ax, (es, ss, title) in zip(axes, [
    (euler_A, slerp_A, "Through pitch = 90 deg\nrpy (0, 60, 0) -> (90, 120, 90)"),
    (euler_B, slerp_B, "Forward -> straight down\nrpy (0, 0, 0) -> (90, 90, 90)"),
]):
    ax.plot(es, label="Naive Euler-angle interpolation", color="firebrick")
    ax.plot(ss, label="Quaternion SLERP", color="seagreen")
    ax.set_xlabel("interpolation step")
    ax.set_ylabel("angular speed (rad/step)")
    ax.set_ylim(0, max(es.max(), ss.max()) * 1.15)
    ax.set_title(title)
    ax.legend(loc="lower left")
fig.suptitle("Euler vs. SLERP when roll and yaw also change")
fig.tight_layout()

out = Path(__file__).resolve().parent / "figures" / "section2_gimbal_lock_corrected.png"
out.parent.mkdir(exist_ok=True)
fig.savefig(out, dpi=110, bbox_inches="tight")
print(f"Saved {out.name}")
