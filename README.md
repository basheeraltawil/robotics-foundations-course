# Robotics Foundations — Module 1: Kinematics (Free)

**Learn robot kinematics by running it.** Rotation matrices, quaternions, forward and inverse kinematics, the Jacobian and singularities, explained with verified, runnable Python notebooks and short animated video lessons.

This repository is the free first module of **Robotics Foundations**, a hands-on robotics course by **AIBOMECH** (AI · Robotics · Mechatronics). Every number in the lessons is computed and checked in code, so you can see the result change when you change the input.

> **Coming next:** Dynamics · Probability & SLAM · ROS 2, Simulation & NVIDIA Isaac · Motion Planning. See [Coming modules](#coming-modules).

[![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/YOUR_USERNAME/YOUR_REPO/blob/main/Module_1_Kinematics/kinematics_lab.ipynb)
![Python](https://img.shields.io/badge/Python-3.x-blue)
![NumPy](https://img.shields.io/badge/NumPy-Matplotlib-orange)
![Module 1](https://img.shields.io/badge/Module%201-Free-brightgreen)

---

## What you will learn

| Section | Topic | What you compute |
|---|---|---|
| 1 | Rotation matrices, SO(3) | Build R_z(45°), check RᵀR = I and det(R) = +1 |
| 2 | Quaternions and gimbal lock | The quaternion for the same turn, and why Euler interpolation wobbles while SLERP does not |
| 3 | Forward kinematics | Hand position of a 2-link arm: (30°, 45°) → (0.537, 0.636) m |
| 4 | Inverse kinematics | One target, two answers: (13.5°, +60°) and (66.1°, −60°), both verified by FK |
| 5 | The Jacobian and singularities | det(J) = 0.141 at the example pose; damped least squares vs the plain inverse near a singularity |
| 6 | Screw theory | Product of exponentials: the matrix exponential of a twist gives the rotation |

By the end you can:

- choose the right orientation representation (think in Euler, code in quaternion, debug in Euler),
- solve forward and inverse kinematics for a 2-link arm by hand and in code,
- read `det(J)` and tell when an arm is close to a singularity,
- explain why MoveIt 2 and TRAC-IK use damped least squares.

---

## Watch the lessons

Each notebook section has a short video (about 4 minutes) on the AIBOMECH YouTube channel:

| Part | Video |
|---|---|
| 1 | Which Way Is It Facing? Rotation matrices, Euler angles and quaternions — [watch](https://www.youtube.com/watch?v=LjzKx1PuxJY) |
| 2 | From Joint Angles to Position (forward kinematics) — [watch](https://www.youtube.com/watch?v=ZADfwUZO0EA) |
| 3 | One Target, Two Answers (inverse kinematics) — [watch](https://www.youtube.com/watch?v=OlKaNiCmJjQ) |
| 4 | The Jacobian and the Safety Check (singularities) — [watch](https://www.youtube.com/watch?v=Ft3E8gUwGzA) |
| 5 | Screw Theory and Module Wrap-Up — coming soon |

Full playlist: [Robotics Fundamentals on YouTube](https://studio.youtube.com/playlist/PLF17wat5284k/videos)

---

## Quick start

**In the browser:** click the *Open in Colab* badge above. Nothing to install.

**On your machine:**

```bash
git clone https://github.com/YOUR_USERNAME/YOUR_REPO.git
cd YOUR_REPO
pip install numpy matplotlib jupyter
jupyter notebook Module_1_Kinematics/kinematics_lab.ipynb
```

No GPU or robot hardware needed. Prerequisites: basic Python and linear algebra (vectors, matrices).

---

## Repository layout

```
Module_1_Kinematics/
├── kinematics_lab.ipynb              # the lab: 6 sections, every result checked with assert
├── gimbal_lock_demo_corrected.py     # stand-alone gimbal-lock demo (Euler vs SLERP)
├── figures/                          # the plots the notebook produces
└── slides/                           # lecture slides (coming soon)
```

---

## Coming modules

Module 1 is free and open. The next modules are being released as part of the full **Robotics Foundations** course, in the same format: a verified lab notebook, slides, short video lessons and a real-world case study for every module.

### Module 2 — Dynamics: How Forces Cause Motion *(coming soon)*
- The mass matrix M(q) and the master equation of robot motion
- Gravity, Coriolis and the full joint-torque breakdown, and why gravity dominates at low speed
- What a wrong mass in your URDF actually does to the arm
- Friction cones and the grip force needed to hold an object
- Case study: the cobot that passed certification and still hurt someone

### Module 3 — Probability, Estimation & SLAM *(coming soon)*
- Bayes' theorem as the base of every estimator
- The Kalman filter built from zero, then the Extended Kalman Filter for real robot motion
- The particle filter and the kidnapped-robot problem
- Factor graphs and loop closure: the complete SLAM picture
- Case study: the warehouse robot that was confidently lost

### Module 4 — ROS 2, Simulation & NVIDIA Isaac *(coming soon)*
- ROS 2's four building blocks: topics, services, actions and parameters
- Gazebo vs Isaac Sim vs Isaac Gym, compared with numbers
- Closing the sim-to-real gap, and a full mobile-manipulator (TIAGo) software stack
- A miniature fetch-and-retrieve state machine, plus an 8-week implementation roadmap
- Case study: "it works in Gazebo" and the four gaps that make that meaningless

### Module 5 — Motion Planning & Optimization *(bonus, coming soon)*
- RRT*: motion planning that gets better with more time
- CHOMP-style trajectory optimization: gradient descent on a whole path
- TOPP-RA-style time parameterization: how fast, not just which way
- Case study: the pick-and-place cell that ran 40% slower than quoted

| Module | Topic | Status |
|---|---|---|
| 1 | Kinematics | ✅ Free — this repository |
| 2 | Dynamics | 🔜 Coming soon |
| 3 | Probability, estimation & SLAM | 🔜 Coming soon |
| 4 | ROS 2, simulation & NVIDIA Isaac | 🔜 Coming soon |
| 5 | Motion planning & optimization (bonus) | 🔜 Coming soon |

**Want the rest of the course?**
- **Get the full course:** [LINK]
- **Be notified when each module is released:** ⭐ star and 👁 watch this repository, and subscribe on [YouTube](LINK).

---

## About

Created by **Basheer Al-Tawil** · **AIBOMECH** — AI, Robotics, Mechatronics.

- YouTube: [AIBOMECH](LINK)
- Questions or ideas: open an issue in this repository.

If this helped you, a ⭐ on the repository helps others find it.

## License

Code: see `LICENSE`. Course text, slides and figures: all rights reserved unless stated otherwise.
