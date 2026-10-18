# Robotics Foundations — Module 1: Kinematics 

**Learn robot kinematics by running it.** Rotation matrices, quaternions, forward and inverse kinematics, the Jacobian and singularities, explained with verified, runnable Python notebooks and short animated video lessons.

This repository is the free first module of **Robotics Foundations**, a hands-on robotics course by **AIBOMECH** (AI · Robotics · Mechatronics). Every number in the lessons is computed and checked in code, so you can see the result change when you change the input.

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
| 1 | Which Way Is It Facing? Rotation matrices, Euler angles and quaternions — [watch](LINK) |
| 2 | From Joint Angles to Position (forward kinematics) — [watch](LINK) |
| 3 | One Target, Two Answers (inverse kinematics) — [watch](LINK) |
| 4 | The Jacobian and the Safety Check (singularities) — [watch](LINK) |
| 5 | Screw Theory and Module Wrap-Up — coming soon |

Full playlist: [Robotics Fundamentals on YouTube](LINK)

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

## The full course

Module 1 is free. The complete **Robotics Foundations** course continues with the same format: notebook, slides, video lessons and a real-world project scenario for every module.

| Module | Topic | Access |
|---|---|---|
| 1 | Kinematics: the geometry of motion | Free — this repository |
| 2 | Dynamics: how forces cause motion | Full course |
| 3 | Probability, estimation and SLAM | Full course |
| 4 | ROS 2, simulation and NVIDIA Isaac | Full course |
| 5 | Motion planning and optimization (bonus) | Full course |

**What the full course adds:** a lab notebook for every module, the complete slide decks, a practical project scenario per module, and the full set of video lessons.

**Get the full course:** [LINK]

---

## About

Created by **Basheer Al-Tawil** · **AIBOMECH** — AI, Robotics, Mechatronics.

- YouTube: [AIBOMECH](LINK)
- Questions or ideas: open an issue in this repository.

If this helped you, a ⭐ on the repository helps others find it.

## License

Code: see `LICENSE`. Course text, slides and figures: all rights reserved unless stated otherwise.
