# Module 2 — Dynamics: How Forces Cause Motion

*A practical implementation companion to "Robotics Foundations," Section 2*

## 0. Curriculum Metadata

*Standardized against `MASTER_CURRICULUM_MAP.md` — see there for how this module fits the full repository-wide dependency graph.*

- **Difficulty:** Intermediate
- **Estimated study time:** ~1 week (5-7 focused hours: README + notebook + `project_scenario.md`)
- **Software:** Python 3, NumPy, Matplotlib, Jupyter; SymPy optional if you want to redo the Lagrangian derivation symbolically instead of by hand.
- **Prerequisites (explicit, cross-program):** **Module 1 (Kinematics)** in this program — this module reuses Module 1's Jacobian, transposed, to map contact forces into joint torques. Calculus/linear algebra fundamentals from `DataScience_Robotics_Roadmap` Module 1.
- **Papers/books:** *Modern Robotics* — Lynch & Park, Ch. 8 (see `MASTER_BIBLIOGRAPHY.md` Section 1).
- **Learning objectives (Bloom's taxonomy):**
  - *Remember* — state the master equation `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ` and name each term.
  - *Understand* — explain why gravity compensation dominates low-speed robot control.
  - *Apply* — derive the mass matrix, gravity vector, and Coriolis terms for a 2-link arm via the Lagrangian, matching the notebook's reproduction of the source book's worked numbers.
  - *Analyze* — diagnose a wrong-URDF-mass bug from its symptoms alone (silent, unexplained steady-state drift with no error in the logs).
  - *Evaluate* — judge whether a given PID gain set is strong enough to mask a real modeling bug, and explain why that masking is dangerous.
  - *Create* — extend the Lagrangian derivation from a 2-link to a 3-link arm.
- **Forward/backward links:** Depends on **Module 1 (Kinematics)** in this program. It is extended by **Module 4 (ROS 2, Simulation & Isaac)** in this program, whose physics engines solve this exact master equation at every simulation step, and by **`6-Month_Program` Month 3**, which derives Lagrangian dynamics and the Jacobian directly for a real TIAGo arm.

## Kinematics vs. dynamics — the fundamental difference

Kinematics asks: if I rotate my shoulder 30° and bend my elbow 45°, where is my fingertip? Pure geometry, no forces. Dynamics asks the question that actually determines what your motors have to do: if I apply 5 Newton-meters of torque to my shoulder, how does it accelerate? How much force does my elbow feel? How long until I reach the cup?

**The shopping-cart analogy carries the whole module:** an empty cart has low inertia and is easy to accelerate; a full cart has high inertia, so the same push force produces much less acceleration (`F = ma`). Turning while pushing introduces Coriolis forces that push sideways. Going uphill, gravity resists your push even when you're not accelerating at all — you have to push just to hold still. Every one of these everyday phenomena has an exact mathematical counterpart inside your robot's controller.

## 2.1 — The master equation

Every robot's equation of motion, regardless of how many joints it has, takes this one form:

```
M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ
```

- **M(q)** — the mass/inertia matrix: how much the robot resists acceleration, and it *depends on configuration* (a stretched-out arm is harder to swing than a folded one, exactly like a figure skater's arms).
- **C(q,q̇)·q̇** — Coriolis and centrifugal terms: velocity-dependent coupling forces between joints.
- **g(q)** — the gravity vector: the torque needed just to hold the current position, with zero motion.
- **τ** — the joint torques your motors actually produce.

Two directions matter in practice: **inverse dynamics** (given `q, q̇, q̈`, find `τ`) is used for feedforward control — it's what tells the motors how hard to push. **Forward dynamics** (given `q, q̇, τ`, find `q̈`) is used for simulation — it's what Gazebo and Isaac Sim compute at every physics step.

## 2.2 — Lagrangian mechanics: deriving the master equation from energy

Newton's approach requires drawing every force on every link and writing `F=ma` for each — for a 7-DOF arm, that's dozens of forces to track by hand. Lagrange's approach tracks only energy, and the full equations of motion fall out automatically:

```
L = T - V                                    (kinetic energy minus potential energy)
τ_i = d/dt(∂L/∂q̇_i) - ∂L/∂q_i                (Euler-Lagrange equation, one per joint)
```

This module's notebook carries out the full derivation for a 2-link planar arm (`L1=0.5m, m1=2kg, L2=0.4m, m2=1kg`), producing the mass matrix, gravity vector, and Coriolis terms as explicit functions of joint configuration — and reproduces the book's exact worked numbers at every step.

## 2.3 — The most important result: gravity dominates at low speed

At the book's example configuration (`q1=30°, q2=45°`, with realistic joint velocities and accelerations), the joint-1 torque breaks down as:

| Term | Value | % of total | Physical meaning |
|---|---|---|---|
| Inertia `M·q̈` | 1.222 Nm | 11.9% | Force to accelerate the arm itself |
| Coupling `M12·q̈2` | 0.124 Nm | 1.2% | Joint 2's acceleration affects joint 1 |
| Coriolis `C` | -0.088 Nm | -0.9% | Velocity-dependent coupling (small here) |
| **Gravity `g(q)`** | **9.004 Nm** | **87.8%** | **Just holding the arm up against gravity** |

This is the single most important practical lesson in the module: **at low speed, gravity compensation is almost the entire control problem.** If your URDF has wrong link masses, `g(q)` is wrong, and the arm will visibly drift under gravity even when every other part of the controller is correct — this is the most common and hardest-to-diagnose bug in real robot deployment, precisely because everything else about the system can be working perfectly.

## 2.4 — Contact dynamics: friction cones and grasping

Press a finger against a table and push sideways gently — it doesn't slip. Push harder, and eventually it does. The **friction cone** defines exactly how hard you can push sideways before slipping, and every valid contact force must stay inside it:

```
||f_tangential|| ≤ μ · f_normal          (Coulomb friction model)
```

**Worked example — TIAGo gripping a 500g bottle:** lifting force required is `m·g = 4.905 N`. With a rubber-on-plastic friction coefficient `μ=0.5`, the minimum normal force each finger must apply is `F_lift/(2μ) = 4.905 N`; with a 2× safety factor, the actual commanded force is `9.81 N` per finger. **If the surface is wet (`μ=0.1`), that requirement jumps to 24.5 N** — nearly 2.5× more force needed for the identical object, purely because of a change in surface friction. This is exactly why gripper force is not a fixed constant in a well-designed grasping controller — it must account for the actual friction coefficient of what's being grasped.

The full dynamics equation extends naturally to include contact:

```
M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ + J_contact^T · λ
```

where `λ` is the contact force vector and `J_contact` (the contact Jacobian) maps it into joint torques — the exact same Jacobian machinery from Module 1, reused for force instead of velocity.

## Where this connects in your actual robot stack

| Method | Use for | Key output | In your robot |
|---|---|---|---|
| Lagrangian | Deriving equations of motion symbolically | `M(q)`, `C(q,q̇)`, `g(q)` | Thesis mathematics, understanding |
| Newton-Euler | Real-time control, any DOF | Joint torques numerically | Pinocchio RNEA at 1kHz |
| Pinocchio RNEA | Fast inverse dynamics | `τ` given `q, q̇, q̈` | `ros2_control` feedforward torque |
| Pinocchio ABA | Fast forward dynamics | `q̈` given `q, q̇, τ` | Physics simulation, Gazebo |
| Gravity `g(q)` | Feedforward compensation | Holding torques | First term of every robot controller |
| Contact/friction | Grasping, manipulation planning | Required gripper force | Gripper force commands |

## What the notebook proves, numerically

1. The mass matrix at `q2=45°` is exactly `M = [[0.611, 0.124], [0.124, 0.053]]`.
2. The gravity vector at `q1=30°, q2=45°` is exactly `g = [9.004, 0.508]` Nm.
3. Coriolis terms at the book's example velocities are exactly `C = [-0.088, 0.071]` Nm.
4. The full joint torques combine to `τ1 = 10.26 Nm, τ2 = 0.880 Nm`, with gravity confirmed as 87.8% of joint 1's total torque.
5. The friction-cone grip-force calculation reproduces both the dry (9.81 N with safety factor) and wet (24.5 N) results, visualized as the cone geometry itself.
6. A full-arm simulation shows what happens if URDF mass is wrong: the arm visibly drifts from a held pose under a small gravity-model error, even with a working feedback controller — reproducing the module's warning about the most common real-robot bug.

## Real-world applied case — the cobot that passed certification and still hurt someone

**The setting.** A collaborative robot rated for human-adjacent operation moves a 3 kg payload between a conveyor and a fixture. It passed its power-and-force-limiting assessment at the commissioning pose. Months later, an operator reaching into the cell takes a bruising impact from the same arm running the same program at the same commanded speed.

**Why the certification did not transfer — it is Section 2.1's first term.** The mass matrix `M(q)` is a function of configuration. The certification measured impact force at *one* configuration; the incident happened at another. Concretely, the **reflected inertia** seen at the end-effector varies dramatically across the workspace: an arm folded near its base presents low apparent inertia, while an arm fully extended presents several times more, because inertia scales with the square of the distance from the rotation axis. Same motor, same velocity, several times the kinetic energy delivered on contact:

```
E = ½ · m_eff(q) · v²
```

with `m_eff` depending on `q` exactly as `M(q)` says it does. Nothing about the program changed. The *geometry* of where the program ran changed.

**The second contributor — gravity compensation was masking a model error.** Section 2.3's result is that gravity dominates at low speed: `g(q)` is the largest term in the torque budget. Cobots compensate for it in feedforward using the URDF's mass and centre-of-mass values. If the URDF's payload mass is wrong — because a gripper was swapped, or a tool was added and never modelled — the feedforward is wrong by a constant offset, and the PID quietly absorbs the difference. The arm still reaches its targets, so nothing looks broken. But the controller is now running with a persistent torque bias, and on contact that bias adds to the impact.

**The tell, which is the same one as Section 2.3's diagnostic:** a wrong-URDF-mass bug is *silent and steady-state*. There is no error in the logs. Look instead at the **steady-state current draw while holding a pose against gravity** and compare it to what `g(q)` predicts from the model. A consistent discrepancy is a modelling error, not a tuning problem — and no gain change will fix it, though strong gains will hide it.

**What a correct treatment requires:**

| Requirement | Why the dynamics module is the right place for it |
|---|---|
| Assess impact energy across the **whole reachable workspace**, not one pose | Because `M(q)` is configuration-dependent — this is the module's central claim |
| Re-run the assessment after **any** payload or tool change | Because the payload enters `M(q)` and `g(q)` directly |
| Verify feedforward against measured holding current | Because `g(q)` is the dominant term and its errors are silent |
| Speed-limit by **energy**, not by joint velocity | Because a fixed joint-velocity cap permits very different impact energies at different configurations |

**The transferable lesson, and why it is a safety point rather than a modelling nicety.** `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ` is usually taught as the equation that tells motors what to do. It is equally the equation that tells you **how much energy the robot can deliver to a person at a given configuration** — and that reading is the one that matters when a human shares the cell. This case is also the clearest instance in the repository of the physical-safety gap recorded in `AUDIT_LOG.md` Section 16, recommendation 2.

**Connects to:** Module 1's Jacobian (the same matrix, transposed, maps that contact force back to joint torques); Module 4, whose simulators solve this equation every physics step and therefore inherit any URDF error; `6-Month_Program` Month 3, which derives Lagrangian dynamics for a real TIAGo.

---

## Next module

Dynamics tells us how force produces motion, but it assumes we know exactly where the robot is. Real sensors are noisy and real motion has uncertainty — Module 3 builds the probability tools that let a robot say, honestly, "I think I'm here, with this much uncertainty," and use that belief to build a map while staying localized.
