# The theory of a 7-DOF arm

This document describes the model of a redundant manipulator, from rotation
groups up to torque commands. It is the companion to [README.md](README.md).
The README covers what the code does and how to run it. Everything below is the
maths that the code implements.

The notation is that of Lynch & Park, *Modern Robotics*. Twists are `[ω; v]`,
with the angular part first. Wrenches are `[m; f]`, with the moment first. All
units are SI units.

**Contents**

1. [Redundancy](#1-redundancy)
2. [Rotations: SO(3)](#2-rotations-so3)
3. [Rigid motions: SE(3), twists, and the adjoint](#3-rigid-motions-se3-twists-and-the-adjoint)
4. [The robot model: screws](#4-the-robot-model-screws)
5. [Forward kinematics](#5-forward-kinematics)
6. [Velocity kinematics: the Jacobian](#6-velocity-kinematics-the-jacobian)
7. [Inverse kinematics](#7-inverse-kinematics)
8. [The topology of joint space](#8-the-topology-of-joint-space)
9. [Redundancy resolution: the null space](#9-redundancy-resolution-the-null-space)
10. [Dynamics](#10-dynamics)
11. [Trajectory generation](#11-trajectory-generation)
12. [Computed-torque control](#12-computed-torque-control)
13. [Numerical verification](#13-numerical-verification)

---

## 1. Redundancy

An end-effector pose is an element of SE(3). It has three numbers for position
and three numbers for orientation. Thus a pose has six numbers.

A 6-DOF arm has exactly six joints. Its Jacobian `J` is 6×6 and generically
invertible. Thus a desired tool velocity maps to a unique joint velocity. A
desired pose has a finite set of solutions. For an arm with a spherical wrist,
this set has eight solutions.

A 7-DOF arm has one joint more than the task needs. `J` is 6×7. Thus, by
rank-nullity, its null space has a dimension of at least one:

```
dim null(J)  ≥  n − rank(J)  =  7 − 6  =  1
```

There is a one-parameter family of joint velocities `q̇` that satisfy
`J q̇ = 0`. These are motions of the arm that produce no tool motion. If the tool
is held fixed, the elbow can still move. The set of configurations that reach a
fixed pose is a one-dimensional curve through configuration space. This curve is
called the **self-motion manifold**.

Thus inverse kinematics for a redundant arm has no unique answer. It has a
continuum of answers. The position along that manifold parameterises these
answers. The choice among them is called **redundancy resolution**. Section 9
describes this choice.

The practical consequence is that secondary objectives have no cost. A motion
along the self-motion manifold costs nothing in tool accuracy. Thus the arm can
hold its pose and improve its posture at the same time. For example, the arm
can move the elbow away from an obstacle or away from a singularity. It can also
keep the joints near the centre of their travel.

---

## 2. Rotations: SO(3)

### The group

`SO(3)` is the set of 3×3 matrices that satisfy

```
RᵀR = I,        det R = +1
```

It is a group under matrix multiplication and a smooth 3-dimensional manifold.
This representation of rotations avoids the defects of three-angle
parameterisations. For example, roll-pitch-yaw loses a degree of freedom when
the pitch reaches ±90°. This defect is called gimbal lock. The angular-velocity
map of roll-pitch-yaw is also singular at the same configurations.

### The Lie algebra

The tangent space to `SO(3)` at the identity is `so(3)`. This is the set of
skew-symmetric 3×3 matrices. It is isomorphic to ℝ³ through the *hat* operator:

```
ω = (ω₁, ω₂, ω₃)      ω̂ = [  0   −ω₃   ω₂ ]
                           [  ω₃   0   −ω₁ ]
                           [ −ω₂   ω₁   0  ]
```

The inverse operator is *vee*. Hat has the property `ω̂ p = ω × p`. Thus `ω̂` is
the linear operator that takes the cross product with ω.

### Exponential and logarithm

The exponential map carries a rotation vector to a rotation. Let `θ = ‖ω‖`. With
the identity `ω̂³ = −θ²ω̂`, the series of the matrix exponential collapses in
closed form to **Rodrigues' rotation formula**:

```
exp(ω̂) = I + (sin θ / θ) ω̂ + ((1 − cos θ) / θ²) ω̂²
```

The inverse map takes a rotation to the vector that generates it:

```
θ = arccos((tr R − 1) / 2),      ω̂ = (θ / (2 sin θ)) (R − Rᵀ)
```

Both coefficients in Rodrigues' formula are `0/0` at `θ = 0`. The singularities
are removable. The limits are `sin θ / θ → 1` and `(1 − cos θ)/θ² → ½`. But the
evaluation of the closed form near zero loses precision. The numerator has
cancellation, and the denominator underflows. Implementations switch to the
Taylor expansion below a small threshold.

The logarithm has a related problem at `θ → π`. There `sin θ → 0`, and the axis
becomes ill-conditioned.

Implementation: `libkinematics/src/math/so3.cpp`.

---

## 3. Rigid motions: SE(3), twists, and the adjoint

### The group

A rigid motion is a rotation and a translation. It is written as a 4×4
homogeneous transform:

```
T = [ R   p ]     R ∈ SO(3),  p ∈ ℝ³
    [ 0   1 ]
```

Its inverse is

```
T⁻¹ = [ Rᵀ  −Rᵀp ]
      [ 0     1  ]
```

### Twists

The Lie algebra `se(3)` is the set of elements

```
[V] = [ ω̂   v ]        V = [ω; v] ∈ ℝ⁶
      [ 0   0 ]
```

`V` is a **twist**. It is the angular velocity `ω` stacked on the linear
velocity `v`. The order is a convention. It must be fixed globally, because the
alternative order `[v; ω]` produces matrices of the same shape, whose blocks are
transposed relative to each other. This project uses `[ω; v]` everywhere.

The exponential map is

```
exp([V]) = [ exp(ω̂)   G(θ) v ]
           [   0          1   ]

G(θ) = I + ((1 − cos θ)/θ²) ω̂ + ((θ − sin θ)/θ³) ω̂²
```

`G` has the same removable singularity at `θ = 0` as Rodrigues' formula. The
logarithm uses `G⁻¹`, which has one more such singularity.

### Screws

Each rigid motion is a rotation about an axis, combined with a translation along
the same axis. This is **Chasles' theorem**. The pair of the axis and the pitch
is a **screw**. A twist is a screw axis scaled by a rate. For a revolute joint
with a unit axis `ω̂` through a point `q`, the twist is

```
V = [ ω ; −ω × q ]
```

The linear part is not the velocity of the joint. It is the velocity that a
point rigidly attached at the origin has under that rotation.

### The adjoint

The adjoint changes the frame of a twist. A twist in frame A becomes, in frame
B,

```
V_B = Ad_{T_BA} V_A

Ad_T = [  R     0 ]        (for [ω; v] ordering)
       [ p̂R     R ]
```

The adjoint is the 6×6 representation of a change of frame that acts on twists.
Its block structure depends on the twist order. If the order is `[v; ω]`, the
off-diagonal block moves to the upper right. Both forms are invertible. Only one
form is correct for a given convention. This is the reason that the convention
is fixed globally.

The sections below use two identities many times:

```
Ad_{T₁T₂} = Ad_{T₁} Ad_{T₂}        Ad_{T}⁻¹ = Ad_{T⁻¹}
```

Wrenches are dual to twists. Thus they transform by the transpose:
`F_A = Ad_{T_BA}ᵀ F_B`.

Implementation: `libkinematics/src/math/se3.cpp`.

---

## 4. The robot model: screws

A Denavit-Hartenberg table can describe a serial chain with four parameters per
joint. This table needs a prescribed frame assignment at each joint. The
**product of exponentials** description needs no intermediate frames. It uses
two things:

- `M` ∈ SE(3), the tool pose when each joint is at zero. This is the *home*
  pose.
- `Bᵢ` ∈ ℝ⁶, the screw axis of joint *i* expressed **in the tool frame**, with the
  robot at the home configuration.

The space-frame variant uses `Sᵢ`. These are the same axes expressed in the base
frame. The adjoint of the home pose relates the two:

```
Sᵢ = Ad_M Bᵢ
```

For this arm, the model is stored as YAML:

```yaml
# kinova_gen2_screws.yaml  (rounded; the file carries full precision)
home_pose:                    # M: tool pose at q = 0
  position: [0, -0.0098, -0.0872]
  quaternion_wxyz: [0, 0.70710678, 0.70710678, 0]
body_screw_axes:              # B_i = [omega; v], in the TOOL frame at home
  - [0,  0,  1,  0,  -0.0098,  0]
  - [-1, 0,  0,  0,   0.3627,  0]
  - [0,  0, -1,  0,   0.0114,  0]
  ...
```

A screw axis is meaningful only together with the frame in which it is
expressed. Joint 1 above has `ω = [0, 0, +1]` in the tool frame. The axis of the
same joint in world coordinates is `[0, 0, −1]`. The home pose is a 180°
rotation, so the `+z` of the tool frame points along the `−z` of the world. Both
values describe the same physical axis.

---

## 5. Forward kinematics

In body form, the tool pose is the home pose followed by the screw motion of
each joint. Each screw motion is expressed in the frame that the motions of the
earlier joints produce:

```
T(q) = M · exp([B₁] q₁) · exp([B₂] q₂) · … · exp([B₇] q₇)
```

The space form accumulates on the left instead:

```
T(q) = exp([S₁] q₁) · … · exp([S₇] q₇) · M
```

Both forms give the same transform. This project uses the body form. The reason
is that its Jacobian, derived in §6, is already expressed in the tool frame. The
task is specified in that frame.

Intermediate link frames come from a truncated product. The pose of link *k* is
the accumulated product of the first *k* exponentials, applied to the home frame
of that link. The collision geometry and the obstacle-clearance gradients in §9
need these frames.

Implementation: `libkinematics/src/fk.cpp`. Cost: 440 ns for the tool pose,
480 ns for all seven link frames.

---

## 6. Velocity kinematics: the Jacobian

### Body and space Jacobians

The derivative of forward kinematics gives a linear map from joint rates to the
tool twist:

```
V_b = J_b(q) q̇          J_b ∈ ℝ⁶ˣ⁷
```

Column *i* of the body Jacobian is the screw axis of joint *i*, pushed forward
by the joints *distal* to it:

```
J_b,ᵢ = Ad_{[exp([Bᵢ₊₁]qᵢ₊₁) ⋯ exp([B₇]q₇)]⁻¹} Bᵢ,      J_b,₇ = B₇
```

The space Jacobian is the same map written in the base frame. Its column *i* is
the space screw axis, pushed forward by the joints *proximal* to it:

```
J_s,ᵢ = Ad_{exp([S₁]q₁) ⋯ exp([S_{i−1}]q_{i−1})} Sᵢ,      J_s,₁ = S₁
```

The adjoint of the current tool pose relates the two:

```
J_s = Ad_{T(q)} J_b
```

### Manipulability

The image of the unit ball of joint velocities under `J` is an ellipsoid in
twist space. This is the **manipulability ellipsoid**. Its principal axes are
the left singular vectors of `J`. The lengths of these axes are the singular
values. Its volume is proportional to the **Yoshikawa manipulability index**:

```
w(q) = √det(J Jᵀ)
```

`w` is zero exactly at a singularity. There `J` loses rank, and some direction
of tool motion is not reachable at any joint speed. Near a singularity, `w` is
small and the ellipsoid is flat. Motion along the direction that collapses
demands large joint rates.

For a square Jacobian, `w = |det J|`. For the redundant case, `J Jᵀ` is the 6×6
Gram matrix. This matrix keeps full rank if the arm can move its tool in all six
directions.

Implementation: `libkinematics/src/jacobian.cpp`.

---

## 7. Inverse kinematics

### Newton iteration

Inverse kinematics solves `T(q) = T_target` for `q`. The problem is written as
root finding. The error at iteration *k* is the twist that carries the current
pose to the target:

```
V = log( T(q)⁻¹ T_target )
```

A Newton step solves `J(q) Δq = V`. For a redundant arm, `J` is not square. Thus
the minimum-norm solution uses the pseudoinverse `Δq = J⁺V`.

### Damping

Near a singularity, `J⁺` has entries proportional to `1/σ_min`. Thus a small
requested tool motion produces an unbounded joint velocity. **Damped least
squares**, from Nakamura and from Wampler, replaces the exact solve with a
regularised solve:

```
minimise  ‖J q̇ − V‖² + λ²‖q̇‖²

⟹   q̇ = Jᵀ(J Jᵀ + λ²I)⁻¹ V
```

An equivalent form is `q̇ = (JᵀJ + λ²I)⁻¹Jᵀ V`. The first form inverts a 6×6
matrix, and the second form inverts a 7×7 matrix. Thus the first form is cheaper
for a redundant arm. In terms of singular values, damping replaces `1/σᵢ` with

```
σᵢ / (σᵢ² + λ²)
```

This value is close to `1/σᵢ` when `σᵢ ≫ λ`. It tends to `σᵢ/λ²` as `σᵢ → 0`.
This bounds the step, at the cost of tracking accuracy in the direction that
collapses.

`λ` is not a constant. The solver schedules it. It stays near zero in
well-conditioned regions. The solver raises it when the arm approaches a
singularity. Here, the solver detects a singularity when the manipulability
falls below a threshold.

### Multiple solution branches

The solution set is a manifold, not a point. Thus a descent from a given seed
converges to the branch in which that seed lies. It can also converge to a local
minimum of the error that is not a solution. Thus solvers try several seeds.

The solver draws the seeds from a fixed pseudorandom sequence. This keeps the
map from goal to solution deterministic. Determinism is necessary when the
result feeds a planner that must be reproducible.

Implementation: `libkinematics/src/ik.cpp`. Round-trip accuracy on random poses:
99.6% convergence, 0.045 ms median.

---

## 8. The topology of joint space

Joint limits are usually written as an interval `[qmin, qmax]`. This treats the
configuration space as a box in ℝⁿ. That is correct only for joints with
mechanical stops.

On this arm, the four roll joints have no stop. These are joints 1, 3, 5, and 7.
They rotate continuously. Their configuration space is the circle `S¹`, not an
interval. The full configuration space is

```
C  =  S¹ × [q₂ᵐⁱⁿ, q₂ᵐᵃˣ] × S¹ × [q₄ᵐⁱⁿ, q₄ᵐᵃˣ] × S¹ × [q₆ᵐⁱⁿ, q₆ᵐᵃˣ] × S¹
```

On `S¹`, the points `q = −π` and `q = +π` are the same point. The distance
between two angles is not `|q₂ − q₁|`. It is the geodesic

```
d(q₁, q₂) = remainder(q₂ − q₁, 2π)  ∈  [−π, π]
```

The interval is closed at both ends, not half-open. `std::remainder` rounds the
quotient half to even. Thus an exactly antipodal displacement resolves to one of
`±π`. The parity of that quotient selects which one. For example,
`remainder(π, 2π)` is `+π` and `remainder(3π, 2π)` is `−π`.

Both name the same point of `S¹`, so the geodesic length does not change. But
code that branches on the sign of the displacement gets a direction that depends
on the representative that it receives.

Two consequences follow. First, a displacement computed as a plain difference can
be up to `2π` longer than the true shortest path. The arm then rotates through
the longer arc instead of across the branch cut. Second, two pieces of code that
reason about the same displacement must use the same metric. An example is a
trajectory generator and a collision checker. If the metrics differ, one will
validate a path that the other never takes.

Joint-limit repulsion also does not apply to a continuous joint, because a
continuous joint has no limit.

---

## 9. Redundancy resolution: the null space

### The projector

For a 6×7 Jacobian,

```
N(q) = I − J⁺J
```

is the orthogonal projector onto `null(J)`. It is symmetric and idempotent.
Also, `J (N v) = 0` for each `v`. Thus a vector passed through `N` produces no
tool motion. The general solution to `J q̇ = V` is thus

```
q̇ = J⁺ V  +  N(q) q̇₀
```

for an arbitrary `q̇₀`. The first term is the minimum-norm solution to the task.
The second term ranges over the full self-motion manifold. For this arm,
`rank N = 1`. This matches the one redundant degree of freedom.

### Secondary objectives

The choice `q̇₀ = ∓∇H(q)` does gradient descent or gradient ascent on a scalar
cost `H`. The tool task stays exactly satisfied. Common choices are:

**Joint-limit centering.** This cost penalises the distance from the middle of
the range of each joint:

```
H(q) = (1/2n) Σᵢ ((qᵢ − qᵢᵐⁱᵈ) / (qᵢᵐᵃˣ − qᵢᵐⁱⁿ))²
```

**Manipulability.** This objective ascends `w(q) = √det(J Jᵀ)` to move away from
singularities. The analytic gradient needs `∂J/∂qᵢ`. Thus implementations
usually compute the gradient numerically.

**Obstacle clearance.** This cost models each link as a bounding sphere. It
penalises proximity to obstacle spheres with an inverse-distance barrier:

```
H(q) = Σ_links Σ_obstacles  1 / d(q)
```

Here `d` is the signed clearance between the sphere surfaces. The barrier must
stay finite and strictly decreasing in `d` through `d = 0`. A naive
`1/max(d, ε)` is constant below `ε`. Thus it has zero gradient exactly where
contact occurs. A linear extension of the barrier below `ε`, along its own
tangent, keeps the cost `C¹`. It also keeps the gradient nonzero through contact.

### Limits of redundancy resolution

With the tool pose fixed, the reachable set of postures is the self-motion
manifold. This set is one-dimensional. Thus the elbow has a single direction of
freedom at each configuration. Whether that direction points away from an
obstacle is a property of the geometry, not of the gains. Projected-gradient
avoidance works well when the obstacle is near the tangent of the manifold. It
works poorly in other cases.

To make clearance work in each direction, the problem must be posed as a
constrained optimisation instead:

```
minimise    ‖q̇ − q̇_desired‖²
subject to  J q̇ = V              (tool task, hard equality)
            ∇dⱼᵀ q̇ ≥ −α dⱼ       (clearance, inequality per pair)
            q̇ᵐⁱⁿ ≤ q̇ ≤ q̇ᵐᵃˣ
```

This is a task-priority quadratic program. It handles inequality constraints
that a projected potential field cannot express.

Implementation: `libkinematics/src/null_space.cpp`.

---

## 10. Dynamics

### The equations of motion

For a serial chain,

```
τ = M(q) q̈  +  C(q, q̇) q̇  +  g(q)  +  J_bᵀ f_ext
```

The terms are:

- `M`, the joint-space mass matrix. It is symmetric and positive definite.
- `C q̇`, the Coriolis and centrifugal torques.
- `g`, the gravity torque.
- `f_ext`, the wrench that the tool applies to its environment.

### Recursive Newton-Euler

The `O(n)` algorithm computes `τ` from `(q, q̇, q̈)` in two sweeps.

The *forward* sweep goes from the base to the tool. It propagates the twist and
the acceleration through the adjoints:

```
Vᵢ    = Ad_{T i,i−1} Vᵢ₋₁  +  Aᵢ q̇ᵢ
V̇ᵢ    = Ad_{T i,i−1} V̇ᵢ₋₁  +  ad_{Vᵢ} Aᵢ q̇ᵢ  +  Aᵢ q̈ᵢ
```

Here `Aᵢ` is the screw of joint *i* in the frame of link *i*. `ad_V` is the Lie
bracket matrix. Gravity enters as a fictitious base acceleration
`V̇₀ = [0; −g]`.

The *backward* sweep goes from the tool to the base. It propagates the wrenches
inward:

```
Fᵢ  = Ad_{T i+1,i}ᵀ Fᵢ₊₁  +  𝒢ᵢ V̇ᵢ  −  ad_{Vᵢ}ᵀ 𝒢ᵢ Vᵢ
τᵢ  = Fᵢᵀ Aᵢ
```

Here `𝒢ᵢ` is the 6×6 spatial inertia of link *i*. The recursion needs no matrix
inversions.

The other quantities are corollaries:

- `g(q)` is RNE evaluated at `q̇ = q̈ = 0` with gravity on.
- Column *j* of `M(q)` is RNE at `q̇ = 0`, `q̈ = eⱼ`, gravity off.
- `C(q,q̇) q̇` is RNE at `q̈ = 0` with gravity subtracted.

### The Coriolis matrix and passivity

The vector `C q̇` is sufficient for control. But the passivity property needs the
matrix `C` itself. Its entries are the Christoffel symbols of the first kind,
built from the derivatives of the mass matrix:

```
Cᵢⱼ = Σₖ ½ ( ∂Mᵢⱼ/∂qₖ  +  ∂Mᵢₖ/∂qⱼ  −  ∂Mⱼₖ/∂qᵢ ) q̇ₖ
```

With `C` defined this way,

```
Ṁ(q) − 2C(q, q̇)   is skew-symmetric
```

Thus `xᵀ(Ṁ − 2C)x = 0` for all `x`. The choice `x = q̇` gives
`q̇ᵀṀq̇ = 2q̇ᵀCq̇`. This states that the Coriolis terms do no net work. They
redistribute kinetic energy, and they do not add or remove it. Energy-based
stability proofs for impedance controllers and passivity-based controllers rely
on this property.

The identity is a consequence of the Christoffel construction. It holds for any
symmetric `M`. Thus a check of the identity does not verify `M`. An independent
check compares `C(q,q̇) q̇` with the Newton-Euler recursion. The recursion
computes the same quantity by a different route.

Implementation: `libkinematics/src/dynamics.cpp`. Costs: 990 ns for inverse
dynamics, 7.2 µs for the mass matrix, which takes seven RNE passes.

---

## 11. Trajectory generation

Inverse kinematics produces a goal configuration. It does not specify time. A
trajectory assigns a configuration to each instant. It obeys the velocity,
acceleration, and jerk limits of the actuators.

### Path parameterisation

The path is the straight line in joint space from `q_start` to `q_goal`. A
scalar `s ∈ [0, 1]` parameterises it:

```
q(s)  = q_start + s·Δq,        Δq = q_goal − q_start
q̇     = ṡ Δq
q̈     = s̈ Δq
q⃛     = ⃛s Δq
```

Each joint is then an affine function of one variable. Thus the per-joint limits
map directly onto limits on `s`:

```
ṡ_max   = minᵢ ( q̇ᵢᵐᵃˣ / |Δqᵢ| )
s̈_max   = minᵢ ( q̈ᵢᵐᵃˣ / |Δqᵢ| )
⃛s_max   = minᵢ ( q⃛ᵢᵐᵃˣ / |Δqᵢ| )
```

The minimum over the joints means that the most-constrained joint saturates
first. Thus no joint can exceed its limit. The joints are also automatically
**synchronised**. They start together, they finish together, and they keep a
fixed ratio throughout. This is because they share the single parameter `s`.

On `S¹` joints, `Δq` is the geodesic difference from §8, not the plain
difference.

### Time-optimal profile on s

The problem is now one-dimensional. The task is to drive `s` from 0 to 1 in
minimum time, subject to `|ṡ| ≤ ṡ_max`, `|s̈| ≤ s̈_max`, and `|⃛s| ≤ ⃛s_max`.
Without the jerk bound, the solution is the classical trapezoidal profile for
velocity. This profile has three phases:

1. Accelerate at `s̈_max`.
2. Coast at `ṡ_max`.
3. Decelerate at `−s̈_max`.

For short moves, the coast phase vanishes. This is the triangular case.

With a jerk bound, the acceleration itself must ramp. This gives the S-curve
profile. It has up to seven phases. In this profile, `s̈` is trapezoidal in
time, and `ṡ` is smoothed as a result. The jerk bound is important because a
step in acceleration is a step in the commanded torque. Such a step excites
structural modes and is audible in a physical machine.

The braking distance includes the jerk-limited ramp that brings `s̈` back to
zero. It is

```
Δs_brake  =  ṡ²/(2 s̈_max)  +  ṡ · s̈_max /(2 ⃛s_max)
```

Deceleration starts when the distance that remains reaches this value.

If the goal changes during a motion, the generator first brakes to rest along
the current direction. Then it plans a new trajectory. A new direction spliced
into a reference that is still in motion would step the reference velocity. That
step is the discontinuity that the jerk limit exists to prevent.

---

## 12. Computed-torque control

The trajectory gives a reference `(q_ref, q̇_ref, q̈_ref)`. A controller must make
the plant follow it.

### The control law

Independent-joint PD ignores two facts. The inertia of the plant varies with the
configuration, and it couples the joints. Thus a single set of gains cannot be
critically damped everywhere. **Computed torque** uses the model to cancel the
nonlinearities. With the error `e = q_ref − q`, the command is

```
τ = M(q)·( q̈_ref + K_d ė + K_p e + K_i ∫e )  +  C(q,q̇) q̇  +  g(q)
```

After substitution of this command into the equations of motion, `M` cancels on
both sides. The closed-loop error then obeys

```
ë + K_d ė + K_p e = 0
```

This is a linear, decoupled, second-order system. It is the same in each joint
at each configuration. The choice

```
K_p = ω_n²,        K_d = 2ζω_n
```

with `ζ = 1` gives a critically damped response with no overshoot. `ω_n` sets
the bandwidth. This is the purpose of the mass matrix of §10. It converts a
coupled nonlinear plant into `n` independent second-order systems.

### Practical terms

**Friction feedforward.** The controller adds Coulomb friction as
`f_c·sign(q̇_ref)`. The *reference* velocity drives this term. If the measured
velocity drives it, the term becomes a positive feedback loop around zero
crossings, and the joint oscillates.

**Anti-windup.** Actuators saturate. If the integrator continues to accumulate
while the command is clipped, the accumulated term must unwind later. This
produces a large overshoot. Conditional integration suspends accumulation when
the command is saturated and the error would drive it further in the same
direction. The model of saturation in the controller must match the actual
limit of the plant. If it does not, the integrator continues to accumulate
beyond a limit that the controller does not know about.

**Model error.** The cancellation is only as accurate as `M`, `C`, and `g`. A
residual error appears as a disturbance to the linear error dynamics. The PD
terms reject this disturbance in proportion to their bandwidth. This is the
reason that §13 verifies the model against an independent oracle, and does not
assume that it is correct.

---

## 13. Numerical verification

The tests check the maths above against MuJoCo. MuJoCo computes the same
quantities for the same model with unrelated algorithms. Agreement at machine
precision across a large random sample is evidence that both are correct.

The model-consistency check rebuilds the screw parameters from the body frames
of the MJCF:

| | agreement |
|---|---|
| home pose, position | 2.4 × 10⁻¹⁶ |
| home pose, rotation | 2.6 × 10⁻¹² |
| all seven body screw axes | 1.6 × 10⁻¹⁵ |

The kinematics and dynamics checks use 1000 random configurations. The mass
matrix check uses 50 configurations:

| quantity | worst error |
|---|---|
| FK tool pose | 1.3 × 10⁻¹⁴ |
| FK link positions | 5.3 × 10⁻¹⁵ m |
| mass matrix `M(q)` | 8.9 × 10⁻¹⁵ kg·m² |
| gravity `g(q)` | 1.4 × 10⁻¹³ N·m |
| inverse dynamics `τ` | 1.5 × 10⁻¹³ N·m |

The tests check quantities with no external oracle against an independent
construction instead. The tests compare the body Jacobian with a 5-point
numerical derivative of forward kinematics. The derivative agrees to 10⁻⁵ and
shares no code with the analytic form. The tests compare the space Jacobian with
the left-accumulated screw form of §6, not with `Ad_T J_b`. The reason is that
the code computes the space Jacobian as that product. The tests compare the
Coriolis matrix with the Newton-Euler recursion.

To reproduce these results, run:

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build
./build/libkinematics/kin_report
```
