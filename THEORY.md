# The theory of a 7-DOF arm

How a redundant manipulator is modelled, from rotation groups up to torque
commands. This is the companion to [README.md](README.md), which covers what the
code does and how to run it. Everything below is the maths that code implements.

Notation is Lynch & Park, *Modern Robotics*. Twists are `[ω; v]`, angular part
first; wrenches are `[m; f]`, moment first. SI units throughout.

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

An end-effector pose lives in SE(3): three numbers for position, three for
orientation. Six.

A 6-DOF arm has exactly that many joints. Its Jacobian `J` is 6×6 and generically
invertible, so a desired tool velocity maps to a unique joint velocity, and a
desired pose has a finite solution set (eight, for an arm with a spherical wrist).

A 7-DOF arm has one joint more than the task requires. `J` is 6×7, so by
rank-nullity its null space has dimension at least one:

```
dim null(J)  ≥  n − rank(J)  =  7 − 6  =  1
```

There is a one-parameter family of joint velocities `q̇` satisfying `J q̇ = 0`:
motions of the arm that produce no tool motion whatsoever. Physically, clamp the
gripper in a vice and the elbow can still swing. The set of configurations
reaching a fixed pose is a one-dimensional curve through configuration space,
called the **self-motion manifold**.

Inverse kinematics for a redundant arm therefore has no unique answer. It has a
continuum of answers, parameterised by position along that manifold. Choosing
among them is called **redundancy resolution**, and it is what §9 is about.

The practical consequence is that secondary objectives become free. Any motion
along the self-motion manifold costs nothing in tool accuracy, so the arm can
simultaneously hold its pose and improve its posture: move the elbow away from an
obstacle, retreat from a singularity, or keep joints near the centre of their
travel.

---

## 2. Rotations: SO(3)

### The group

`SO(3)` is the set of 3×3 matrices satisfying

```
RᵀR = I,        det R = +1
```

It is a group under matrix multiplication and a smooth 3-dimensional manifold.
Representing rotations this way avoids the defects of three-angle
parameterisations: roll-pitch-yaw loses a degree of freedom when pitch reaches
±90° (gimbal lock), and its angular-velocity map is singular at the same
configurations.

### The Lie algebra

The tangent space to `SO(3)` at the identity is `so(3)`, the set of
skew-symmetric 3×3 matrices. It is isomorphic to ℝ³ via the *hat* operator:

```
ω = (ω₁, ω₂, ω₃)      ω̂ = [  0   −ω₃   ω₂ ]
                           [  ω₃   0   −ω₁ ]
                           [ −ω₂   ω₁   0  ]
```

with inverse *vee*. Hat has the property `ω̂ p = ω × p`, so `ω̂` is the linear
operator "take the cross product with ω".

### Exponential and logarithm

The exponential map carries a rotation vector to a rotation. Writing `θ = ‖ω‖`
and using `ω̂³ = −θ²ω̂`, the matrix exponential series collapses in closed form to
**Rodrigues' rotation formula**:

```
exp(ω̂) = I + (sin θ / θ) ω̂ + ((1 − cos θ) / θ²) ω̂²
```

The inverse takes a rotation to the vector generating it:

```
θ = arccos((tr R − 1) / 2),      ω̂ = (θ / (2 sin θ)) (R − Rᵀ)
```

Both coefficients in Rodrigues' formula are `0/0` at `θ = 0`. The singularities
are removable, with limits `sin θ / θ → 1` and `(1 − cos θ)/θ² → ½`, but
evaluating the closed form near zero loses precision: the numerator suffers
cancellation while the denominator underflows. Implementations switch to the
Taylor expansion below a small threshold. The logarithm has a matching issue at
`θ → π`, where `sin θ → 0` and the axis becomes ill-conditioned.

Implementation: `libkinematics/src/math/so3.cpp`.

---

## 3. Rigid motions: SE(3), twists, and the adjoint

### The group

A rigid motion is a rotation and a translation, written as a 4×4 homogeneous
transform:

```
T = [ R   p ]     R ∈ SO(3),  p ∈ ℝ³
    [ 0   1 ]
```

with

```
T⁻¹ = [ Rᵀ  −Rᵀp ]
      [ 0     1  ]
```

### Twists

The Lie algebra `se(3)` consists of elements

```
[V] = [ ω̂   v ]        V = [ω; v] ∈ ℝ⁶
      [ 0   0 ]
```

`V` is a **twist**: angular velocity `ω` stacked on linear velocity `v`. The
ordering is a convention, and it must be fixed globally, because the alternative
ordering `[v; ω]` produces matrices of identical shape whose blocks are
transposed relative to each other. This project uses `[ω; v]` everywhere.

The exponential map is

```
exp([V]) = [ exp(ω̂)   G(θ) v ]
           [   0          1   ]

G(θ) = I + ((1 − cos θ)/θ²) ω̂ + ((θ − sin θ)/θ³) ω̂²
```

`G` carries the same removable singularity at `θ = 0` as Rodrigues' formula, plus
one more in `G⁻¹` used by the logarithm.

### Screws

Every rigid motion is a rotation about some axis combined with a translation
along that same axis. This is **Chasles' theorem**, and the axis-plus-pitch pair
is a **screw**. A twist is a screw axis scaled by a rate. For a revolute joint
with unit axis `ω̂` passing through a point `q`, the twist is

```
V = [ ω ; −ω × q ]
```

The linear part is not the joint's velocity; it is the velocity that a point
rigidly attached at the origin would have under that rotation.

### The adjoint

A twist expressed in frame A becomes, in frame B,

```
V_B = Ad_{T_BA} V_A

Ad_T = [  R     0 ]        (for [ω; v] ordering)
       [ p̂R     R ]
```

The adjoint is the 6×6 representation of a change of frame acting on twists. Its
block structure depends on the twist ordering: swap to `[v; ω]` and the off-diagonal
block moves to the upper right. Both forms are invertible and only one is correct
for a given convention, which is why the convention is fixed globally.

Two identities used repeatedly below:

```
Ad_{T₁T₂} = Ad_{T₁} Ad_{T₂}        Ad_{T}⁻¹ = Ad_{T⁻¹}
```

Wrenches, being dual to twists, transform by the transpose:
`F_A = Ad_{T_BA}ᵀ F_B`.

Implementation: `libkinematics/src/math/se3.cpp`.

---

## 4. The robot model: screws

A serial chain can be described by a Denavit-Hartenberg table of four parameters
per joint, which requires a prescribed frame assignment at each joint. The
**product of exponentials** description needs no intermediate frames at all. It
uses two things:

- `M` ∈ SE(3), the tool pose when every joint is at zero (the *home* pose).
- `Bᵢ` ∈ ℝ⁶, the screw axis of joint *i* expressed **in the tool frame**, with the
  robot at that home configuration.

The space-frame variant uses `Sᵢ`, the same axes expressed in the base frame, and
the two are related by the adjoint of the home pose:

```
Sᵢ = Ad_M Bᵢ
```

For this arm the model is stored as YAML:

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

A screw axis is only meaningful together with the frame it is expressed in. Joint
1 above reads `ω = [0, 0, +1]` in the tool frame, while the same joint's axis in
world coordinates is `[0, 0, −1]`: the home pose is a 180° rotation, so the tool
frame's `+z` points along the world's `−z`. Both values describe the same
physical axis.

---

## 5. Forward kinematics

In body form, the tool pose is the home pose followed by each joint's screw
motion, expressed in the frame the preceding joints leave behind:

```
T(q) = M · exp([B₁] q₁) · exp([B₂] q₂) · … · exp([B₇] q₇)
```

The space form accumulates on the left instead:

```
T(q) = exp([S₁] q₁) · … · exp([S₇] q₇) · M
```

Both give the same transform. The body form is used here because the Jacobian
that falls out of it (§6) is already expressed in the tool frame, where the task
is specified.

Intermediate link frames come from truncating the product. The pose of link *k*
is the accumulated product of the first *k* exponentials applied to that link's
own home frame. These are needed for collision geometry and for the
obstacle-clearance gradients in §9.

Implementation: `libkinematics/src/fk.cpp`. Cost: 440 ns for the tool pose,
480 ns for all seven link frames.

---

## 6. Velocity kinematics: the Jacobian

### Body and space Jacobians

Differentiating forward kinematics gives a linear map from joint rates to the
tool twist:

```
V_b = J_b(q) q̇          J_b ∈ ℝ⁶ˣ⁷
```

Column *i* of the body Jacobian is joint *i*'s screw axis, pushed forward by the
joints *distal* to it:

```
J_b,ᵢ = Ad_{[exp([Bᵢ₊₁]qᵢ₊₁) ⋯ exp([B₇]q₇)]⁻¹} Bᵢ,      J_b,₇ = B₇
```

The space Jacobian is the same map written in the base frame. Its column *i* is
the space screw axis pushed forward by the joints *proximal* to it:

```
J_s,ᵢ = Ad_{exp([S₁]q₁) ⋯ exp([S_{i−1}]q_{i−1})} Sᵢ,      J_s,₁ = S₁
```

The two are related by the adjoint of the current tool pose:

```
J_s = Ad_{T(q)} J_b
```

### Manipulability

The image of the unit joint-velocity ball under `J` is an ellipsoid in twist
space, the **manipulability ellipsoid**. Its principal axes are the left singular
vectors of `J`, with lengths equal to the singular values. Its volume is
proportional to the **Yoshikawa manipulability index**:

```
w(q) = √det(J Jᵀ)
```

`w` is zero exactly at a singularity, where `J` loses rank and some direction of
tool motion becomes unreachable at any joint speed. Near a singularity `w` is
small and the ellipsoid is flattened: motion along the collapsing direction
demands very large joint rates.

For a square Jacobian `w = |det J|`. For the redundant case `J Jᵀ` is the 6×6
Gram matrix, which stays full rank as long as the arm can move its tool in all
six directions.

Implementation: `libkinematics/src/jacobian.cpp`.

---

## 7. Inverse kinematics

### Newton iteration

Inverse kinematics solves `T(q) = T_target` for `q`. Written as root finding, the
error at iteration *k* is the twist carrying the current pose to the target:

```
V = log( T(q)⁻¹ T_target )
```

and a Newton step solves `J(q) Δq = V`. For a redundant arm `J` is not square, so
the minimum-norm solution uses the pseudoinverse `Δq = J⁺V`.

### Damping

Near a singularity `J⁺` has entries proportional to `1/σ_min`, so a small
requested tool motion produces an unbounded joint velocity. **Damped least
squares** (Nakamura; Wampler) replaces the exact solve with a regularised one:

```
minimise  ‖J q̇ − V‖² + λ²‖q̇‖²

⟹   q̇ = Jᵀ(J Jᵀ + λ²I)⁻¹ V
```

Equivalently `q̇ = (JᵀJ + λ²I)⁻¹Jᵀ V`; the first form inverts a 6×6 and the second
a 7×7, so the first is cheaper for a redundant arm. In terms of singular values,
damping replaces `1/σᵢ` with

```
σᵢ / (σᵢ² + λ²)
```

which is close to `1/σᵢ` when `σᵢ ≫ λ` and rolls off to `σᵢ/λ²` as `σᵢ → 0`,
bounding the step at the cost of tracking accuracy in the collapsing direction.

`λ` is scheduled rather than constant: it stays near zero in well-conditioned
regions and is raised as the arm approaches a singularity, detected here by
manipulability falling below a threshold.

### Multiple solution branches

Because the solution set is a manifold rather than a point, a descent from a
given seed converges to whichever branch that seed lies in, and may converge to a
local minimum of the error that is not a solution at all. Solvers therefore try
several seeds. Drawing them from a fixed pseudorandom sequence keeps the mapping
from goal to solution deterministic, which matters when the result feeds a
planner that must be reproducible.

Implementation: `libkinematics/src/ik.cpp`. Round-trip accuracy on random poses:
99.6% convergence, 0.045 ms median.

---

## 8. The topology of joint space

Joint limits are usually written as an interval `[qmin, qmax]`, which treats
configuration space as a box in ℝⁿ. That is correct only for joints with
mechanical stops.

On this arm the four roll joints (1, 3, 5, 7) have no stop; they rotate
continuously. Their configuration space is the circle `S¹`, not an interval, and
the full configuration space is

```
C  =  S¹ × [q₂ᵐⁱⁿ, q₂ᵐᵃˣ] × S¹ × [q₄ᵐⁱⁿ, q₄ᵐᵃˣ] × S¹ × [q₆ᵐⁱⁿ, q₆ᵐᵃˣ] × S¹
```

On `S¹` the points `q = −π` and `q = +π` are identical, and the distance between
two angles is not `|q₂ − q₁|` but the geodesic

```
d(q₁, q₂) = remainder(q₂ − q₁, 2π)  ∈  [−π, π]
```

The interval is closed at both ends, not half-open. `std::remainder` rounds the
quotient half to even, so an exactly antipodal displacement resolves to whichever
of `±π` the parity of that quotient selects: `remainder(π, 2π)` is `+π` while
`remainder(3π, 2π)` is `−π`. Both name the same point of `S¹`, so the geodesic
length is unaffected, but code that branches on the sign of the displacement gets
a direction that depends on the representative it was handed.

Two consequences follow. First, a displacement computed as a plain difference can
be up to `2π` longer than the true shortest path, which shows up as the arm
unwinding the long way around rather than crossing the branch cut. Second, any
two pieces of code that reason about the same displacement (a trajectory
generator and a collision checker, say) must use the same metric, or one will
validate a path the other never takes.

Joint-limit repulsion also does not apply to a continuous joint, because there is
no limit to be repelled from.

---

## 9. Redundancy resolution: the null space

### The projector

For a 6×7 Jacobian,

```
N(q) = I − J⁺J
```

is the orthogonal projector onto `null(J)`. It is symmetric and idempotent, and
`J (N v) = 0` for every `v`, so any vector passed through `N` produces no tool
motion. The general solution to `J q̇ = V` is therefore

```
q̇ = J⁺ V  +  N(q) q̇₀
```

for arbitrary `q̇₀`. The first term is the minimum-norm solution to the task; the
second ranges over the entire self-motion manifold. For this arm `rank N = 1`,
matching the one redundant degree of freedom.

### Secondary objectives

Choosing `q̇₀ = ∓∇H(q)` performs gradient descent (or ascent) on a scalar cost `H`
while the tool task is satisfied exactly. Common choices:

**Joint-limit centering.** Penalise distance from the middle of each joint's
range:

```
H(q) = (1/2n) Σᵢ ((qᵢ − qᵢᵐⁱᵈ) / (qᵢᵐᵃˣ − qᵢᵐⁱⁿ))²
```

**Manipulability.** Ascend `w(q) = √det(J Jᵀ)` to move away from singularities.
The analytic gradient requires `∂J/∂qᵢ`, so it is usually taken numerically.

**Obstacle clearance.** Model each link as a bounding sphere and penalise
proximity to obstacle spheres with an inverse-distance barrier:

```
H(q) = Σ_links Σ_obstacles  1 / d(q)
```

where `d` is the signed clearance between sphere surfaces. The barrier must
remain finite and strictly decreasing in `d` through `d = 0`; a naive
`1/max(d, ε)` is constant below `ε` and therefore has zero gradient exactly where
contact occurs. Extending the barrier linearly below `ε` along its own tangent
keeps the cost `C¹` and the gradient nonzero through contact.

### What redundancy resolution can and cannot do

With the tool pose fixed, the reachable set of postures is the self-motion
manifold: one-dimensional. The elbow therefore has a single direction of freedom
at any configuration, and whether that direction happens to point away from an
obstacle is a property of the geometry, not of the gains. Projected-gradient
avoidance works well when the obstacle lies near the manifold's tangent and
poorly otherwise.

Making clearance work in every direction requires posing the whole thing as a
constrained optimisation instead:

```
minimise    ‖q̇ − q̇_desired‖²
subject to  J q̇ = V              (tool task, hard equality)
            ∇dⱼᵀ q̇ ≥ −α dⱼ       (clearance, inequality per pair)
            q̇ᵐⁱⁿ ≤ q̇ ≤ q̇ᵐᵃˣ
```

which is a task-priority quadratic program. It handles inequality constraints
that a projected potential field cannot express.

Implementation: `libkinematics/src/null_space.cpp`.

---

## 10. Dynamics

### The equations of motion

For a serial chain,

```
τ = M(q) q̈  +  C(q, q̇) q̇  +  g(q)  +  J_bᵀ f_ext
```

with `M` the joint-space mass matrix (symmetric, positive definite), `C q̇` the
Coriolis and centrifugal torques, `g` the gravity torque, and `f_ext` the wrench
the tool applies to its environment.

### Recursive Newton-Euler

The `O(n)` algorithm computes `τ` from `(q, q̇, q̈)` in two sweeps.

*Forward*, from base to tool, propagating twist and acceleration through the
adjoints:

```
Vᵢ    = Ad_{T i,i−1} Vᵢ₋₁  +  Aᵢ q̇ᵢ
V̇ᵢ    = Ad_{T i,i−1} V̇ᵢ₋₁  +  ad_{Vᵢ} Aᵢ q̇ᵢ  +  Aᵢ q̈ᵢ
```

where `Aᵢ` is joint *i*'s screw in link *i*'s frame and `ad_V` is the Lie bracket
matrix. Gravity enters as a fictitious base acceleration `V̇₀ = [0; −g]`.

*Backward*, from tool to base, propagating wrenches inward:

```
Fᵢ  = Ad_{T i+1,i}ᵀ Fᵢ₊₁  +  𝒢ᵢ V̇ᵢ  −  ad_{Vᵢ}ᵀ 𝒢ᵢ Vᵢ
τᵢ  = Fᵢᵀ Aᵢ
```

with `𝒢ᵢ` the 6×6 spatial inertia of link *i*. The recursion needs no matrix
inversions.

The remaining quantities are corollaries:

- `g(q)` is RNE evaluated at `q̇ = q̈ = 0` with gravity on.
- Column *j* of `M(q)` is RNE at `q̇ = 0`, `q̈ = eⱼ`, gravity off.
- `C(q,q̇) q̇` is RNE at `q̈ = 0` with gravity subtracted.

### The Coriolis matrix and passivity

The vector `C q̇` is enough for control, but the matrix `C` itself is needed for
the passivity property. Its entries are the Christoffel symbols of the first kind
built from derivatives of the mass matrix:

```
Cᵢⱼ = Σₖ ½ ( ∂Mᵢⱼ/∂qₖ  +  ∂Mᵢₖ/∂qⱼ  −  ∂Mⱼₖ/∂qᵢ ) q̇ₖ
```

With `C` defined this way,

```
Ṁ(q) − 2C(q, q̇)   is skew-symmetric
```

so `xᵀ(Ṁ − 2C)x = 0` for all `x`. Taking `x = q̇` gives `q̇ᵀṀq̇ = 2q̇ᵀCq̇`, which is
the statement that the Coriolis terms do no net work: they redistribute kinetic
energy rather than adding or removing it. This is the property energy-based
stability proofs for impedance and passivity-based controllers rely on.

Note that the identity is a consequence of the Christoffel construction and holds
for any symmetric `M`, so verifying it does not verify `M`. An independent check
compares `C(q,q̇) q̇` against the Newton-Euler recursion, which computes the same
quantity by a different route.

Implementation: `libkinematics/src/dynamics.cpp`. Costs: 990 ns for inverse
dynamics, 7.2 µs for the mass matrix (seven RNE passes).

---

## 11. Trajectory generation

Inverse kinematics produces a goal configuration. It says nothing about time. A
trajectory assigns a configuration to every instant, subject to the actuators'
velocity, acceleration, and jerk limits.

### Path parameterisation

Take the straight line in joint space from `q_start` to `q_goal`, parameterised by
a scalar `s ∈ [0, 1]`:

```
q(s)  = q_start + s·Δq,        Δq = q_goal − q_start
q̇     = ṡ Δq
q̈     = s̈ Δq
q⃛     = ⃛s Δq
```

Every joint is then an affine function of one variable, so the per-joint limits
map directly onto limits on `s`:

```
ṡ_max   = minᵢ ( q̇ᵢᵐᵃˣ / |Δqᵢ| )
s̈_max   = minᵢ ( q̈ᵢᵐᵃˣ / |Δqᵢ| )
⃛s_max   = minᵢ ( q⃛ᵢᵐᵃˣ / |Δqᵢ| )
```

Taking the minimum over joints means the most-constrained joint saturates first
and no joint can exceed its limit. The joints are also automatically
**synchronised**: they start together, finish together, and hold a fixed ratio
throughout, because they share the single parameter `s`.

On `S¹` joints, `Δq` is the geodesic difference from §8 rather than the plain
difference.

### Time-optimal profile on s

What remains is a one-dimensional problem: drive `s` from 0 to 1 in minimum time
subject to `|ṡ| ≤ ṡ_max`, `|s̈| ≤ s̈_max`, `|⃛s| ≤ ⃛s_max`. Without the jerk bound
the solution is the classical trapezoidal velocity profile: accelerate at
`s̈_max`, coast at `ṡ_max`, decelerate at `−s̈_max`, with the coast phase vanishing
for short moves (the triangular case).

With a jerk bound, acceleration itself must ramp, giving the S-curve profile: up
to seven phases, with `s̈` trapezoidal in time and `ṡ` correspondingly smoothed.
Bounding jerk matters because a step in acceleration is a step in commanded
torque, which excites structural modes and shows up as audible knock in a real
machine.

The braking distance, including the jerk-limited ramp needed to bring `s̈` back to
zero, is

```
Δs_brake  =  ṡ²/(2 s̈_max)  +  ṡ · s̈_max /(2 ⃛s_max)
```

and deceleration begins when the remaining distance reaches it.

A goal that changes mid-motion is handled by first braking to rest along the
current direction and then planning afresh. Splicing a new direction into a
moving reference would step the reference velocity, which is precisely the
discontinuity the jerk limit exists to prevent.

---

## 12. Computed-torque control

The trajectory gives a reference `(q_ref, q̇_ref, q̈_ref)`. A controller has to
make the plant follow it.

### The control law

Independent-joint PD ignores the fact that the plant's inertia varies with
configuration and couples the joints, so a single set of gains cannot be
critically damped everywhere. **Computed torque** uses the model to cancel the
nonlinearities. With error `e = q_ref − q`, command

```
τ = M(q)·( q̈_ref + K_d ė + K_p e + K_i ∫e )  +  C(q,q̇) q̇  +  g(q)
```

Substituting into the equations of motion, `M` cancels on both sides and the
closed-loop error obeys

```
ë + K_d ė + K_p e = 0
```

a linear, decoupled, second-order system, identical in every joint at every
configuration. Choosing

```
K_p = ω_n²,        K_d = 2ζω_n
```

with `ζ = 1` gives a critically damped response with no overshoot, and `ω_n` sets
the bandwidth. This is what the mass matrix of §10 is *for*: it converts a
coupled nonlinear plant into `n` independent second-order systems.

### Practical terms

**Friction feedforward.** Coulomb friction is added as `f_c·sign(q̇_ref)`, driven
by the *reference* velocity. Using the measured velocity makes the term a
positive feedback loop around zero crossings, and the joint buzzes.

**Anti-windup.** Actuators saturate. If the integrator keeps charging while the
command is clipped, the accumulated term must later be unwound, producing a large
overshoot. Conditional integration suspends accumulation whenever the command is
saturated and the error would drive it further in the same direction. The
controller's model of saturation has to match the plant's actual limit, otherwise
it integrates against a wall it does not know about.

**Model error.** Cancellation is only as good as `M`, `C`, and `g`. Residual
error appears as a disturbance to the linear error dynamics, which the PD terms
reject in proportion to their bandwidth. This is the reason the model is verified
against an independent oracle in §13 rather than assumed.

---

## 13. Numerical verification

The maths above is checked against MuJoCo, which computes the same quantities for
the same model by unrelated algorithms. Agreement at machine precision across a
large random sample is strong evidence that both are right.

Model consistency, rebuilding the screw parameters from the MJCF's body frames:

| | agreement |
|---|---|
| home pose, position | 2.4 × 10⁻¹⁶ |
| home pose, rotation | 2.6 × 10⁻¹² |
| all seven body screw axes | 1.6 × 10⁻¹⁵ |

Kinematics and dynamics, over 1000 random configurations (50 for the mass
matrix):

| quantity | worst error |
|---|---|
| FK tool pose | 1.3 × 10⁻¹⁴ |
| FK link positions | 5.3 × 10⁻¹⁵ m |
| mass matrix `M(q)` | 8.9 × 10⁻¹⁵ kg·m² |
| gravity `g(q)` | 1.4 × 10⁻¹³ N·m |
| inverse dynamics `τ` | 1.5 × 10⁻¹³ N·m |

Quantities with no external oracle are checked against an independent
construction instead. The body Jacobian is compared with a 5-point numerical
derivative of forward kinematics, which agrees to 10⁻⁵ and shares no code with
the analytic form. The space Jacobian is compared with the left-accumulated screw
form of §6 rather than with `Ad_T J_b`, since that product is how it is computed.
The Coriolis matrix is compared with the Newton-Euler recursion.

Reproduce with:

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build
./build/libkinematics/kin_report
```
