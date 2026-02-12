# MPPI — Mathematical Summary

## Problem Setup

State:
```
x = [px, py, yaw, v]
```

Control:
```
u = [a, δ]
```

Dynamics (kinematic bicycle):
```
px_{t+1}  = px_t + v_t cos(yaw_t) dt
py_{t+1}  = py_t + v_t sin(yaw_t) dt
yaw_{t+1} = yaw_t + (v_t / L) tan(δ_t) dt
v_{t+1}   = v_t + a_t dt
```

We optimize a control sequence over a horizon:
```
U = {u_0, …, u_{T-1}}
```

---

## MPPI Sampling

Sample noisy control sequences:
```
ε_k,t ~ N(0, Σ)

u_k,t = u_t + ε_k,t
```

Roll out trajectories:
```
x_{k,t+1} = f(x_{k,t}, u_k,t)
```

---

## Trajectory Cost

Total cost of trajectory `k`:

```
J_k = Σ_{t=0}^{T-1} q(x_{k,t}, u_{k,t}, u_{k,t-1}) + φ(x_{k,T})
```

MPPI importance sampling correction:

```
J_k += Σ_t λ (u_tᵀ Σ⁻¹ ε_k,t)
J_k += Σ_t ½ λ (ε_k,tᵀ Σ⁻¹ ε_k,t)
```

---

## Weight Computation (Softmin)

For N trajectories:

```
w_k = exp(-(J_k - J_min)/λ)
w_k = w_k / Σ_i w_i
```

λ = temperature.

---

## Path Integral Control Update

Noise-weighted update of the nominal controls:

```
Δu_t = Σ_k w_k ε_k,t
u_t ← u_t + Δu_t
```

---

## Receding Horizon

Execute first control:
```
u_exec = u_0
```

Shift horizon and repeat.

---

## Full MPPI Loop

```
repeat:
    sample noisy controls
    simulate trajectories
    compute costs J_k
    compute weights w_k
    update controls u_t += Σ w_k ε_k,t
    apply u_0
```
## Critic Cost Function

The critic evaluates the trajectory by providing the **running cost** $q(x_t, u_t, u_{t-1})$ and the **terminal cost** $\phi(x_T)$.

### Reference Path Metrics
Given the nearest reference path, we define the following states:

* **Tangent:** $t = [t_x, t_y]$
* **Lateral error:** $e_{lat}$
* **Heading error:** $$\Delta\psi = \text{wrap}(\psi - \operatorname{atan2}(t_y, t_x))$$
* **Forward progress:** $$p = v(\cos\psi \, t_x + \sin\psi \, t_y)$$

---

### Running Cost

The total running cost is the summation of tracking precision, progress, smoothness, and control effort.

#### 1. Tracking
$$q_{track} = dt \left( w_{dev} e_{lat}^2 + w_{heading} \Delta\psi^2 \right)$$

#### 2. Progress Reward
$$q_{prog} = - w_{progress} \, dt \, p$$

#### 3. Smoothness
Defined by the jerk ($j$) and steering rate ($\dot{\delta}$):
$$j = \frac{a_t - a_{t-1}}{dt}, \quad \dot{\delta} = \frac{\delta_t - \delta_{t-1}}{dt}$$
$$q_{smooth} = w_{jerk} j^2 + w_{steer\_rate} \dot{\delta}^2$$

#### 4. Control Effort
$$q_{effort} = dt \left( w_a a^2 + w_\delta \delta^2 \right)$$

> **Total Running Cost:**
> $$q = q_{track} + q_{prog} + q_{smooth} + q_{effort}$$

---

### Terminal Cost
The terminal cost penalizes the final state of the trajectory to ensure long-term stability:

$$\phi = w_{dev}^T e_{lat}^2 + w_{heading}^T \Delta\psi^2 - w_{progress}^T \, p$$

---

### Constraint Handling
If the resulting trajectory is non-finite or violates hard constraints, an **invalid cost** is assigned:
$$J = 10^6 \quad \text{if non-finite}$$