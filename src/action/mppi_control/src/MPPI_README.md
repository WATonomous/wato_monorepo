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

```markdown
## Critic Cost Function

The critic provides the running cost  
\[
q(x_t,u_t,u_{t-1})
\]
and terminal cost  
\[
\phi(x_T)
\]
for trajectory evaluation.

Let the nearest reference path give:

- Tangent: \( t = [t_x, t_y] \)
- Lateral error: \( e_{lat} \)
- Heading error:
\[
\Delta\psi = \text{wrap}(\psi - \text{atan2}(t_y,t_x))
\]
- Forward progress:
\[
p = v(\cos\psi\,t_x + \sin\psi\,t_y)
\]

---

### Running Cost

Tracking:
\[
q_{track} = dt\left(
w_{dev} e_{lat}^2 +
w_{heading} \Delta\psi^2
\right)
\]

Progress reward:
\[
q_{prog} = - w_{progress}\, dt\, p
\]

Smoothness:
\[
j = \frac{a_t-a_{t-1}}{dt}, \quad
\dot{\delta} = \frac{\delta_t-\delta_{t-1}}{dt}
\]
\[
q_{smooth} =
w_{jerk} j^2 +
w_{steer\_rate} \dot{\delta}^2
\]

Control effort:
\[
q_{effort} = dt\left(
w_a a^2 +
w_\delta \delta^2
\right)
\]

Total running cost:
\[
\boxed{
q = q_{track} + q_{prog} + q_{smooth} + q_{effort}
}
\]

---

### Terminal Cost

\[
\boxed{
\phi =
w_{dev}^T e_{lat}^2 +
w_{heading}^T \Delta\psi^2
- w_{progress}^T \, p
}
\]

Invalid costs:
\[
J = 10^6 \quad \text{if non-finite}
\]
```
