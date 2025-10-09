# (﹙˓ 📟 ˒﹚) Angle‑Only Trajectory Estimation

> Recovering a moving target’s **starting position** and **constant velocity** using only **AoA** (Angle of Arrival) bearings from a maneuvering sensor.

**See the research report:** <a href="AoA_Report.pdf" target="_blank">See here</a> for the full academic write‑up (derivations, Jacobians, solver details).  

---

## 🚗 Scenario at a glance

One sensor moves along a **known path** and measures only the **bearing** to a target once per second. There are **no range** readings. From the time‑sequence of angles, the system estimates the target’s **initial position** $(x_0,y_0)$ and **constant velocity** $(v_x,v_y)$ in the East–North plane.

<div align="center">
  <img width="434" height="292" alt="SCR-20250929-nybu" src="https://github.com/user-attachments/assets/9acc1132-dfa2-476b-b711-b55f635af999" />
  <br/>
  <sub><b>Fig S — Project scenario.</b></sub>
</div>

At time $t_i$, the sensor at $(\xi_p,\eta_p)$ points to the user at $(\xi,\eta)$; the measured bearing is $\theta(t_i)$.


---

## 🧠 What each piece does (intuitive)

### 1) Motion model (target)
Assume **constant velocity (CV)** over the 30‑s window:
- Position now = **start + velocity × time**.  
- Unknowns to estimate: $(x_0,y_0,v_x,v_y)$.

### 2) Known sensor path
At every second $t_k$, the sensor’s position $(x_p(t_k),y_p(t_k))$ is known (westward leg, then a northward turn in this project). This **viewpoint change** is what injects information into the angles.

### 3) Measurement model (how bearings are computed)
For each time step, form the vector from **sensor → target**:
- $\Delta x_k = x(t_k) - x_p(t_k)$,  $\Delta y_k = y(t_k) - y_p(t_k)$  
- The ideal bearing is the **four‑quadrant arctangent**  
  $$\theta_k = arctan2(\Delta y_k,\ \Delta x_k).$$

- Unlike $\arctan(\Delta y/\Delta x)$, `atan2(y,x)` uses the **signs of both inputs** to return the correct direction in $(-\pi,\pi]$.  
- It behaves well on **vertical lines** ($\Delta x=0$) and distinguishes left/right quadrants correctly.  
- In short: it’s the right tool whenever you want a true **heading** from a vector.

### 4) Noise and wrapping
Measurements are noisy angles $z_k=\theta_k+w_k$ (here, $\sim\!\mathcal N(0,(1^\circ)^2)$). When comparing model vs. data, angle differences are **wrapped** to $(-\pi,\pi]$ so a tiny physical error near $\pm\pi$ isn’t mistaken for a $2\pi$ jump.

### 5) Fitting all angles at once
A **Gauss–Newton** solver adjusts $(x_0,y_0,v_x,v_y)$ to make **all** wrapped angle residuals small at the same time. Conceptually: “what little change to the four numbers would swing all predicted bearings toward the measured ones?” Repeat until changes are tiny.

### 6) The turn is crucial (observability)
If the sensor never turns, many different target states can produce nearly the same angle history. The **west‑then‑north** path creates **cross‑range motion**, making the angle curve distinctive enough to reveal both position **and** velocity.

---

## 🧪 The specific setup used

- **Target (truth):** starts at $(0,0)$, moves northeast at constant speed (≈30 m/s).  
- **Sensor path (known):** starts east, goes **west** for 15 s, then **north** for 14 s at ≈35 m/s.  
- **Sampling:** 1 angle per second, **30** samples total.  
- **Noise:** Gaussian, **1°** standard deviation.

This geometry supplies the needed viewpoint change for reliable estimation.

---

## 🎞️ Results 

Now that the workflow is clear, the three outputs below illustrate the run on the scenario above.

<div align="center">
  <img width="600" height="650" alt="trajectories" src="https://github.com/user-attachments/assets/3d6863e2-ddc7-4778-98dd-5933fc79985a" />
  <br/>
  <sub><b>Fig A — Trajectories.</b> The turning sensor provides cross‑range motion that disambiguates position and velocity.</sub>
</div>

<div align="center">
  <img width="600" height="650" alt="angles" src="https://github.com/user-attachments/assets/0f90e844-3540-415e-918e-8640cac6c2f0" />
  <br/>
  <sub><b>Fig B — Bearings over time.</b> Blue = true angle; orange = noisy measurements (~1° std). The curvature arises from the sensor’s turn.</sub>
</div>

<div align="center">
  <img width="600" height="650" alt="convergence" src="https://github.com/user-attachments/assets/b5683b1a-ff04-47b5-95cd-7df846c53873" />
  <br/>
  <sub><b>Fig C — Solver convergence.</b> The four parameters settle close to truth after a few Gauss–Newton iterations.</sub>
</div>

---
## 🧩 Practical guidance

- **Initialization.** Start near the first line‑of‑sight with a reasonable speed; try a few random restarts if convergence stalls.  
- **Units.** Keep all calculations in **radians**; convert to degrees for plots only.  
- **Robustness.** Clip extreme residuals or use a gentle robust loss if a few angles are suspect.  
- **When it struggles.** Nearly straight sensor paths or very close passes reduce information; deliberate **L/S‑turns** help.  
- **Extensions.** Constant Velocity works for this window; for long windows consider constant‑acceleration or coordinated‑turn models (and EKF/UKF) as noted in the report.
  
---

## 🗣️ Glossary (quick)

**AoA / Bearing** — Direction from sensor to target; computed with `atan2(Δy,Δx)` in $(-\pi,\pi]$.  
**Angle wrapping** — Mapping any angle back into a principal interval (e.g., $(-\pi,\pi]$).  
**Constant‑velocity model** — Position evolves linearly with time; velocity fixed over the window.  
**Gauss–Newton** — Iterative least‑squares method that adjusts parameters to shrink the total squared angle error.  
**Observability** — Whether the angle history carries enough information to infer the state (improved by sensor maneuvers).

---

## License

MIT — see `LICENSE`.
