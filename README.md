# 📍 Angle‑Only Motion Estimation — Recovering Trajectory from AoA

> Estimating a moving target’s **starting position** and **constant velocity** using only **angle‑of‑arrival (AoA)** bearings from a maneuvering sensor.

**See the research report:** <a href="AoA_Report.pdf" target="_blank">full academic write‑up</a> (derivations, Jacobians, solver details).

---

## 🚗 What problem is being solved? (quick overview)

A single sensor knows **its own path** and measures only **bearings** to a target once per second. There are **no ranges**. From this time‑sequence of angles, the system recovers the target’s **initial position** $(x_0,y_0)$ and **constant velocity** $(v_x,v_y)$ in the ground plane.

**Why bearings alone can work.** Angles by themselves are low‑information, but a **changing viewpoint** (the sensor turns) imprints enough geometry in the angle history to reveal both where the target started and how it moves.

---

## 🧠 How it works (intuitive walkthrough)

1. **Predict positions**  
   Assume constant‑velocity: position now ≈ start + velocity × time. The sensor’s own position at each second is known.

2. **Turn geometry into angles**  
   For each time step, form the vector from the **sensor** to the **target**, $(\Delta x,\,\Delta y)$, and compute the bearing with `atan2(Δy, Δx)` so the result has the **correct quadrant** and lies in $(-\pi,\pi]$.  
   *What `atan2` is doing:* it’s the “four‑quadrant arctangent.” Unlike $\arctan(\Delta y/\Delta x)$, which can’t tell left from right and breaks when $\Delta x=0$, `atan2` uses the **signs of both inputs** to pick the right direction and handles vertical lines cleanly. The order is `(y, x)` by convention.

3. **Compare to measurements**  
   Subtract the predicted angle from the measured angle to get the **bearing residual**, then **wrap** it back into $(-\pi,\pi]$ so a small physical error near $\pm\pi$ isn’t mistaken for a $2\pi$ jump.

4. **Adjust the guess**  
   A **Gauss–Newton** routine nudges $(x_0,y_0,v_x,v_y)$ to reduce all residuals at once. Intuitively, it asks “which tiny change in the four numbers would make the bearings line up better?” and repeats until the change is negligible.

5. **Why a turn matters**  
   If the sensor flies straight with constant speed, many different target states can produce almost the same angle history. A **turn (cross‑range motion)** makes the angles curve in a way that uniquely fingerprints the target’s position and velocity.

---

## 🧪 Scenario used for the figures

- **Target (truth):** starts at $(0,0)$ and moves northeast at constant speed (e.g., 30 m/s).  
- **Sensor (known path):** starts east of the target; flies **west** for a while, then turns **north** at the same speed (e.g., 35 m/s).  
- **Sampling:** one bearing per second for ~30 s with **1° Gaussian noise**.

This geometry supplies the needed **viewpoint change** to make the parameters observable.

---

## 🎞️ Results (figures)

<div align="center">
  <img width="600" height="650" alt="trajectories" src="https://github.com/user-attachments/assets/3d6863e2-ddc7-4778-98dd-5933fc79985a" />
  <br/>
  <sub><b>Fig A — Trajectories.</b> The turning sensor provides cross‑range motion that disambiguates position and velocity.</sub>
</div>

<div align="center">
  <img width="600" height="650" alt="angles" src="https://github.com/user-attachments/assets/0f90e844-3540-415e-918e-8640cac6c2f0" />
  <br/>
  <sub><b>Fig B — Bearings over time.</b> Blue = true angle; orange = noisy measurements (~1° std). The curvature comes from the sensor’s turn.</sub>
</div>

<div align="center">
  <img width="600" height="650" alt="convergence" src="https://github.com/user-attachments/assets/b5683b1a-ff04-47b5-95cd-7df846c53873" />
  <br/>
  <sub><b>Fig C — Solver convergence.</b> The four parameters settle close to their true values after a few Gauss–Newton iterations.</sub>
</div>

---

## 🧩 Practical guidance

- **Initialization.** Start with a plausible guess: target near the first line‑of‑sight, modest speed. Random restarts help if convergence stalls.  
- **Angle wrapping.** Always wrap residuals to $(-\pi,\pi]$ to avoid artificial $2\pi$ jumps.  
- **Units.** Keep calculations in **radians**; convert to degrees only for display.  
- **Outliers.** Cap the influence of obvious mis‑detections (heavy‑tailed loss or simple clipping).  
- **When it struggles.** Very close passes or nearly straight sensor paths reduce information; a deliberate **S‑ or L‑turn** helps.

---

## 🗣️ Glossary (quick)

**AoA / Bearing** — The direction from sensor to target in the plane, returned by `atan2(Δy,Δx)` in $(-\pi,\pi]$.  
**Angle wrapping** — Mapping any angle back into a principal interval like $(-\pi,\pi]$.  
**Constant‑velocity model** — Position evolves linearly with time; velocity is fixed over the window.  
**Gauss–Newton** — An iterative least‑squares method that adjusts parameters to shrink the sum of squared residuals.

---

## License

MIT — see `LICENSE`.
