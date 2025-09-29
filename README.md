# 📍 Angle‑Only Tracking — Estimating Motion from AoA (Angle of Arrival)

> Estimating a moving target’s **initial position** and **constant velocity** using only **angle‑of‑arrival (AoA)** measurements from a moving sensor.

**See the research report:** <a href="AoA_Report.pdf" target="_blank">full academic write‑up</a> (derivations, Jacobians, and solver details).  

---

## 🎞️ Figures 

<div align="center">
  <img width="600" height="650" alt="trajectories" src="https://github.com/user-attachments/assets/3d6863e2-ddc7-4778-98dd-5933fc79985a" />
  <br/>
  <sub><b>Fig A — Trajectories.</b> Target moves northeast at constant speed; the sensor flies west then turns north.</sub>
</div>


<div align="center">
  <img width="600" height="650" alt="angles" src="https://github.com/user-attachments/assets/0f90e844-3540-415e-918e-8640cac6c2f0" />
  <br/>
  <sub><b>Fig B — Angle‑of‑arrival over time.</b> Blue = true angle; orange = noisy measurements (~1° std).</sub>
</div>


<div align="center">
  <img width="600" height="650" alt="convergence" src="https://github.com/user-attachments/assets/b5683b1a-ff04-47b5-95cd-7df846c53873" />
  <br/>
  <sub><b>Fig C — Solver convergence.</b> Estimated position/velocity settle close to truth within a few iterations.</sub>
</div>


---

## 🚗 What problem is being solved?

Only **bearings (angles)** to the target are available—no ranges. A single moving sensor collects these bearings once per second while it follows a known path. From the *sequence* of angles, the system recovers the target’s **starting location** and **constant velocity** in the ground plane.

Why this is interesting: angles are **low‑information** by themselves. The key is that a **changing viewpoint** makes the pattern of angles depend on both where the target started and how it moves.

---

## 🧠 How it works (intuitive walkthrough)

1. **Predict where everyone is**  
   A constant‑velocity model says the target’s position changes linearly with time: position now = start + velocity × time. The sensor’s path is known for each timestamp.

2. **Turn geometry into angles**  
   For each time step, form the vector from sensor → target and compute its **bearing** with `atan2(Δy, Δx)` so the quadrant is correct.

3. **Compare to measurements**  
   Subtract the predicted angle from the measured one to get an **angle error** (wrapped to $(-\pi,\pi]$ so differences near $\pi$ don’t jump).

4. **Nudge the guess to reduce error**  
   A standard **Gauss–Newton** routine tweaks the four unknowns — start $(x_0,y_0)$ and velocity $(v_x,v_y)$ — to make all angle errors small at once. The report shows the derivatives that tell the solver *which way to move* each parameter.

5. **Stop when stable**  
   When the total error stops dropping (or changes are tiny), the parameters are taken as the estimate. Fig C shows a typical convergence trace.

---

## 🧪 Scenario behind the figures (typical test)

- **Target (truth):** starts at $(0,0)$ and moves northeast at a constant speed (e.g., 30 m/s).  
- **Sensor (known path):** starts to the east; flies **west** for a while, then turns **north** at the same speed (e.g., 35 m/s).  
- **Sampling:** one angle per second for ~30 s with **1° Gaussian noise**.

This turn in the sensor path is crucial: without some **cross‑range** motion, the problem is poorly constrained.

---

## 🧩 Practical design choices

- **Initialization.** Start with a rough guess (e.g., target near the first line of sight, modest speed). The solver will refine it.  
- **Angle wrapping.** Always wrap differences to $(-\pi,\pi]$ to avoid spurious $2\pi$ jumps.  
- **Outlier handling.** If a few angles look inconsistent (glints, mis‑detections), cap their influence or drop them.  
- **Units.** Keep everything in **radians** internally; display in degrees if desired.  
- **Termination.** Stop when the error reduction per iteration is tiny or a max‑iteration cap is hit.

For full equations and Jacobians, see the <a href="Report.pdf" target="_blank">research report</a>.

---

## 📖 Reading the plots

- **Fig A (Trajectories).** A turning observer creates geometry that disambiguates both position and velocity. Straight‑line observers give far weaker angle patterns.  
- **Fig B (Angles).** The smooth blue curve is the angle that would be seen with perfect measurements; orange points show the noisy angles fed to the solver.  
- **Fig C (Convergence).** Each line tracks an estimated parameter moving toward its true value; flat lines mean the solution has stabilized.

---

## ⚠️ Limitations & edge cases

- **No range information.** Bearings alone cannot fix absolute scale from a stationary observer. **Observer maneuvers** (turns, speed changes) are needed.  
- **Near‑singular passes.** If the target passes very close to the sensor, small bearing noise can cause large errors.  
- **Model mismatch.** The method assumes **constant velocity** during the window; strong target maneuvers break that assumption.  
- **Initialization sensitivity.** Extremely bad initial guesses can stall the solver; a few random restarts help on tough cases.

---

## 🗂️ What’s in the repo (typical)

- `README.md` — this document.  
- `Report.pdf` — the full academic write‑up (rename your file or update the link).  
- `figs/` — three images for Fig A/B/C (use your own links above if storing elsewhere).  
- `scripts/` — optional notebook/script that generates the figures from the scenario.

---

## 🧠 Glossary (one‑liners)

**AoA / Bearing** — Angle from the sensor to the target in the ground plane.  
**Constant‑velocity model** — Position evolves linearly with time; velocity is fixed.  
**Gauss–Newton** — Solver that adjusts parameters to minimize total squared error.  
**Angle wrapping** — Converting any angle to a principal interval like $(-\pi,\pi]$ to avoid jumps.

---

## License

MIT — see `LICENSE`.
