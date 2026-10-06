# OPT-1A Objective Diagnosis Report

- Document: `docs/gait_optimization/OPT1A_OBJECTIVE_DIAGNOSIS.md`
- Stage: **Phase OPT-1A — Objective Function Diagnosis (read-only; no code, no optimizer, no fitness change)**
- Date: 2026-10-01 (Asia/Shanghai)
- Workspace: `/home/liu/workspace/Nao_Program/nao_gazebo_ws`
- Author: Independent Optimization Architect (Implementation-Agent role)
- Input phase: OPT-0 `CLOSED` (`OPT0_IMPLEMENTATION_READY` + `OPT0C_RESULT = PASS`, `LEVEL_3 = ACHIEVED`)
- Status at end: **`OPT1A_DIAGNOSIS_READY`** — awaiting independent audit. This is NOT an acceptance.

> Scope guard (from the OPT-1 freeze rule). This document **reads** artifacts only. It changes no
> file under `optimization/`, `tools/`, `src/`, `config/`, and starts no optimization. Every
> conclusion below is derived from an existing OPT-0 artifact named in §0. Where the artifacts do
> not support a conclusion, it is recorded as **Evidence insufficient** (§7), not guessed.

---

## 0. Evidence base & method

Artifacts read (all pre-existing, produced by OPT-0 / OPT-0C):

| Artifact | Used for |
|---|---|
| `optimization/fitness.py` | exact scoring formula, refs, weights, penalties |
| `optimization/parameter_space.py` | the 5 axes and their `[low, high]` bounds/anchors |
| `config/gait_opt_baseline.yaml` | frozen fitness refs/weights, seed policy, frozen background |
| `tools/run_gait_optimization.py` | the ask→run→tell loop, x0, sigma, bounds |
| `tools/analyze_measured_gait.py` | **exact definition** of every metric the fitness consumes |
| `optimization_results/baseline.json` | anchor θ terms + fitness (single seed 101) |
| `optimization_results/best_fitness.json`, `best_theta.yaml` | best θ overrides + provenance |
| `optimization_results/generation_log.json` | all 13 OPT-0 candidates: θ, terms, fitness, status |
| `optimization_results/episodes/*/result_p0a.json` | raw `measured` blocks (displacements, pon, yaw …) |
| `optimization_results/confirmation/rows.json`, `summary.json` | 6 CRN episodes (baseline vs best × 3 seeds) |
| `optimization_results/confirmation/episodes/*/trace/body_pose_trace.jsonl` | trajectories for heading/curvature |
| `docs/gait_optimization/OPT0_IMPLEMENTATION_REPORT.md`, `OPT0C_CONFIRMATION_REPORT.md` | prior findings P-1…P-8, caveats |
| `docs/gait_optimization/CURRENT_IMPLEMENTATION_AUDIT.md` | metric inventory, OQ-1…OQ-5 |

Read-only analyses performed (no repo file written):
1. **Fitness reconstruction** — recomputed `raw`/`fitness` from the stored `terms` and from the
   raw `measured` blocks to confirm the formula reproduces the stored scores.
2. **Trajectory diagnosis** — re-ran the **frozen** window helpers
   (`tools/phase4e_analysis.steady_window/select_indices/unwrap`, exactly as the analyzer uses
   them) over each `body_pose_trace.jsonl`; computed heading consistency, curvature radius,
   `net/path`, `yaw` monotonicity, and per-metre turn rate.

The frozen boundary was verified intact for this phase (`docs/gait_optimization/CURRENT_IMPLEMENTATION_AUDIT.md`
§1; `confirmation/rows.json → provenance.frozen_sha256`). No frozen file was touched.

### 0.1 The scoring function (verbatim from `optimization/fitness.py:148-154`)

```
vel        = clip(speed / V_ref, 0, 1)             speed = forward_disp / effective_duration
instab     = 0.5*clip(tilt/tilt_ref,0,1) + 0.5*clip(|yaw|/yaw_ref,0,1)
energy     = clip((path_over_net - 1) / (pon_ref - 1), 0, 1)      (1.0 if non-finite)
slip       = clip(max_single_step / step_ref, 0, 1)               (1.0 if non-finite)
raw        = 1.0*vel - 0.3*instab - 0.1*energy - 0.1*slip
fitness    = 0.5 * (tanh(raw / 0.3) + 1)           in (0,1)
refs       = {V_ref 0.05 m/s, tilt_ref 30 deg, yaw_ref 45 deg, pon_ref 20, step_ref 0.01 m}
fall = -1.0 ; invalid = -2.0
```

### 0.2 Metric construction (verbatim semantics, `tools/analyze_measured_gait.py`)

- `forward_displacement_m` = `(end − start) · f0`, where **f0 is the window-start measured
  heading** (`:267-275, :333`). The forward axis is **frozen at window start**; it is not the
  instantaneous heading.
- `actual_speed_mean_mps` = `forward_displacement_m / effective_duration` (`:292, :335`) — a
  *signed scalar* over the steady window, not a path speed.
- `lateral_displacement_m` = `(end − start) · left0`.
- `path_over_net` = `path_length / net_displacement`; `None` when `net ≤ 1e-9` (`:338`).
- `max_single_step_m` = largest inter-sample displacement in the window (`:289-291`).
- The analyzer self-reports the anomaly `lateral_dominates_forward` when `|lateral| > |forward|`
  (`:353-354`).

This construction matters for §3–§4: **the fitness never sees the trajectory shape**; it sees only
three endpoint projections (`forward`, `lateral`, `net`), a path-length ratio, a peak per-sample
step, and the tilt/yaw extremes.

---

## 1. Executive Summary

**Question:** is the current fitness landscape encouraging real forward locomotion?

**Answer (evidence-based): No — in its current form the objective does not select for real
forward locomotion.** It selects for *reduced in-place oscillation* with a small, net-forward
residual. The optimiser is optimising a real, monotone quantity, but that quantity is only
loosely coupled to locomotion. Five independent pieces of evidence support this:

1. **The velocity reference is far above the reachable band.** `V_ref = 0.05 m/s`; the largest
   forward speed observed across **all 17 measurable gaits** is `0.0048 m/s` = **9.6 % of
   `V_ref`** (§2.1, M-1). Over 90 % of the velocity term's range was never entered.
2. **No heading/trajectory objective exists.** The only heading pressure is `|Δyaw|` folded into
   the composite `instability` term at effective weight `0.15`. The best gait still turns
   `+16…+28°` while advancing `≈0.05 m` (§3.2).
3. **The endpoint projection credits diagonal drift as "forward".** In `opt0_g01_c00` the body's
   net displacement points **35.3° off world-x** while `f0` is at **3.8°**, yet 85 % of it is
   scored as forward (§3.3).
4. **All three penalty terms bias toward low amplitude / low frequency.** The best θ sits on the
   **lower bound** of `frequency` (0.400 Hz), `sway` (0.0503 rad) and near the lower bound of
   `step` (0.0218) and `swing_height` (0.00656) — i.e. the low-motion corner of the box (§3.4).
5. **Single-episode fitness is noise-dominated at the improvement scale.** Re-running the **same
   θ at the same seed** moved fitness by `0.128` — the same order as the whole baseline→best
   improvement (`+0.15` single-seed) (§4.3).

The OPT-0 loop itself is credible (OPT-0C confirmed a positive, CRN-paired relative improvement).
The problem diagnosed here is **not** the optimiser, the simulator, or the runner — it is that the
*fitness landscape* the loop ascends is a "calmer shuffle" landscape, not a "better walking"
landscape. This is consistent with the OPT-0 report's own finding P-4 ("fitness rewards slow,
low-energy, low-instability gaits") and the OPT-0C caveat "Residual caveats (a)".

> This section states a diagnosis, not a fix. No weight, ref, or formula is proposed (§6).

---

## 2. Current Fitness Behavior Analysis

### 2.1 Velocity term (`w_velocity = 1.0`, `V_ref = 0.05`)

**Observation.** Reachable forward speed over all measurable gaits:

```
opt0 run   (n=11, seed 101): speeds in [-0.0005, +0.0048] m/s
OPT-0C     (n=6,  seeds 101/202/303): baseline +0.00015±0.00046 ; best +0.00343±0.00028 m/s
max |speed| over all 17 measurable gaits = 0.0048 m/s   (= 9.6% of V_ref)
max velocity TERM (= speed/V_ref)        = 0.096        (9.6% of the term's range)
```

Head-room table (fraction of the velocity term never reached), OPT-0 population:

| candidate | vel term | unused |
|---|---:|---:|
| opt0_baseline | 0.022 | 97.8 % |
| opt0_g01_c00 (best) | 0.096 | 90.4 % |
| opt0_g00_c02 | 0.084 | 91.6 % |
| (min, all) | 0.000 | 100 % |

**Diagnosis.** Over the entire explored region the velocity term lives in `[0, 0.096]`. It never
saturates, so it does carry a *gradient*, but the gradient is evaluated in the near-zero regime
of a `tanh`-shaped score whose useful dynamic range sits far above.

The local sensitivity at the best point is

```
raw(best) = -0.1079 ;  d(fitness)/d(speed) = 0.5·sech²(raw/0.3)/0.3 · (w_v/V_ref) = 29.4 per (m/s)
⇒ +0.001 m/s  ⇒  +0.029 fitness
```

while the same-θ / same-seed re-run moved fitness by `0.128` (§4.3). **A velocity gain would have
to exceed ≈0.0044 m/s to stand above one-run noise — i.e. the best gait's speed would need to
roughly double before the velocity term becomes a reliable selection signal.** (Computation in
§4.3.)

Answers to Task-1 questions:

- **Is `V_ref` above the reachable region?** Within the *explored* set, yes — no candidate reached
  10 % of `V_ref`. Whether the region is *physically* unreachable (vs merely unsearched) is
  **Evidence insufficient** (§7, EI-1): no target-saturation probe exists.
- **Can the gradient drive the wrong three behaviours?** Evidence maps as follows:
  - *tiny movement* — supported: the argmax is a tiny-motion gait (§3.4).
  - *unstable forward projection* — supported: best forward progress is a diagonal, `+23°`-curved
    drift, not straight walking (§3.2–§3.3).
  - *accidental velocity* — supported in the sense that `actual_speed_mean` is a *net residual* of
    a large oscillation: best `net ≈ 0.066–0.079 m` is the residue of a `≈1.28–1.41 m` body path
    (§3.3), and the same θ re-run changes it by ±35 % (§4.3).

### 2.2 Instability term (`w_instability = 0.3`, weights tilt/yaw 50/50)

**Observation.** For the best group, `tilt_max ≈ 6.6°` (ref 30 → `tilt_n ≈ 0.22`) and
`|Δyaw| ≈ 16…28°` (ref 45 → `yaw_n ≈ 0.36…0.63`). `instability ≈ 0.27–0.43`. In the audit's
metric inventory, `actual_speed_std_mps` (per-sample forward-speed dispersion) is listed as the
"stability" input (`CURRENT_IMPLEMENTATION_AUDIT.md` §5) — **but the implemented fitness does not
use `speed_std` at all**; `instability` is built only from `tilt` and `|Δyaw|`.

**Diagnosis.** Two consequences: (a) the term is only *partly* about stability — it also proxies
*amplitude* (large sway/swing raises tilt), so it double-counts the low-amplitude bias of §3.4;
(b) the sign of the heading change is discarded (`abs(yaw)`), and no *consistency* of heading
(monotonicity, wander) is measured. See §3.2 and GAP-01/GAP-02.

### 2.3 Energy term (`w_energy = 0.1`, proxy = `path_over_net`, `pon_ref = 20`)

**Observation.** The term is defined only for `path_over_net ∈ [1, 20]`; at `pon ≥ 20` it
saturates at 1.0. In the OPT-0 population **10 of 11 `ok` candidates had `pon ≥ 20` → energy = 1.0
(saturated)**; the anchor had `pon = 118.4 → energy = 1.0`. Only the single best gait pulled `pon`
into the sensitive band (`15.8 → 0.78`; the OPT-0C best runs were `17.9–23.0 → 0.89–1.00`).

**Diagnosis.** For essentially the whole early population the energy term was a **constant 0.1
penalty carrying no gradient**; it only became informative once a candidate already walked
straighter than `pon ≈ 20`. A constant penalty cannot guide search. (This is a property of the
proxy + saturation, not a bug: `path_over_net` is the audit's recorded proxy for energy, OQ-1.)

### 2.4 Slip term (`w_slip = 0.1`, proxy = `max_single_step`, `step_ref = 0.01`)

**Observation.** `slip = clip(max_single_step/0.01, 0, 1)`. Across baseline and best:

```
baseline: max_single_step 0.0063–0.0069 m → slip 0.63–0.69
best    : max_single_step 0.0044–0.0057 m → slip 0.44–0.57
```

**Diagnosis.** (a) `max_single_step` is the **peak per-sample body displacement** (`:289-291`),
i.e. a proxy for *peak body speed*, not foot slip. (b) Its range across every gait is narrow
(0.44–0.69) so, like energy, it is nearly constant and contributes little gradient. (c) Its sign of
influence is unambiguous: it is **lower when motion is slow**, so it adds to the low-amplitude bias
of §3.4. True foot-slip is not measured (audit OQ-2).

---

## 3. Failure Mode Analysis

### 3.1 Shuffle convergence

**Observation.** `path_over_net` and `net/path` (straightness, from the frozen traces):

| episode | path (m) | net (m) | `path_over_net` | `net/path` |
|---|---:|---:|---:|---:|
| opt0_baseline | 2.274 | 0.0192 | 118.4 | 0.008 |
| conf_baseline_s101 | 2.260 | 0.0235 | 96.4 | 0.010 |
| opt0_g01_c00 (best) | 1.292 | 0.0816 | 15.8 | 0.063 |
| conf_best_s101 | 1.280 | 0.0661 | 19.3 | 0.052 |
| conf_best_s202 | 1.411 | 0.0790 | 17.9 | 0.056 |

**Diagnosis.** The optimizer did reduce the oscillation ratio (baseline `net/path ≈ 0.8–1.0 %` →
best `4–6 %`) — a real, monotone improvement of the `energy` proxy. But even the best gait still
travels **~16–23× more path than net displacement**; it is a *quieter shuffle*, not a stride. The
objective rewarded "shop less in place", and that is what it got.

### 3.2 Yaw drift

**Observation.** From the frozen traces (window = frozen `steady_window`):

| episode | `yaw_change` | `yaw_rate` | turn per forward-metre | arc radius |
|---|---:|---:|---:|---:|
| conf_best_s101 | +25.1° | +1.74 °/s | 558 °/m | 0.103 m |
| conf_best_s202 | +28.3° | +1.95 °/s | 516 °/m | 0.111 m |
| conf_best_s303 | +16.1° | +1.11 °/s | 325 °/m | 0.176 m |
| opt0_g01_c00 | +14.4° | +0.99 °/s | 206 °/m | 0.278 m |
| conf_baseline_s101 | +31.1° | +2.15 °/s | 5097 °/m | 0.011 m |

Additional structure: within the window the yaw is **oscillatory with a small net bias** —
`|yaw|` reaches `17–35°`, while the fraction of successive yaw deltas sharing one sign is only
`0.50–0.55` (≈ a coin flip). So `+23°` is a *net drift*, not a steady rotation.

**Diagnosis.** The `+23°` is **not** a mechanism that manufactures forward displacement — a pure
rotation yields `|net| ≈ 0` and would score `vel ≈ 0`. It is instead the visible symptom of an
objective that is **indifferent to heading**: the best gaits are tight arcs (radius 0.10–0.28 m,
i.e. turning 325–558° per metre advanced), and nothing in the fitness requires the end heading to
match the start heading. The only opposing pressure is `|Δyaw|` inside `instability`, worth
`0.5·0.3·(|Δyaw|/45°)` = `0.048` of raw at `Δyaw = 14.4°` — against a total penalty budget of
`0.108`. So this is best classified as **neither** "normal locomotion deviation" **nor** an
actively-exploited loophole, but an **unconstrained degree of freedom**: heading is not an
objective. (Answers the prompt's A-vs-B question: it is A in *mechanism*, but the objective's
blindness to heading is the design limitation — see GAP-01/GAP-02.)

### 3.3 Lateral motion

**Observation.** Lateral magnitude is comparable to (or exceeds) forward, and the frozen analyzer
itself raises `lateral_dominates_forward`:

| episode | forward (m) | lateral (m) | `|lat|/|fwd|` | analyzer anomaly |
|---|---:|---:|---:|---|
| conf_best_s101 | 0.0450 | 0.0485 | 1.08 | `lateral_dominates_forward` |
| conf_best_s202 | 0.0548 | 0.0570 | 1.04 | `lateral_dominates_forward` |
| conf_best_s303 | 0.0495 | 0.0363 | 0.73 | — |
| opt0_g01_c00 | 0.0696 | 0.0427 | 0.61 | — |
| conf_baseline_s101 | 0.0061 | 0.0226 | 3.70 | `lateral_dominates_forward` |

Projection geometry for the best single run `opt0_g01_c00`: `body_dx = 0.0666`, `body_dy = 0.0472`
→ net displacement direction `atan2(0.0472,0.0666) = 35.3°` off world-x; window-start heading
`f0 = 3.8°`. The displacement is therefore **31.5° off the initial heading**, yet
`forward = |net|·cos(31.5°) = 0.0852·|net|` — **85 % of a diagonal drift is credited as forward**.

**Diagnosis.** The fitness uses `forward/t` only. It does not use `lateral`, `net`, or any
forward-vs-total ratio. Consequently a curved/side-shuffling gait that happens to place its net
displacement near `f0` is scored as "walking". That the *frozen analyzer independently flags two of
the three best runs as lateral-dominant* is direct artifact evidence that the current objective is
not distinguishing true walking from a side/curved shuffle.

### 3.4 Low-movement attractor

**Observation A — the argmax is at the low-motion corner of the box.** Best θ vs bounds
(`parameter_space.py:31-37`):

| axis | best value | bounds | position |
|---|---:|---|---|
| `step_length` | 0.02185 m | [0.02, 0.08] | **+2.9 % into range** |
| `frequency` | 0.40001 Hz | [0.40, 1.60] | **at lower bound** |
| `swing_height` | 0.00656 m | [0.005, 0.040] | **+4.5 % into range** |
| `double_support` | 0.1204 | [0.05, 0.45] | +17.6 % into range |
| `sway_amplitude` | 0.05029 rad | [0.05, 0.30] | **at lower bound** |

Four of five axes sit at or near their lower bounds (three at ≲5 % into range). The search was
driven to the smallest amplitude/frequency corner available.

**Observation B — the "do-nothing" probe.** Because the three penalty terms can all be driven
toward zero by *not moving much* (small tilt/yaw, small peak step) while `energy` saturates at a
bounded `0.1`, an **idealised still robot** (`vel=0, instab=0, energy=1, slip=0`) scores:

```
raw = -0.10  →  fitness = 0.5·(tanh(-0.10/0.3)+1) = 0.3392
```

which is **above the best gait actually found** (`0.3275`) and far above the anchor (`0.1769`).

**Diagnosis.** The objective's preferability ordering near the "quiet" end of the space is at
least as favourable to *not walking* as to the best walking gait found. The optimizer's improvement
over the anchor is therefore better explained as "**reduce the in-place oscillation and tilt
penalties faster than you lose (already‑negligible) velocity reward**" than as "learn to walk".
This is a **low-movement attractor**, and it is visible both in the search result (Obs. A) and in
the score structure of the "still" probe (Obs. B).

> Caveat (not a retraction): the still probe uses `instab = 0`. A *realistic* still robot with, e.g.,
> `tilt = 5°` and `|Δyaw| = 10°` gives `instab = 0.194 → raw = -0.158 → fitness ≈ 0.259`, i.e.
> **below** the best found gait. Whether a *realisable* θ actually attains the 0.3392 corner is
> **Evidence insufficient** (§7, EI-2). Obs. A (search behaviour) is the load-bearing evidence here;
> Obs. B is a structural bound.

---

## 4. Reward Landscape Diagnosis

### 4.1 What the optimizer is *in fact* optimizing

Reconstructed per-candidate terms (all reproductions match the stored fitness to <1e-3):

```
id               freq   step    vel    instab energy   raw     fitness
opt0_baseline    0.500  0.0200  0.022  0.282  1.000   -0.2306  0.1769
opt0_g01_c00     0.400  0.0218  0.096  0.274  0.780   -0.1079  0.3275   <- best
opt0_g02_c02     0.563  0.0208  0.068  0.249  1.000   -0.1721  0.2409
```

Effective objective at the improvement scale:

- **Dominant gradient:** `−energy` (i.e. reduce `path_over_net`) *once `pon < 20`*; below that it
  saturates and stops guiding (§2.3).
- **Secondary:** `−instability` (reduce `tilt`, reduce `|Δyaw|`).
- **Weak pull:** `+velocity`, evaluated in the `[0, 0.096]` regime (§2.1).
- **Near-constant:** `−slip` (§2.4).

Population correlations (OPT-0 `ok` set, n = 11):

```
corr(fitness, speed)          = +0.713     <- the velocity term does track speed
corr(fitness, path_over_net)  = -0.284     <- weaker; energy is often saturated/constant
```

So the objective is **partly** speed-tracking (r = +0.71), which is why OPT-0C shows a positive
paired improvement — but the same objective is also strongly satisfied by reduced oscillation and
reduced tilt/yaw at near-zero speed (§3.4).

### 4.2 Reward-hacking risk: bug vs objective-design limitation

Applying the prompt's distinction:

| Candidate cause | Verdict | Evidence |
|---|---|---|
| missing heading constraint | **design limitation** | §3.2: no term constrains end vs start heading; `|yaw|` only weakly penalised |
| missing trajectory constraint | **design limitation** | §0.2/§3.3: only endpoint projections + `path/net` are seen |
| velocity-metric ambiguity | **design limitation** | §3.3: `forward` is a projection of a diagonal drift onto a *window-start* axis; 85 % of a 31.5°-off drift is credited |
| insufficient behaviour constraints | **design limitation** | §2.2/§2.3: `speed_std`, contact alternation, `lateral` are measured but unused |
| a *bug* (code defect) | **not supported** | the formula reproduces stored scores exactly; the `tanh` map and `clip` semantics behave as documented; `_scan_trace_fall` is independent of the window |

**Diagnosis:** the observed "high-ish reward + poor locomotion" is **not** a bug. It is the
intended-though-misaligned behaviour of a bounded, proxy-based objective: it is monotone, it is not
saturated for non-fall gaits, and it *does* reward the one thing it measures (`forward/t`) — but
`forward/t` itself is a weak proxy for locomotion (§3.3), and the penalty terms dominate the raw
score at the amplitude scale the robot actually operates at.

### 4.3 Signal-to-noise consequence for the landscape

**Observation.** The same θ at the same physics seed, re-run in OPT-0C:

| group | θ | seed | fitness (OPT-0) | fitness (OPT-0C) | Δ | forward Δ | speed Δ |
|---|---|---:|---:|---:|---:|---:|---:|
| baseline | anchor | 101 | 0.1769 | 0.1178 | −0.059 | 0.0162→0.0061 | 0.00112→0.00042 |
| best | `g01_c00` | 101 | 0.3275 | 0.1998 | **−0.128** | 0.0696→0.0450 | 0.00481→0.00311 |

**Diagnosis.** Single-episode fitness has a re-run spread (~0.13) that is **the same order as the
entire single-seed baseline→best improvement (+0.15)** and far larger than the ~0.03–0.06 spread
between most candidates. Combined with §2.1 (`+0.001 m/s ⇒ +0.029 fitness`), the **fitness
landscape as currently sampled is noise-dominated for ranking nearby gaits**: the optimiser is
partly ranking physics RNG, not θ. This is the OPT-0 report's P-3 ("physics is not reproducible
run-to-run even with a fixed `gz_seed`") expressed as an objective-space quantity. The OPT-0C CRN
design *removes* this for a *paired* baseline-vs-best decision, but the *optimizer itself* used
single-episode scores as its selection signal (P-3 status: **open**).

---

## 5. Objective Gap List

Gaps only. No formula, weight, ref, or code is proposed here (see §6).

| id | Gap | What is missing | Evidence |
|---|---|---|---|
| **GAP-01** | Heading stability | No term requires the window-end heading to remain near the window-start heading beyond a weak `\|Δyaw\|` penalty inside `instability`; heading *sign* and *wander* are not measured. | §3.2 |
| **GAP-02** | Direction consistency | No term measures whether the body's velocity/heading is *consistent* over the window (monotonicity, dwell, variance). `actual_speed_std_mps` exists but is unused. | §2.2, §3.2 |
| **GAP-03** | True forward-locomotion metric | No forward-vs-total ratio (`\|forward\|/net`, `\|forward\|/path`) and no `lateral` term: the objective cannot separate a stride from a diagonal/curved shuffle. | §3.3 |
| **GAP-04** | Trajectory-shape constraint | Only endpoint projections + `path/net` are seen; net *displacement* is scored, not the shape/straightness of the *path* (a curved arc with a lucky chord is fully rewarded). | §0.2, §3.3 |
| **GAP-05** | Velocity reference calibration | `V_ref = 0.05` lies far above the reachable band (max 9.6 %); the velocity term is calibrated for a speed the current gait family does not approach. | §2.1 |
| **GAP-06** | Energy-term dynamic range | `energy` saturates (`pon ≥ 20 → 1.0`); it is a constant penalty for the typical gait and only informative near `pon ≈ 1`. | §2.3 |
| **GAP-07** | Anti-stationary / motion-presence term | Nothing prevents the low-motion attractor: the penalty set is minimised by *moving less*. No term requires a minimum useful displacement/speed before reward. | §3.4 |
| **GAP-08** | Real energy / real slip | `energy` and `slip` are proxies (`path_over_net`, `max_single_step`), not energy/slip; `slip` in particular proxies peak body speed, biasing toward slowness. | §2.3, §2.4 (audit OQ-1/OQ-2) |
| **GAP-09** | Stability metric fidelity | `instability` blends tilt and `\|Δyaw\|` but omits the available `actual_speed_std_mps`, and double-counts amplitude (tilt rises with sway/swing). | §2.2 |
| **GAP-10** | Evaluation-noise control | The optimizer scored single episodes; no N-seed/CRN aggregation is in the selection path, so reward ≈ θ-signal + physics noise. | §4.3 |
| **GAP-11** | Success/criterion semantics | There is no objective-level definition of "a real walking gait"; `fitness` mixes a scale-free ratio, a thin velocity proxy, and saturating penalties, so no score threshold means "walks". | §1, §4 |

---

## 6. Recommendation Boundary

- OPT-1A **does not** modify, and this document **does not propose**, any change to:
  `optimization/fitness.py`, reward refs/weights, `optimization/parameter_space.py`,
  `tools/run_gait_optimization.py`, CMA-ES parameters, the evaluation budget, the gait generator,
  the runner, or the frozen simulator.
- GAP-01…GAP-11 are **inputs** to the next stage, not decisions. Selecting which gaps to close,
  and *how* (which metrics, refs, weights, and any N-seed policy), is the remit of **OPT-1B —
  Fitness Contract RFC**, which must itself pass the independent-audit gate before any code is
  written.
- No gap above is declared "the" fix, and no ordering among them is implied. In particular, GAP-10
  (evaluation noise) is an *objective-evaluation* property that OPT-1B must decide whether to
  address before or together with GAP-01…GAP-09.

```text
OPT-1A MAY:        diagnose (this document)
OPT-1A MUST NOT:   change code, fitness, optimizer, budget
OPT-1B MAY (after RFC acceptance): design Fitness Contract v2 (gaps → terms)
```

---

## 7. Evidence Insufficiency Register

Recorded explicitly rather than guessed.

| id | Statement that is *not* supported by the current artifacts | What measurement would settle it |
|---|---|---|
| **EI-1** | "`V_ref = 0.05` is *physically* unreachable by this gait family." Only *explored-set* evidence exists (max 9.6 % of `V_ref`). | a target-saturation probe: sweep `(step_length, frequency)` toward the producer caps and record the max stable `actual_speed_mean`. |
| **EI-2** | "A realisable θ attains the idealised 'still' score (0.3392)." Only the structural bound is computed; no still-gait θ was measured. | run a low-amplitude θ and measure its real `tilt`/`yaw`/`slip`. |
| **EI-3** | "The 0.128 same-θ divergence is attributable to dartsim/bridge nondeterminism." P-3 asserts this; no isolated decomposition (RNG vs bridge vs real-time scheduling vs window truncation) exists. | controlled repeatability study isolating each suspect. |
| **EI-4** | "The true energy / foot-slip costs differ from the proxies." Proxies are known (audit OQ-1/OQ-2) but no true-signal derivation exists to compare against. | read-only `Σ\|Δq\|` derivation (command trace) and a true foot-slip estimate (contact vs body motion). |
| **EI-5** | Any ranking of GAP importance. | OPT-1B analysis, not OPT-1A. |

Note on `trace_truncated`: all analysed episodes report `trace_truncated = true` with
`effective_duration ≈ 14.48 s` vs nominal `≈14.5 s` (missing ≈0.02 s). The truncation is ≈0.1 % of
the window and is therefore not a candidate explanation for any finding above; it is recorded here
for completeness.

---

## 8. Status

```text
OPT1A_STATUS        = OPT1A_DIAGNOSIS_READY
SCOPE               = read-only diagnosis (no code / no optimizer / no fitness change)
FROZEN_FILES_CHANGED= 0
CONCLUSION          = objective as implemented does NOT select for real forward locomotion;
                      it selects for a lower-oscillation, lower-tilt shuffle (11 gaps recorded).
NEXT                = independent audit (GLM-5.3)  ->  OPT-1B Fitness Contract RFC
```

### Provenance

```text
git_commit            5526a26e35e2001f295bee3cb4ec89760165089d
anchor_sha256         116dde704ba2d4bbf3f5afaaefc0c5a77a478541d341a86536e38ac0426bc13b
best_theta_sha256     15371d91739aa168faab14509f4e2e922ca986f1c315ad8da769173027e52278
frozen_sha256         phase4b_producer aab60f6f… | run_p0a_gait_candidate 00aef712… | nao_gait_phase 4cee76de…
primary data          optimization_results/{baseline.json,best_fitness.json,generation_log.json,confirmation/rows.json}
traces                optimization_results/{episodes,confirmation/episodes}/*/trace/body_pose_trace.jsonl
method                read-only; frozen `phase4e_analysis` window helpers reused; no repo write
```

End of report.
