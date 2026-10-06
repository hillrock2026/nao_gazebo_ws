# OPT-2A REV2 — CHANGELOG

- Artifact: `docs/gait_optimization/OPT2A_FITNESS_V2_DESIGN_RFC_REV2.md`
- Status token: `OPT2A_REV2_STATUS = RFC_DRAFT_READY`
- Post-audit state: `POST_AUDIT_REMEDIATION = TECHNICAL_REPAIRS_COMPLETE`,
  `CONTRACT_STATE = WAITING_CONTRACT_OWNER_RULING`
- Revision chain: REV0 (REJECTED) → REV1 (REJECTED) → **REV2 (current, post-remediation)**
- Drivers: `docs/gait_optimization/OPT2A_FOCUSED_REAUDIT_REPORT.md`
  (`REAUDIT_DECISION = REJECTED`; `OPT2A-MAJ-1-R`; `OPT2A-FR-MIN-1…FR-MIN-5`), and
  `docs/gait_optimization/OPT2A_REV2_FOCUSED_REAUDIT_REPORT.md`
  (GLM-5.3; `AUDIT_DECISION = REJECTED`, `OPT-2B_ALLOWED = NO`; `R2-MIN-1…5`)
- Date: 2026-10-02
- Base commit: `5526a26e35e2001f295bee3cb4ec89760165089d`

This revision closes **one** finding (`OPT2A-MAJ-1-R`) and **five** minors
(`FR-MIN-1…FR-MIN-5`), then receives a **post-audit remediation** round (§R2 below) for the
GLM-5.3 re-audit's five new minors `R2-MIN-1…5`. It is a semantics correction; no fitness redesign.

## Files written

| File | Change |
|---|---|
| `OPT2A_FITNESS_V2_DESIGN_RFC_REV2.md` | **new** — the REV2 RFC (then **edited** in the R2 remediation) |
| `OPT2A_REV2_CHANGELOG.md` | **new / edited** — this file |
| `OPT2A_REV2_SELF_AUDIT_REPORT.md` | **new / edited** — self-audit (R2 section added) |
| `OPT2A_GAP11_OWNER_DECISION_REQUEST.md` | **new** — R2 §5: contract-owner decision request |
| `OPT2A_POST_GLM_REMEDIATION_REPORT.md` | **new** — R2 §5: remediation report |
| `OPT2A_FITNESS_V2_DESIGN_RFC.md` | **edited** — supersession banner added (FR-MIN-5) |
| `OPT2A_FITNESS_V2_DESIGN_RFC_REV1.md` | **edited** — supersession banner added (FR-MIN-5) |

No frozen file, no tracked source/config file, no OPT-1B/OPT-1A file was modified.

## Changes (REV1 → REV2)

### C-1 — Single success definition (Req 1)
- **Was:** two non-equivalent heads — behavioural conjunctive (`∀k c_k ≥ α_k`) and operational
  superlevel set (`S(c) ≥ S*`) — joined by "Operationally:".
- **Now:** **only** the behavioural conjunctive definition (§4.2):
  `Success(g) ⟺ status(g)=ok ∧ ∀k: c_k(g) ≥ α_k`.
- **Deleted phrasings:** `Success ⟺ Fit ≥ S*`, "superlevel-set success", "the fitness threshold
  defines success" (§4.3). `S*` remains only as a *derived reference level* (§6.3), not a success
  criterion.
- **Finding:** MAJ-1-R(a),(b).

### C-2 — Episode-to-gait aggregation defined; order fixed (Req 2)
- **Was:** `c(g)` undefined; protocol score `median_i(raw^i)`; HM/median silent.
- **Now:** fixed order (§5.2): `c_k(g) = median_i c_k^i` **first**, then `Fit(g) = HM(c(g))`.
  Rationale (§5.3): the success definition is per capability, so repeat aggregation is per
  capability; median for C6/GAP-10 robustness; determinism.
- `HM(median)` vs `median(HM)` demonstrated to differ (0.978 vs 0.357 on the re-audit's triple);
  `median(HM)` rejected.
- **Finding:** MAJ-1-R(c).

### C-3 — T-success redesigned (Req 3)
- **Was:** unsuccessful fixture defined by `S(c^U) < S*` (circular); only deep failure; "exhaustive
  form" was a restatement of the definition.
- **Now:** fixtures defined **behaviourally** (`∃j: c_j < α_j`); **deep** (F-D) **and** **shallow**
  (F-Sh, F-Sh′ in the band `[S*/(5−4S*), α_j)`) cases; `Fit` computed; expected outcome stated —
  including that the shallow case **exposes** the residual (§7.3).
- **Finding:** MAJ-1-R(d).

### C-4 — GAP-11 restated as consistency + disclosed residual (Req 7)
- **Was:** "discharge by construction" (REV1) — tautological/vacuous.
- **Now:** **Theorem 1** `Success(g) ⟹ Fit(g) ≥ S*` proved (harmonic mean monotonicity);
  **Corollary 1** contrapositive. `A ⊆ R`, `A ≠ R` proved (explicit witness + smooth-vs-corner
  argument). Residual **`L-1`** disclosed and proved **non-empty for every non-degenerate
  calibration**, with the bound `min_k c_k ≥ S*/5`, and escalated to the Contract owner (§8.4).
  REV2 **does not claim GAP-11 fully discharged.**
- **Finding:** MAJ-1-R(a),(b).

### C-5 — HM bound labels fixed (FR-MIN-1)
- Corrected §10.1: `HM ≥ min` is the lower bound; `HM ≤ 5·min` is the upper bound. (Inequality itself
  was always correct.)

### C-6 — ε_c floor handled (FR-MIN-2)
- Bounds stated over the **floored** vector `v_k = max(c_k, ε_c)`; `HM ≤ 5·min_k c_k` does not hold
  against unfloored `c` when `c_k < ε_c`.
- Removed "unconditional" / "any gait with positive `S`" phrasings (§7.5, §10.5). Ranking claims
  restricted to capabilities strictly above the floor; `O(ε_c)` caveat stated.

### C-7 — Sufficient-condition derivation fixed (FR-MIN-3)
- §7.2 now derives `HM(c^U) ≤ 5/(1/x + 4)` from the **capability upper bound `c_k ≤ 1`** (not
  "others ≤ α_k"), giving `x < S*/(5−4S*)`. Premise wording corrected.

### C-8 — `α_k` marked illustrative-only (FR-MIN-4)
- §4.1.1: `α = (0.5,…,0.5)` flagged **illustrative only, not frozen**; all worked numbers marked
  illustrative pending calibration (§10.6).

### C-9 — Document identity unified (FR-MIN-5)
- Canonical status token `OPT2A_REV2_STATUS` declared; supersession banners added to REV0 and REV1;
  consolidated provenance (§11, Status/Provenance).

## Unchanged (carried from REV1)

- Capability sub-scores `c1…c5` and their derivations (REV1 §6).
- Soft in-band normalisation (REV1 §5.3).
- Fall/invalid ordering `valid ≥ 0 > fall(−1) > invalid(−2)`.
- Evaluation-protocol shape (`N_rep`, CRN, status precedence, provenance).
- Boundary of §11; no weights; no optimizer parameters.

---

## R2 — Post-GLM remediation round

Driver: `OPT2A_REV2_FOCUSED_REAUDIT_REPORT.md` (GLM-5.3 Independent Adversarial Re-Audit;
`AUDIT_DECISION = REJECTED`, `OPT-2B_ALLOWED = NO`). The audit **confirmed the technical repairs**
of `OPT2A-MAJ-1-R` sub-findings `(a₁, b, c, d)` as `CLOSED`, but kept **`GAP-11` = OPEN** and
**`SP-2` = VIOLATED** (root cause: the unruled `L-1` band), and raised **five new minors** `R2-MIN-1…5`.

This round is **argument-hardening + bookkeeping + an owner decision request**. It does **not**
change the fitness formula, the `HM` aggregate, the aggregation order, or the success definition.

| # | Finding | Fix | Location |
|---|---|---|---|
| R2-MIN-1 | §8.4 irreducibility argument assumed `C¹`; "any contract-compliant ranking" unscoped | **§8.4 rewritten** as a **continuity-only corner-contradiction theorem**; scope fixed to the **continuous scalar objective framework** and grounded in **OPT-1B L72**; the `C¹` premise and the phrase "any contract-compliant ranking" are explicitly **withdrawn**; discontinuity excluded by statement. §8.3.2 marked specific-to-`HM` and pointed to §8.4. Conclusion: GAP-11 pairwise dominance and GAP-04/05/06/09 strictness are **jointly unsatisfiable**. | §8.3.2, §8.4 |
| R2-MIN-2 | filename chain inconsistency (`OPT-2A_…` vs on-disk `OPT2A_…`) | **Filename alias note** added to the Header and Document-identity block: on-disk name canonical; hyphenated form names the same file. | Header, Document identity |
| R2-MIN-3 | L-1 quantification one-sided (floor only) | **New §8.6**: general vs **symmetric** calibration separated; for `α=(a,…,a)`, `sup_{R\A} HM = 5a/(1+4a)`, **maximal inversion `4a(1−a)/(1+4a)`** (= `1/3` at `a=0.5`); flagged **not a universal bound** — owner-ruling example only. | §8.6 |
| R2-MIN-4 | "OPT-1B/OPT-1A unmodified" only indirectly verifiable (untracked tree) | **Provenance note** added: the GLM-5.3 audit verified against the **working tree**; **OPT-1B/OPT-1A sha256 fingerprints recorded**; a **commit fingerprint is required before merge**. | Provenance |
| R2-MIN-5 | `CLOSES = OPT2A-MAJ-1-R + FR-MIN-1..5` overstated closure | Token **replaced**: `TECHNICAL_CLOSURE = OPT2A-MAJ-1-R sub-findings + FR-MIN-1..5 + R2-MIN-1..5`; `CONTRACT_BLOCKER = GAP-11/SP-2 owner ruling pending`; `GAP11_CLAIM = OPEN_ESCALATED`. Added `POST_AUDIT_REMEDIATION`/`CONTRACT_STATE`/`FORMULA_CHANGED` tokens. | Status block |

### R2 deliverables

- `OPT2A_FITNESS_V2_DESIGN_RFC_REV2.md` — revised (as above).
- `OPT2A_REV2_CHANGELOG.md` — this section.
- `OPT2A_REV2_SELF_AUDIT_REPORT.md` — R2 self-audit section added.
- `OPT2A_GAP11_OWNER_DECISION_REQUEST.md` — **new**; the contract ruling request
  (Problem / Evidence / Mathematical result / Options A/B/C / "OPT-2A cannot decide").
- `OPT2A_POST_GLM_REMEDIATION_REPORT.md` — **new**; the remediation report.

### R2 explicit non-changes

- Fitness formula, `HM` aggregate, aggregation order, success definition: **unchanged**.
- No code, pseudocode, runner, optimizer parameter, or weight added.
- OPT-1B / OPT-1A **not modified**. **OPT-2B not entered.**
- `GAP-11` / `SP-2` **not declared closed** — an owner ruling is still required.

End of CHANGELOG.
