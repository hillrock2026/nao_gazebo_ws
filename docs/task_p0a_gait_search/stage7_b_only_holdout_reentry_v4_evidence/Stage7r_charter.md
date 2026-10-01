# Stage 7' Charter — B-only holdout reentry (pre-registered, freeze-once)

- Task: `DeepSeek_Stage7_stop_erratum_and_reentry_repair_prompt.md`
- Workspace: `/home/liu/workspace/Nao_Program/nao_gazebo_ws`
- Evidence dir: `docs/task_p0a_gait_search/stage7_b_only_holdout_reentry_evidence/`
- Author: opencode / DeepSeek
- Status at freeze: `STAGE_7_FAILURE_ARCHIVED__STAGE_7_REENTRY_READY_PENDING_AUTHORIZATION`
- Execution authority: **NOT GRANTED**.

This charter is written and hash-locked **before any Stage 7' episode is started**.
If any file in this charter/gate/preregistration set changes after the first Stage 7'
episode, all Stage 7' runs are void and a new version must be created from zero.

```text
Gate 2A：未通过   P0-a：未解决   P0-b：未解决
```

## 1. Historical status (unchanged)

```text
historical_stage7_status: STAGE_7_FAIL_STOPPED
authoritative_valid_episode_count: 1
valid_seed: 909001
909002-909005: NOT_RUN
failed_gates: lateral_ratio=0.5502>0.50, yaw_change_deg=31.7637>30.0
holdout_confirmation: NOT_ESTABLISHED
stage7_claim_level: single_seed_observation
```

Attempt classifications are fixed by the Stage 7 erratum: attempt 1 INFRA_INVALID,
attempt 2 fatal_integrity_failure (18/18 PASS **not** adopted), attempt 3
valid_protocol_episode (authoritative FAIL). The Stage 7 authorization gap
(attempt 3 had no independent authorization entry) is recorded and must not be
repeated.

## 2. Stage 7' purpose

> Can `explicit_support_swing` B satisfy the P1-R v3 **provisional** engineering
> contract on brand-new holdout physics seeds?

B-only. Not an A/B equivalence, significance, population, P0-a, final-tolerance,
P1-closure or P2 claim.

## 3. Sample size (frozen before any run)

```text
seed_count:        5
episodes_per_seed: 1
episode_count:     5
execution:         strict serial
claim_level:       initial_holdout_confirmation
```

The 5-vs-3 decision is made before runs; it may never be changed after results.
If 5 are preregistered but fewer complete, the stage is INCOMPLETE/FAIL_STOPPED;
completed episodes may not be repackaged as 5.

## 4. Holdout seeds (fresh, published before runs)

```text
909101, 909102, 909103, 909104, 909105   (fixed serial order)
```

Excluded: all P1-BD / old P1 / Phase 2 / Phase 3 / Stage 6 A / Stage 6 B / Stage 6
reentry seeds, the Stage 7 seeds `909001-909005`, and
`FORBIDDEN_SEEDS={10101,30303,40404,50505,606001..606005,707001..707005,808001,809001}`.
Whole-repo semantic freshness scan must show zero semantic hits per seed.
Seeds may not be replaced, re-drawn, repeated or dropped.

## 5. Frozen configuration

```text
config_source:     docs/task_p0a_gait_search/p1bd_evidence/P1BD_config_snapshot.json
gait_phase_mode:   explicit_support_swing
step_frequency_hz: 0.8
sway_sign:         -1
```

All other fields match the reviewed B snapshot. Missing critical fields fail closed.
Forbidden: running A; using Stage 6 A config; `step_frequency_hz=0.5`; default
`sway_sign=+1`; absent `sway_sign`/`step_frequency_hz` in config or launch command.

## 6. Gate policy (`KEEP_PROVISIONAL`)

```text
executable_gate_file:  docs/task_p0a_gait_search/p1r_support_swing/frozen_gates_v2.json
sha256:                bf6e75e7d5f479b2aaba6143a2b3370a602bf643f4f5df401ab47e81f94d38c5
n_hard_gates:          18 (excludes fk_landing_valid)
v3_gate_frozen:        false
strict_v3_compliance:  UNVERIFIED
fk_landing_valid:      UNVERIFIED
```

Required wording: "satisfies the P1-R v3 provisional engineering contract under the
frozen executable gate subset, with fk_landing_valid=UNVERIFIED". Forbidden:
`strict v3 compliant`, `fully v3 validated`, `final v3 pass`.

Diagnostics (reported, not hard verdicts): `zero_contact_max_sec`,
`m0_legacy_contact_delay_max_sec`, `swing_to_support_onset_delay_sec`,
Bool contact rates. Hard exceptions: contact trace invalid => `measurable=false`;
onset `n_missing>0` => hard FAIL.

## 7. Attempt taxonomy and recovery policy (resolves the Stage 7 contradiction)

Attempt types: `initial_episode`, `environment_failure`, `integrity_failure`,
`recovery_after_environment_failure`, `recovery_after_integrity_failure`,
`repeat_same_seed`, `valid_protocol_episode`, `hard_gate_failure`.

```text
hard_gate_failure:
  stop_stage: true
  recovery: false
  same_seed_repeat: false
  seed_replacement: false

environment_failure:
  stop_stage: true
  recovery: only with a pre-issued, single-consumption per-attempt authorization
  max_recovery_per_seed: 1
  automatic: false

fatal_integrity_failure:
  stop_stage: true
  recovery: only with a pre-issued, single-consumption per-attempt authorization
  max_recovery_per_seed: 1
  automatic: false

protocol_violation:
  stop_stage: true
  recovery: false
```

Explicit resolution: an integrity/environment failure always stops the stage; a
same-seed rerun is a **recovery**, permitted only with an authorization that exists
**before** the attempt starts and is consumed exactly once. It is a retry, but never
automatic or implicit. `hard_gate_failure` is terminal: no recovery.

## 8. Per-attempt authorization (repaired mechanism)

- every attempt must consume exactly one unconsumed entry in
  `Stage7r_authorization_ledger.json`, matching `(attempt_id, seed)`;
- an entry has `authorization_id`, `attempt_id`, `seed`, `reason`,
  `same_seed_repeat_allowed`, `wrapper_change_allowed`, `issued_at`, `consumed_at`,
  `status`; status is ISSUED -> CONSUMED (single use);
- no top-level `GRANTED` flag is sufficient; the executor and orchestrator fail
  closed (`BLOCKED_AUTHORIZATION`) without a matching unconsumed entry;
- the ledger starts empty and no execution may start before the user issues
  per-attempt authorizations.

## 9. Strict serial execution

```text
PRECHECK -> FREEZE -> (per attempt: AUTHORIZE -> RUN -> AUDIT) -> FINALIZE
```

Audit-before-next. Any fatal condition stops immediately. No automatic retry, no
seed replacement, no skipping a failed seed.

## 10. PASS condition (5-seed protocol)

5/5 hard-gate PASS; no fall; no fatal integrity failure; no protocol violation; no
automatic retry; no seed replacement; exactly 1 episode per seed; frozen order; raw
traces verifiable; source/asset/environment hashes traceable; PGID + residual checks
per episode; all gates itemized; no overwrite/deletion of failure evidence;
historical assets unchanged. A 3-seed PASS is only `initial holdout confirmation`.

## 11. Evidence per seed

`runs/seed-<SEED>/`: `config_snapshot.json`, `launch_cmd.txt`, `run_config.json`,
`provenance.json`, `run_result.json`, `launch.log`, `stdout.log`, `stderr.log`,
`trace/`, `metrics.json`, `gates.json`, `failure_reason.json`, `stop_reason.json`,
`pgid.json`, `residual_process.json`, `SHA256SUMS`. `run_result.launch_cmd` must not
be null. Both `run_dir_rel` and `run_dir_abs` (= realpath) are recorded.

## 12. Claim ceiling

> explicit B satisfies the P1-R v3 provisional engineering contract on 5 brand-new
> holdout episodes.

B-only; not A/B equivalence; not statistical; not population; not P0-a; not final
tolerance; `fk_landing_valid=UNVERIFIED`; independent review still required; P1
closure and P2 authorization are separate decisions.

## 13. Authorization boundary

Preparation may be performed without execution authorization. Starting any
Gazebo/ROS/controller/episode requires explicit per-attempt authorization recorded
in the ledger. Until then:

```text
new_stage7_reentry_status: READY_PENDING_AUTHORIZATION
execution_authorization:   NOT_GRANTED
new_episode_started:       false
```

## 14. Forbidden actions

run A; continue Stage 7; rerun 909001; run 909002-909005; adopt attempt 2's 18/18;
delete/overwrite any attempt; edit frozen gates/policy after results; claim strict v3;
write UNVERIFIED as PASS; present 3 as 5; present B-only as A/B equivalence; present
Stage 7' as P1 final validation or P0-a completion; close P1 or authorize P2 before
independent review; start any episode without authorization; silently update scripts
after freeze.
