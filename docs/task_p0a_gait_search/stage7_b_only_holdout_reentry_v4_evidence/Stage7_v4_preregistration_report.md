# Stage 7 v4 Preregistration Report

- Prompt: `docs/DeepSeek_Stage7_v4_preregister_and_execute_prompt.md` (Phase 4)
- Prerequisite review: `docs/glm53_stage7_v4_fingerprint_binding_repair_independent_review_report.md`
  → `REVIEW_ACCEPT_STAGE7_V4_READY_FOR_PREREGISTRATION`
- Author: opencode / DeepSeek
- Date: 2026-09-29
- Mode: **B_ONLY**, batch `stage7r_batch_v3`… (new v4 batch)
- Terminal: **`READY_PENDING_GLM53_PREREGISTRATION_REVIEW`**
- Discipline: offline only — no Gazebo/ROS/controller/phase-producer/episode started;
  no authorization issued; `runs/` empty. Bootstrap probes only (`ros2 --help` /
  `pkg prefix` / `launch --show-args`, `gz sim --version`, `python3 -c import rclpy`).

## 1. Entry / environment verification

```text
v4 preregistration: NOT_CREATED (before this phase) -> created now
v4 runs/: empty
v4 authorization ledger: created, authorizations = [] (empty)
historical SHA256SUMS: all pass (see §6)
residual Gazebo/ROS processes: 0
bootstrap: /opt/ros/jazzy/setup.bash + <ws>/install/setup.bash
fingerprint reproduces reviewed value:
  version: stage7r_env/2
  hash:    1c45b6400b057c5420a3e3418b654ad3a040fb7b5dfcf642d1a43eca1cc06fff
```

## 2. Seed allocation (freshness)

```text
seeds: [909401, 909402, 909403, 909404, 909405]   (frozen order)
seed_count: 5
freshness scan: 3852 files, semantic hits = 0 per seed
forbidden/historical: incl. 909001-909005, 909101-909105, 909201-909205, 909301-909305
seed_allocation_sha256: 96174bf1433d16050ecf605099152521920167ea3c0b1c9fc539a2666d0049db
freeze_status: FROZEN
```

## 3. Frozen configuration

```text
mode: B_ONLY
gait_phase_mode: explicit_support_swing
step_frequency_hz: 0.8
sway_sign: -1
automatic_retry: false
seed_replacement_allowed: false
canonical diff: only gait_phase_mode
gate policy: frozen_gates_v2.json (18 executable hard gates; excludes fk_landing_valid)
v3_gate_frozen: false | strict_v3_compliance: UNVERIFIED | fk_landing_valid: UNVERIFIED
```

## 4. Artifact hashes (SHA-256)

```text
Stage7r_preregistration.json   b43cc2d0d47f8f4f7a7325390962f5c41c1806bd63f52eff51bfb370127d8b28
Stage7r_run_order.json         bfb792d3957ebcdad9f9de04451b6c621fb1134105eb20239db84106ae55e801
Stage7r_seed_allocation.json   b1e8ba1e838643226e933417ee9adb50b6ef71ea819452d30f26e9c111c8e1e3
Stage7r_B_config_snapshot.json 70df1803be99aaba1c9084591eec5bae813b25a793ef293efac60c8ef48c43d3
Stage7r_config_diff.json       bb0862c6c3eb7546a309e2cb16d9da09dc7ffa9e9c6c4bcf5d9399495221a270
Stage7r_gate_policy.json       6ea032c2656eb7af2a7524081d91316e1b345366a90991438241f52e254c0a11
Stage7r_attempt_policy.json    7dab22699454328c1a02e3a515a0f21c801211a9d9631c68abc350194f5d2c46
Stage7r_source_manifest.json   9f5633ce0a623bcc720982d774dfe97a574cd1748ce0e2144bb14605803ec3a4
Stage7r_asset_manifest.json    68ee2404b37409eb0ae9780d5ac435a80de56707c0aff552f42c29b4e8def93d
Stage7r_environment_manifest.json 6a18b5f54f754647bae94264c8e2cb801f717851532ef5a49218b878b6828103
Stage7r_authorization_ledger.json 53fc8fcef6f67b379cba957e0dd3ee302a04970f4fd78b9dfb58ddd73857e37d
Stage7r_precheck.json          d27654a11c58c4387c8db848a4feb3eb005d8fc7a96c3da355d44d8dabc47449
SHA256SUMS                     (regenerated at freeze; output manifest excludes itself + Stage7r_output_manifest.json)
```

Preregistration embeds the environment binding:

```text
environment_fingerprint_version: stage7r_env/2
environment_fingerprint_hash:    1c45b6400b057c5420a3e3418b654ad3a040fb7b5dfcf642d1a43eca1cc06fff
environment_fingerprint_keys:    11 required + python_version (12 total)
source_manifest:                 Stage7r_source_manifest.json
                                 sha256 9f5633ce… (created before prereg freeze)
```

Source manifest covers 13 execution scripts (incl. `stage7r_env.py`,
`stage7r_execd.py`, `stage7r_orchestrator.py`), 13 canonical sources, the legacy
wrapper and `tools/run_p0a_gait_candidate.py` (`6e4d9274…`).

## 5. PRECHECK

```text
Stage7r_precheck.json: 32/32 PASS, terminal=PRECHECK_PASS_PENDING_AUTHORIZATION
```

Includes: dir isolation, seed freshness, B config, canonical diff, gate hash,
source-manifest binding (hash embedded + `created_at` before freeze), script hash
lock, orchestrator filename binding (31), live environment-fingerprint binding (32),
taxonomy/protocol-violation, strict reason match, no-retry/no-seed-replacement, v3
provisional unchanged, empty `runs/`, empty ledger, no residual processes, and
historical hash integrity (Stage 0, Stage 7, Stage 6, v2, v3, old Stage 7').

## 6. Historical hash verification (unchanged)

```text
Stage 0 qualification: 32/32 OK
v3: 42/42 OK   v2: 78/78 OK   old Stage 7': 56/56 OK
Stage 7 confirmation: 90/90 OK
Stage 6 screening: 36/36 OK   Stage 6 reentry: 17/17 OK
tools/run_p0a_gait_candidate.py: 6e4d9274…
```

## 7. Unexecuted boundary

```text
execution_authorization: NOT_GRANTED
execution_status:        NOT_STARTED
executed_seeds:          []
runs/:                   empty
ledger:                  []
new episode:             NOT_STARTED
holdout_confirmation:    NOT_ESTABLISHED
P1 closure / P2:         NOT_AUTHORIZED
```

Final stop:

```text
READY_PENDING_GLM53_PREREGISTRATION_REVIEW
```

Next (gated): submit this bundle to GLM-5.3 for preregistration review; only on
`REVIEW_ACCEPT_STAGE7_V4_PREREGISTRATION_READY_FOR_AUTHORIZATION` may Phase 5
proceed — and only after a new explicit **user** authorization (GLM's ACCEPT is not
user authorization). No episode may start in this phase.

End of report.
