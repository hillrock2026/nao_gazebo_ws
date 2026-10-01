# Stage 7' Charter — Amendment F1–F3 (append-only)

- Task: `DeepSeek_Stage7_reentry_F1_F3_fix_prompt.md`
- Source review: `docs/glm53_stage7_failure_archive_reentry_independent_review_report.md`
- Date: 2026-09-28
- Applies to: `Stage7r_charter.md` (unchanged; this amendment adds/clarifies)
- Supersedes: none; amends by addition.
- Status: `STAGE_7_REENTRY_READY_PENDING_AUTHORIZATION`; execution authorization
  `NOT_GRANTED`.

> No Gazebo/ROS/episode executes under this amendment. This document only makes the
> frozen policy explicit and consistent with the repaired code.

## A1. `protocol_violation` attempt type (F1)

The formal taxonomy is now 9 types:

```text
initial_episode
environment_failure
integrity_failure
recovery_after_environment_failure
recovery_after_integrity_failure
repeat_same_seed
valid_protocol_episode
hard_gate_failure
protocol_violation
```

Semantics (frozen):

```text
protocol_violation:
  counts_as_episode: false
  recovery_allowed: false
  stop_stage: true
  automatic_retry: false
  seed_replacement: false
```

Behaviors that are protocol violations: unauthorized attempt; unauthorized
same-seed repeat; unauthorized recovery; seed replacement; running A; continuing
after hard-gate failure; out-of-order seed; modifying frozen gates; modifying the
diagnostic policy; silent script change after freeze; script not in the source
manifest; config/source hash mismatch; forged or missing provenance; executing more
episodes than preregistered.

An unauthorized attempt is classified as `protocol_violation`
(`unauthorized_attempt`) and stops the orchestrator with
`status=STOPPED_PROTOCOL_VIOLATION`; it is never an ordinary initial episode.

## A2. Strict reason matching (F3)

```text
requested_reason == authorization.reason   (exact equality; no fallback)
initial_episode requires its own per-attempt authorization with
  authorization.reason = initial_episode
same_seed_repeat_allowed must be true for any recovery_*/repeat_same_seed reason
wrapper_change_allowed must be true if the attempt changes the wrapper
```

Method A is adopted: **initial episodes also require explicit per-attempt
authorization**. There is no free initial-episode path. A recovery authorization can
never be consumed by a request whose reason is `initial_episode`, and an
initial-episode authorization can never be borrowed for a recovery.

## A3. Source-manifest freeze binding (F2)

The operative preregistration (`Stage7r_preregistration_v2.json`) embeds:

```text
source_manifest.path       = Stage7r_source_manifest_v2.json
source_manifest.sha256     = <sha256 of that file>
source_manifest.created_at = <timestamp, strictly before preregistration freeze>
source_manifest.freeze_status = FROZEN
```

The source manifest covers every execution script, and the reference chain is
closed: preregistration -> source manifest -> every execution script hash. The
manifest is created before the preregistration freeze (proven by
`created_at_epoch`).

## A4. Non-contradiction with earlier charters

- `Stage7r_charter.md` §7 already listed `protocol_violation: stop_stage=true /
  recovery=false`; A1 makes the code enum match it.
- The earlier §7 recovery rule (integrity/environment recovery only with a
  pre-issued, single-consumption authorization) is unchanged; A2 removes only the
  loose `initial_episode` reason fallback and requires an explicit initial-episode
  authorization.
- No frozen gate threshold, diagnostic policy, seed, or config is changed.
