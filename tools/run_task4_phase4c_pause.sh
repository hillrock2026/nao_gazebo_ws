#!/usr/bin/env bash
# Task 4 Phase 4C pause/resume semantics check.
#
# Runs one full gait episode, pauses Gazebo for several wall seconds mid-run and
# resumes it. The read-only trace records simulation time only, so a frozen
# /clock must produce NO new commands and NO simulation-time gap; after resume
# the command stamps must continue strictly increasing with no duplicate or
# backward stamp and no fabricated future time.
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

EVID="${WS}/docs/task4_phase4c_evidence"
RUN_DIR="${EVID}/pause_run"
LOG="${EVID}/launch_pause.log"
OUT="${EVID}/pause_check.txt"
DURATION=22.0
RECORD=28.0
mkdir -p "${RUN_DIR}"
rm -f "${RUN_DIR}"/*.jsonl "${RUN_DIR}/phase4b_summary.json" "${LOG}" "${OUT}"
python3 - "${RUN_DIR}" <<'PY'
import json, sys
d = sys.argv[1]
json.dump({"gait_method": "walk", "start_delay_sec": 0.5, "gait_blend_sec": 1.5,
           "duration_sec": 22.0, "stop_ramp_sec": 1.5, "hold_final_sec": 2.0,
           "stand_sec": 24.0, "step_frequency_hz": 0.5, "command_hold_max": 0.05,
           "supplementary_clearance_m": 0.004, "command_rate_hz": 100.0},
          open(d + "/run_config.json", "w"), indent=2)
PY

export GZ_PARTITION=t4p4c_pause ROS_DOMAIN_ID=132
set -m
setsid bash -c "echo \$\$ > /tmp/p4c_pause.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
  enable_phase4b:=true phase4b_run_gait:=true phase4b_gait_method:=walk \
  phase4b_duration_sec:=${DURATION} phase4b_record_duration:=${RECORD} \
  phase4b_stand_sec:=24.0 phase4b_hold_final_sec:=2.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 phase4b_swing_height_m:=0.008 \
  phase4b_double_support_ratio:=0.3 phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  phase4b_evidence_dir:=${RUN_DIR}" > "${LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4c_pause.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4c_pause.pgid)

# Wait until the gait has been running for a while (many commands recorded).
for _ in $(seq 1 120); do
  if [ -f "${RUN_DIR}/command_trace.jsonl" ] && \
     [ "$(wc -l < "${RUN_DIR}/command_trace.jsonl")" -gt 900 ]; then
    break
  fi
  sleep 1
done

# Simulation-time freeze evidence: the producer is driven by /clock, so a paused
# world must not advance simulation time. Probe the actual /clock timestamp via
# a read-only ROS subscriber (no fragile text parsing) at three wall moments.
{
  echo "### pause/resume run (Phase 4C)"
  echo "wall_before_pause=$(date +%s.%N)"
  echo "sim_before_pause=$(python3 tools/phase4c_clock_probe.py 3)"
  echo "\$ gz service ... pause: true"
} >> "${OUT}"
gz service -s /world/nao_world/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true' >> "${OUT}" 2>&1
{
  echo "wall_during_pause=$(date +%s.%N)"
  echo "sim_during_pause_probe1=$(python3 tools/phase4c_clock_probe.py 3)"
  sleep 3
  echo "sim_during_pause_probe2=$(python3 tools/phase4c_clock_probe.py 3)"
} >> "${OUT}"
gz service -s /world/nao_world/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >> "${OUT}" 2>&1
sleep 2
{
  echo "wall_after_resume=$(date +%s.%N)"
  echo "sim_after_resume=$(python3 tools/phase4c_clock_probe.py 3)"
} >> "${OUT}"

for _ in $(seq 1 240); do
  [ -s "${RUN_DIR}/phase4b_summary.json" ] && break
  sleep 1
done

python3 - "${RUN_DIR}" "${OUT}" <<'PY' >> "${OUT}" 2>&1
import json, sys, os
d, out = sys.argv[1], sys.argv[2]
cmds = [json.loads(l) for l in open(os.path.join(d, 'command_trace.jsonl')) if l.strip()]
acc = [c['t'] for c in cmds if c.get('ok')]
gaps = [(round(b - a, 4), a, b) for a, b in zip(acc, acc[1:]) if b - a > 0.05]
print('command_accepted:', len(acc))
print('stamps_strictly_increasing:', all(b > a for a, b in zip(acc, acc[1:])))
print('duplicate_stamps:', len(acc) != len(set(acc)))
print('sim_time_gaps_gt_0.05s:', gaps[:10])
print('max_command_gap_sec: %.4f' % max((b - a for a, b in zip(acc, acc[1:])), default=0.0))
PY

kill -INT -- "-${PGID}" 2>/dev/null; sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi
{
  echo "pause_run residual=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)"
} >> "${OUT}"
echo "pause run complete"
