#!/usr/bin/env bash
# Task 4 Phase 4D pause/resume semantics for the collector.
#
# Pauses Gazebo mid-collection, proves simulation time is frozen with the
# read-only /clock probe, resumes, and verifies the final dataset has strictly
# increasing simulation timestamps (no wall-time samples, no duplicates).
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"
EVID="${WS}/docs/task4_phase4d_evidence"
OUT="${EVID}/pause_resume.txt"
SUM="${EVID}/pause_run_summary.json"
DS="${EVID}/pause_run_dataset"
rm -rf "${DS}"; rm -f "${SUM}"
: > "${OUT}"

export GZ_PARTITION=t4p4d_pause ROS_DOMAIN_ID=144
set -m
rm -f /tmp/p4d_pause.pgid 2>/dev/null; setsid bash -c "echo \$\$ > /tmp/p4d_pause.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false \
  phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=true \
  phase4b_gait_method:=walk phase4b_duration_sec:=20.0 phase4b_record_duration:=0.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 phase4b_swing_height_m:=0.008 \
  phase4b_double_support_ratio:=0.3 phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS} phase4d_summary_path:=${SUM} \
  phase4d_duration_sec:=12.0 phase4d_overwrite:=true" > "${EVID}/launch_pause.log" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4d_pause.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4d_pause.pgid)

# let the collector accumulate real samples first
sleep 9
{
  echo "### Phase 4D pause/resume"
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
echo "sim_after_resume=$(python3 tools/phase4c_clock_probe.py 3)" >> "${OUT}"

for _ in $(seq 1 180); do
  [ -s "${SUM}" ] && break
  sleep 1
done
kill -INT -- "-${PGID}" 2>/dev/null; sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi

if [ -s "${SUM}" ]; then
  python3 tools/validate_task4_phase4d_dataset.py "${DS}" \
    --json-out "${EVID}/pause_resume_validation.json" >> "${OUT}" 2>&1
else
  echo "ERROR: pause summary missing" >> "${OUT}"
fi
RESIDUAL=99
for _ in $(seq 1 20); do
  RESIDUAL=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)
  [ "${RESIDUAL}" -eq 0 ] && break
  sleep 1
done
echo "pause residual=${RESIDUAL}" >> "${OUT}"
