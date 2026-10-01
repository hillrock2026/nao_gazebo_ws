#!/usr/bin/env bash
# Task 4 Phase 4D runtime negative tests.
#
#   A. no producer                -> 0 samples, command_not_available > 0
#   B. producer stops (stale cmd) -> bounded hold then rejects, no stale samples
#   C. default launch             -> no collector / no /joint_command
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"
EVID="${WS}/docs/task4_phase4d_evidence"
mkdir -p "${EVID}"
OUT="${EVID}/negative_tests.txt"
: > "${OUT}"

wait_pgid () {
  local f="$1"
  for _ in $(seq 1 30); do [ -s "$f" ] && break; sleep 0.2; done
  cat "$f"
}

wait_summary () {
  for _ in $(seq 1 120); do
    [ -s "$1" ] && break
    sleep 1
  done
}

stop_group () {
  local pgid="$1"
  kill -INT -- "-${pgid}" 2>/dev/null || true
  sleep 6
  kill -TERM -- "-${pgid}" 2>/dev/null || true
  sleep 4
  local n
  for _ in $(seq 1 25); do
    n=$(ps -eo cmd | grep -E \
      "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge" \
      | grep -v grep | wc -l)
    [ "$n" -eq 0 ] && break
    sleep 1
  done
  echo "stop_group(${pgid}) residual=${n:-?}"
}

echo "### A. no producer (collector only)" | tee -a "${OUT}"
export GZ_PARTITION=t4p4d_noprod ROS_DOMAIN_ID=141
SUM_A="${EVID}/negative_no_producer_summary.json"
DS_A="${EVID}/negative_no_producer_dataset"
rm -rf "${DS_A}"; rm -f "${SUM_A}"
set -m
rm -f /tmp/p4d_noprod.pgid
setsid bash -c "echo \$\$ > /tmp/p4d_noprod.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS_A} phase4d_summary_path:=${SUM_A} \
  phase4d_duration_sec:=6.0 phase4d_overwrite:=true" > "${EVID}/launch_no_producer.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4d_noprod.pgid)
wait_summary "${SUM_A}"
stop_group "${PGID}"
python3 - "${SUM_A}" "${OUT}" <<'PY' 2>&1 | tee -a "${OUT}"
import json, sys
s = json.load(open(sys.argv[1]))
reasons = s.get('reject_counts_by_reason', {})
ok = s.get('samples_written') == 0 and reasons.get('command_not_available', 0) > 0
print('no_producer samples_written:', s.get('samples_written'))
print('no_producer command_not_available:', reasons.get('command_not_available', 0))
print('no_producer PASS:', ok)
PY

echo | tee -a "${OUT}"
echo "### B. producer stops -> stale command rejection" | tee -a "${OUT}"
export GZ_PARTITION=t4p4d_stale ROS_DOMAIN_ID=142
SUM_B="${EVID}/negative_stale_command_summary.json"
DS_B="${EVID}/negative_stale_command_dataset"
rm -rf "${DS_B}"; rm -f "${SUM_B}"
rm -f /tmp/p4d_stale.pgid
setsid bash -c "echo \$\$ > /tmp/p4d_stale.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false \
  phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=true \
  phase4b_gait_method:=walk phase4b_duration_sec:=6.0 phase4b_record_duration:=0.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 phase4b_swing_height_m:=0.008 \
  phase4b_double_support_ratio:=0.3 phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS_B} phase4d_summary_path:=${SUM_B} \
  phase4d_duration_sec:=13.0 phase4d_overwrite:=true" > "${EVID}/launch_stale_command.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4d_stale.pgid)
wait_summary "${SUM_B}"
stop_group "${PGID}"
python3 - "${SUM_B}" "${OUT}" <<'PY' 2>&1 | tee -a "${OUT}"
import json, sys
s = json.load(open(sys.argv[1]))
reasons = s.get('reject_counts_by_reason', {})
stale = reasons.get('command_stale', 0)
ok = s.get('samples_written', 0) > 0 and stale > 0
print('stale samples_written:', s.get('samples_written'))
print('stale command_stale rejects:', stale)
print('stale PASS:', ok)
PY

echo | tee -a "${OUT}"
echo "### C. default launch has no collector" | tee -a "${OUT}"
export GZ_PARTITION=t4p4d_def ROS_DOMAIN_ID=143
rm -f /tmp/p4d_def.pgid
setsid bash -c "echo \$\$ > /tmp/p4d_def.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true" > "${EVID}/launch_default.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4d_def.pgid)
sleep 16
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "phase4d_collector nodes : $(ros2 node list 2>/dev/null | grep -c phase4d_collector)"
  echo "phase4b_producer nodes  : $(ros2 node list 2>/dev/null | grep -c phase4b_producer)"
  echo "/joint_command in list  : $(ros2 topic list 2>/dev/null | grep -c '^/joint_command$')"
} | tee -a "${OUT}"
stop_group "${PGID}"
echo "default residual=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)" | tee -a "${OUT}"
