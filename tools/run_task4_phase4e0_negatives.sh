#!/usr/bin/env bash
# Task 4 Phase 4E-0 runtime negative regressions (schema v1.0.1).
#
#   A. no producer                 -> 0 samples, command_not_available > 0
#   B. producer stops (stale cmd)  -> bounded hold then rejects
#   C. contact converter disabled  -> 0 samples, contact_not_received > 0
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"
EVID="${WS}/docs/task4_phase4e0_evidence"
VALIDATOR="${WS}/tools/validate_task4_phase4d_dataset.py"
mkdir -p "${EVID}"
: > "${EVID}/negative_no_producer.txt"
: > "${EVID}/negative_stale_command.txt"
: > "${EVID}/negative_contact_missing.txt"

wait_pgid () {
  for _ in $(seq 1 30); do [ -s "$1" ] && break; sleep 0.2; done
  cat "$1"
}
wait_summary () {
  for _ in $(seq 1 160); do
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
}

echo "### A. no producer (collector only)" | tee -a "${EVID}/negative_no_producer.txt"
export GZ_PARTITION=t4p4e0_noprod ROS_DOMAIN_ID=151
SUM_A="${EVID}/negative_no_producer_summary.json"
DS_A="${EVID}/negative_no_producer_dataset"
rm -rf "${DS_A}"; rm -f "${SUM_A}"
set -m
rm -f /tmp/p4e0_noprod.pgid
setsid bash -c "echo \$\$ > /tmp/p4e0_noprod.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS_A} phase4d_summary_path:=${SUM_A} \
  phase4d_validator_script:=${VALIDATOR} \
  phase4d_duration_sec:=6.0 phase4d_overwrite:=true" > "${EVID}/launch_no_producer.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4e0_noprod.pgid)
wait_summary "${SUM_A}"
stop_group "${PGID}"
python3 - "${SUM_A}" "${EVID}/negative_no_producer.txt" <<'PY' 2>&1 | tee -a "${EVID}/negative_no_producer.txt"
import json, sys
s = json.load(open(sys.argv[1]))
r = s.get('reject_counts_by_reason', {})
ok = (s.get('samples_written') == 0 and r.get('command_not_available', 0) > 0
      and s.get('unaccounted') == 0 and s.get('pending_current') == 0)
print('samples_written:', s.get('samples_written'))
print('has_samples:', s.get('has_samples'))
print('command_not_available:', r.get('command_not_available'))
print('accounted_total/unaccounted:', s.get('accounted_total'), s.get('unaccounted'))
print('RESULT:', 'PASS' if ok else 'FAIL')
PY

echo | tee -a "${EVID}/negative_stale_command.txt"
echo "### B. producer stops -> stale command rejection" | tee -a "${EVID}/negative_stale_command.txt"
export GZ_PARTITION=t4p4e0_stale ROS_DOMAIN_ID=152
SUM_B="${EVID}/negative_stale_command_summary.json"
DS_B="${EVID}/negative_stale_command_dataset"
rm -rf "${DS_B}"; rm -f "${SUM_B}"
rm -f /tmp/p4e0_stale.pgid
setsid bash -c "echo \$\$ > /tmp/p4e0_stale.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false \
  phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=true \
  phase4b_gait_method:=walk phase4b_duration_sec:=6.0 phase4b_record_duration:=0.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 phase4b_swing_height_m:=0.008 \
  phase4b_double_support_ratio:=0.3 phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS_B} phase4d_summary_path:=${SUM_B} \
  phase4d_validator_script:=${VALIDATOR} \
  phase4d_duration_sec:=13.0 phase4d_overwrite:=true" > "${EVID}/launch_stale_command.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4e0_stale.pgid)
wait_summary "${SUM_B}"
stop_group "${PGID}"
python3 - "${SUM_B}" "${EVID}/negative_stale_command.txt" <<'PY' 2>&1 | tee -a "${EVID}/negative_stale_command.txt"
import json, sys
s = json.load(open(sys.argv[1]))
r = s.get('reject_counts_by_reason', {})
ok = (s.get('samples_written', 0) > 0 and r.get('command_stale', 0) > 0
      and s.get('unaccounted') == 0 and s.get('pending_current') == 0)
print('samples_written:', s.get('samples_written'))
print('command_stale:', r.get('command_stale'))
print('accounted_total/unaccounted:', s.get('accounted_total'), s.get('unaccounted'))
print('RESULT:', 'PASS' if ok else 'FAIL')
PY

echo | tee -a "${EVID}/negative_contact_missing.txt"
echo "### C. contact converter disabled at runtime" | tee -a "${EVID}/negative_contact_missing.txt"
export GZ_PARTITION=t4p4e0_nocontact ROS_DOMAIN_ID=153
SUM_C="${EVID}/negative_contact_missing_summary.json"
DS_C="${EVID}/negative_contact_missing_dataset"
rm -rf "${DS_C}"; rm -f "${SUM_C}"
rm -f /tmp/p4e0_nocontact.pgid
setsid bash -c "echo \$\$ > /tmp/p4e0_nocontact.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_contact_converter:=false \
  enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
  enable_phase4b:=true phase4b_run_gait:=true phase4b_gait_method:=walk \
  phase4b_duration_sec:=8.0 phase4b_record_duration:=0.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 phase4b_swing_height_m:=0.008 \
  phase4b_double_support_ratio:=0.3 phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS_C} phase4d_summary_path:=${SUM_C} \
  phase4d_validator_script:=${VALIDATOR} \
  phase4d_duration_sec:=9.0 phase4d_overwrite:=true" > "${EVID}/launch_contact_missing.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4e0_nocontact.pgid)
sleep 6
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "left_foot_contact publishers: $(ros2 topic info /left_foot/contact 2>/dev/null | grep -c 'Publisher count: [0-9]')"
  echo "left_foot_contact publisher_count_line: $(ros2 topic info /left_foot/contact 2>/dev/null | grep 'Publisher count')"
} | tee -a "${EVID}/negative_contact_missing.txt"
wait_summary "${SUM_C}"
stop_group "${PGID}"
python3 - "${SUM_C}" "${EVID}/negative_contact_missing.txt" <<'PY' 2>&1 | tee -a "${EVID}/negative_contact_missing.txt"
import json, sys
s = json.load(open(sys.argv[1]))
r = s.get('reject_counts_by_reason', {})
ok = (s.get('samples_written') == 0 and r.get('contact_not_received', 0) > 0
      and s.get('unaccounted') == 0 and s.get('pending_current') == 0)
print('samples_written:', s.get('samples_written'))
print('contact_not_received:', r.get('contact_not_received'))
print('accounted_total/unaccounted:', s.get('accounted_total'), s.get('unaccounted'))
print('RESULT:', 'PASS' if ok else 'FAIL')
PY
