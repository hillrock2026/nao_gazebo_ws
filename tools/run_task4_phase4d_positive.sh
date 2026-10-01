#!/usr/bin/env bash
# Task 4 Phase 4D positive closed-loop collection.
#
# Runs the already-approved Phase 4B gait producer with the actuator and the
# read-only Phase 4D collector, collects a short real dataset, then validates it
# offline. The final dataset directory contains exactly manifest.json and
# samples.jsonl; all other evidence goes to docs/task4_phase4d_evidence/.
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

EVID="${WS}/docs/task4_phase4d_evidence"
OUT="${EVID}/positive_collection"
LOG="${EVID}/launch_positive.log"
SUMMARY="${EVID}/collector_summary.json"
DURATION="${PHASE4D_DURATION:-12.0}"
mkdir -p "${EVID}"
rm -rf "${OUT}"
rm -f "${SUMMARY}" "${LOG}"
: > "${EVID}/stability_check.txt"

export GZ_PARTITION=t4p4d_pos ROS_DOMAIN_ID=140
set -m
rm -f /tmp/p4d_pos.pgid 2>/dev/null; setsid bash -c "echo \$\$ > /tmp/p4d_pos.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true \
  enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
  enable_phase4b:=true phase4b_run_gait:=true phase4b_gait_method:=walk \
  phase4b_duration_sec:=22.0 phase4b_record_duration:=0.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 \
  phase4b_swing_height_m:=0.008 phase4b_double_support_ratio:=0.3 \
  phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${OUT} phase4d_summary_path:=${SUMMARY} \
  phase4d_duration_sec:=${DURATION} phase4d_overwrite:=true" > "${LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4d_pos.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4d_pos.pgid)

# Capture the live runtime topology while the producer is publishing.
sleep 8
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "### Phase 4D positive runtime topology"
  echo "\$ ros2 node list"
  ros2 node list
  echo
  echo "\$ ros2 topic info /joint_command --verbose"
  ros2 topic info /joint_command --verbose
  echo
  echo "\$ ros2 topic info /joint_states --verbose"
  ros2 topic info /joint_states --verbose
} > "${EVID}/runtime_topology.txt" 2>&1

for _ in $(seq 1 180); do
  [ -s "${SUMMARY}" ] && break
  sleep 1
done

kill -INT -- "-${PGID}" 2>/dev/null
sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi
for _ in $(seq 1 20); do
  RESIDUAL=$(ps -eo cmd | grep -E \
    "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot" \
    | grep -v grep | wc -l)
  [ "${RESIDUAL}" -eq 0 ] && break
  sleep 1
done
echo "positive residual=${RESIDUAL}" | tee -a "${EVID}/stability_check.txt"

if [ -s "${SUMMARY}" ]; then
  python3 - "${SUMMARY}" <<'PY' > "${EVID}/positive_summary.txt" 2>&1
import json, sys
s = json.load(open(sys.argv[1]))
print('samples_written:', s.get('samples_written'))
print('reject_total:', s.get('reject_total'))
print('reject_counts_by_reason:', s.get('reject_counts_by_reason'))
print('first/last_written_timestamp:', s.get('first_written_timestamp'),
      s.get('last_written_timestamp'))
print('max_imu_delta:', s.get('max_imu_delta'))
print('max_command_age:', s.get('max_command_age'))
print('max_contact_age_left/right:', s.get('max_contact_age_left'),
      s.get('max_contact_age_right'))
print('command_accepted/rejected:', s.get('command_accepted'), s.get('command_rejected'))
PY
else
  echo "ERROR: collector_summary.json was not produced"
fi
