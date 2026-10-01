#!/usr/bin/env bash
# Task 4 Phase 4E-0 short real closed-loop regression (schema v1.0.1).
#
# Runs the real Phase 4B gait producer with the actuator and the read-only
# Phase 4D/4E-0 collector, caps the dataset at <= 1000 samples, then validates
# it with the independent golden validator.
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

EVID="${WS}/docs/task4_phase4e0_evidence"
OUT="${EVID}/regression_collection"
LOG="${EVID}/launch_regression.log"
SUMMARY="${EVID}/regression_summary.json"
VALIDATOR="${WS}/tools/validate_task4_phase4d_dataset.py"
DURATION="${PHASE4E0_DURATION:-20.0}"
MAXSAMPLES="${PHASE4E0_MAX_SAMPLES:-800}"
mkdir -p "${EVID}"
rm -rf "${OUT}"
rm -f "${SUMMARY}" "${LOG}"
: > "${EVID}/stability_check.txt"

export GZ_PARTITION=t4p4e0_reg ROS_DOMAIN_ID=150
set -m
rm -f /tmp/p4e0_reg.pgid 2>/dev/null
setsid bash -c "echo \$\$ > /tmp/p4e0_reg.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true \
  enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
  enable_phase4b:=true phase4b_run_gait:=true phase4b_gait_method:=walk \
  phase4b_duration_sec:=24.0 phase4b_record_duration:=0.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 \
  phase4b_swing_height_m:=0.008 phase4b_double_support_ratio:=0.3 \
  phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${OUT} phase4d_summary_path:=${SUMMARY} \
  phase4d_validator_script:=${VALIDATOR} \
  phase4d_duration_sec:=${DURATION} phase4d_max_samples:=${MAXSAMPLES} \
  phase4d_overwrite:=true" > "${LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4e0_reg.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4e0_reg.pgid)

# Capture the live runtime topology while the producer is publishing.
sleep 8
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "### Phase 4E-0 regression runtime topology"
  echo "\$ ros2 node list"
  ros2 node list
  echo
  echo "\$ ros2 topic info /joint_command --verbose"
  ros2 topic info /joint_command --verbose
  echo
  echo "\$ ros2 topic info /joint_states --verbose"
  ros2 topic info /joint_states --verbose
} > "${EVID}/runtime_topology.txt" 2>&1

for _ in $(seq 1 240); do
  [ -s "${SUMMARY}" ] && break
  sleep 1
done

kill -INT -- "-${PGID}" 2>/dev/null
sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi
RESIDUAL=99
for _ in $(seq 1 20); do
  RESIDUAL=$(ps -eo cmd | grep -E \
    "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot" \
    | grep -v grep | wc -l)
  [ "${RESIDUAL}" -eq 0 ] && break
  sleep 1
done
echo "regression residual=${RESIDUAL}" | tee -a "${EVID}/stability_check.txt"

if [ -d "${OUT}" ]; then
  PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 "${VALIDATOR}" "${OUT}" \
    --json-out "${EVID}/regression_validation.json" >> "${LOG}" 2>&1
  echo "validator_exit=$?" >> "${EVID}/stability_check.txt"
else
  echo "ERROR: regression dataset directory missing" | tee -a "${LOG}"
  exit 2
fi
