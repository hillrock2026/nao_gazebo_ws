#!/usr/bin/env bash
# Task 4 Phase 4C negative / default-launch / safety checks.
#
# 1. Invalid gait parameters are rejected (producer exits non-zero, logs the
#    offending parameter) instead of being silently clamped.
# 2. The Phase 4A probe and the Phase 4B producer never publish together.
# 3. The default launch starts no actuator / producer / trace and exposes no
#    /joint_command topic (default behaviour unchanged).
# 4. enable_phase4b:=true with phase4b_run_gait:=false loads the actuator but
#    starts no producer and no /joint_command publisher.
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

EVID="${WS}/docs/task4_phase4c_evidence"
OUT="${EVID}/negative_tests.txt"
: > "${OUT}"

echo "### 1. invalid gait parameter rejection (offline, no sim needed)" | tee -a "${OUT}"
run_invalid() {
  local label="$1"; shift
  local out
  out=$(timeout 15 ros2 run nao_gz_sim phase4b_producer.py --ros-args "$@" 2>&1)
  local code=$?
  echo "--- ${label}: ${*} -> exit=${code}" | tee -a "${OUT}"
  echo "${out}" | grep -E "invalid parameter|ValueError" | head -2 | tee -a "${OUT}"
}
for spec in \
  "step_length -p step_length_m:=0.5" \
  "frequency -p step_frequency_hz:=5.0" \
  "swing_height -p swing_height_m:=0.2" \
  "double_support -p double_support_ratio:=0.7" \
  "command_rate -p command_rate_hz:=1.0" \
  "sway -p sway_amplitude_rad:=1.0" \
  "leg_extension -p leg_extension_m:=0.30" \
  "duration -p duration_sec:=0.0"; do
  run_invalid ${spec}
done
echo | tee -a "${OUT}"

echo "### 2. probe/producer mutual exclusion + live runtime topology" | tee -a "${OUT}"
export GZ_PARTITION=t4p4c_neg ROS_DOMAIN_ID=133
NEG_DIR="${EVID}/negative_run"
mkdir -p "${NEG_DIR}"
rm -f "${NEG_DIR}"/*.jsonl "${NEG_DIR}/phase4b_summary.json"
set -m
# Long duration keeps the producer alive while the runtime graph is inspected.
setsid bash -c "echo \$\$ > /tmp/p4c_neg.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=true \
  phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=true \
  phase4b_gait_method:=walk phase4b_duration_sec:=120.0 phase4b_record_duration:=130.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 phase4b_swing_height_m:=0.008 \
  phase4b_double_support_ratio:=0.3 phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  phase4b_evidence_dir:=${NEG_DIR}" > "${EVID}/launch_negative.log" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4c_neg.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4c_neg.pgid)
for _ in $(seq 1 100); do
  ros2 topic info /joint_command 2>/dev/null | grep -q "Publisher count: 1" && break
  sleep 0.5
done
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "### runtime_topology (Phase 4C, producer alive)"
  echo "\$ ros2 node list"
  ros2 node list
  echo
  echo "\$ ros2 topic list"
  ros2 topic list
  echo
  echo "\$ ros2 topic info /joint_command --verbose"
  ros2 topic info /joint_command --verbose
  echo
  echo "\$ ros2 topic info /joint_states --verbose"
  ros2 topic info /joint_states --verbose
  echo
  echo "\$ gz topic -l"
  gz topic -l
} > "${EVID}/runtime_topology.txt" 2>&1
{
  echo "phase4a_probe node present : $(ros2 node list 2>/dev/null | grep -c phase4a_probe)"
  echo "phase4b_producer node present: $(ros2 node list 2>/dev/null | grep -c phase4b_producer)"
  echo "--- /joint_command publisher ---"
  ros2 topic info /joint_command --verbose 2>/dev/null | grep -A4 "Publisher count" | head -8
} | tee -a "${OUT}"
kill -INT -- "-${PGID}" 2>/dev/null; sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi
echo "residual=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)" | tee -a "${OUT}"
# The negative run's long traces are not used as measurement evidence.
rm -f "${NEG_DIR}"/*.jsonl "${NEG_DIR}/phase4b_summary.json"

echo "### 3. default launch (enable_phase4a:=false enable_phase4b:=false)" | tee -a "${OUT}"
export GZ_PARTITION=t4p4c_def ROS_DOMAIN_ID=134
DEF_LOG="${EVID}/launch_default.log"
rm -f "${DEF_LOG}"
setsid bash -c "echo \$\$ > /tmp/p4c_def.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true" > "${DEF_LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4c_def.pgid ] && break; sleep 0.2; done
DEFPGID=$(cat /tmp/p4c_def.pgid)
sleep 18
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "nao_phase4a_actuator nodes : $(ros2 node list 2>/dev/null | grep -c nao_phase4a_actuator)"
  echo "phase4a_probe nodes        : $(ros2 node list 2>/dev/null | grep -c phase4a_probe)"
  echo "phase4b_producer nodes     : $(ros2 node list 2>/dev/null | grep -c phase4b_producer)"
  echo "phase4b_trace nodes        : $(ros2 node list 2>/dev/null | grep -c phase4b_trace)"
  echo "/joint_command in list     : $(ros2 topic list 2>/dev/null | grep -c '^/joint_command$')"
  echo "--- topic info /joint_command ---"
  ros2 topic info /joint_command 2>&1
  echo "Task 3 topics present      : $(ros2 topic list 2>/dev/null | grep -cE '^/joint_states$|^/imu/data$|^/body_pose$|^/left_foot/contact$|^/right_foot/contact$|^/tf$|^/clock$')/7"
} | tee -a "${OUT}"
kill -INT -- "-${DEFPGID}" 2>/dev/null; sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${DEFPGID}" 2>/dev/null; sleep 4; fi
echo "default residual=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)" | tee -a "${OUT}"

echo "### 4. enable_phase4b:=true phase4b_run_gait:=false (actuator only)" | tee -a "${OUT}"
export GZ_PARTITION=t4p4c_act ROS_DOMAIN_ID=135
rm -f "${EVID}/launch_actuator_only.log"
setsid bash -c "echo \$\$ > /tmp/p4c_act.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false \
  phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=false" \
  > "${EVID}/launch_actuator_only.log" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4c_act.pgid ] && break; sleep 0.2; done
ACTPGID=$(cat /tmp/p4c_act.pgid)
sleep 15
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "actuator ready in gz log   : $(grep -c 'nao_phase4a_actuator ready' "${EVID}/launch_actuator_only.log")"
  echo "phase4b_producer nodes     : $(ros2 node list 2>/dev/null | grep -c phase4b_producer)"
  echo "--- /joint_command publisher count ---"
  ros2 topic info /joint_command 2>&1 | grep -i "publisher"
} | tee -a "${OUT}"
kill -INT -- "-${ACTPGID}" 2>/dev/null; sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${ACTPGID}" 2>/dev/null; sleep 4; fi
echo "actuator-only residual=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)" | tee -a "${OUT}"
