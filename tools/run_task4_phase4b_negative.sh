#!/usr/bin/env bash
# Task 4 Phase 4B negative / safety checks.
#
# 1. Invalid gait parameters are rejected (producer exits non-zero, logs the
#    offending parameter) instead of being silently clamped.
# 2. The Phase 4A probe and the Phase 4B producer never publish together.
# 3. The default topic graph has a single /joint_command publisher.
set +u

WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

EVID="${WS}/docs/task4_phase4b_evidence"
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
run_invalid "step_length" -p gait_method:=walk -p step_length_m:=0.5
run_invalid "frequency" -p gait_method:=walk -p step_frequency_hz:=5.0
run_invalid "swing_height" -p gait_method:=walk -p swing_height_m:=0.2
run_invalid "double_support" -p gait_method:=walk -p double_support_ratio:=0.7
run_invalid "command_rate" -p gait_method:=walk -p command_rate_hz:=1.0
run_invalid "sway" -p gait_method:=walk -p sway_amplitude_rad:=1.0
echo | tee -a "${OUT}"

echo "### 2. probe/producer mutual exclusion (runtime)" | tee -a "${OUT}"
export GZ_PARTITION=t4p4b_neg
export ROS_DOMAIN_ID=120
NEG_DIR="${EVID}/negative_run"
mkdir -p "${NEG_DIR}"
rm -f "${NEG_DIR}"/*.jsonl "${NEG_DIR}/phase4b_summary.json"
setsid bash -c "echo \$\$ > /tmp/p4b_neg.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=true \
  phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=true \
  phase4b_gait_method:=walk phase4b_duration_sec:=6.0 phase4b_record_duration:=10.0 \
  actuator_hold_on_start:=true phase4b_evidence_dir:=${NEG_DIR}" > "${EVID}/launch_negative.log" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4b_neg.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4b_neg.pgid)
for _ in $(seq 1 100); do
  ros2 topic info /joint_command 2>/dev/null | grep -q "Publisher count: 1" && break
  sleep 0.5
done
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
{
  echo "phase4a_probe node present : $(ros2 node list 2>/dev/null | grep -c phase4a_probe)"
  echo "phase4b_producer node present: $(ros2 node list 2>/dev/null | grep -c phase4b_producer)"
  echo "--- /joint_command publisher ---"
  ros2 topic info /joint_command --verbose 2>/dev/null | grep -A1 "PUBLISHER" | head
} | tee -a "${OUT}"
kill -INT -- "-${PGID}" 2>/dev/null; sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi
echo "residual=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)" | tee -a "${OUT}"
