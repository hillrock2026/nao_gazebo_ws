#!/usr/bin/env bash
# Task 4 Phase 4A negative-test runner.
#
# Loads the Phase 4A actuator without the built-in probe, runs the negative
# test publisher (illegal commands + one final valid command) and records the
# actuator's rejection log and the resulting topology.
set +u

WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

export GZ_PARTITION="${GZ_PARTITION:-t4p4a_neg}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"

EVID="${WS}/docs/task4_phase4a_evidence"
LOG="${EVID}/launch_phase4a_negative.log"
OUT="${EVID}/negative_tests.txt"
rm -f "${LOG}" "${OUT}" "${EVID}/phase4a_summary.json" \
      "${EVID}/command_state_trace.jsonl"

setsid bash -c \
  "echo \$\$ > /tmp/phase4a_neg.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
   gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false \
   phase4a_record_duration:=0.0" > "${LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/phase4a_neg.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/phase4a_neg.pgid)
echo "launch pgid=${PGID}"

# Wait for the actuator subscription and a valid /joint_states stream.
for _ in $(seq 1 80); do
  if ros2 topic info /joint_command 2>/dev/null | grep -q "Subscription count"; then
    break
  fi
  sleep 0.5
done
sleep 8

python3 "${WS}/tools/phase4a_negative.py" --ros-args -p use_sim_time:=true \
  2>&1 | tee -a "${OUT}"

sleep 2
{
  echo
  echo "### actuator rejection log lines"
  grep "nao_phase4a_actuator: rejected command" "${LOG}" || echo "(none)"
  echo
  echo "### /joint_command after the test publisher exited"
  ros2 topic info /joint_command
  echo
  echo "### node list"
  ros2 node list
} >> "${OUT}" 2>&1

kill -INT -- "-${PGID}" 2>/dev/null
sleep 6
if pgrep -x gz >/dev/null 2>&1; then
  kill -TERM -- "-${PGID}" 2>/dev/null
  sleep 4
fi

echo "=== residual process count (expect 0) ==="
ps -eo cmd | grep -E \
  "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot" \
  | grep -v grep | wc -l
