#!/usr/bin/env bash
# Task 4 Phase 4A closed-loop validation runner.
#
# Starts the Phase 4A execution chain (enable_phase4a:=true), captures the
# runtime topology while the probe is active, waits for the trace summary,
# then stops the launch with SIGINT to the whole process group (equivalent to
# Ctrl-C) and checks that no process is left behind.
#
# This script changes nothing in the repository; it only launches and observes.
set +u

WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

export GZ_PARTITION="${GZ_PARTITION:-t4p4a_run}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

EVID="${WS}/docs/task4_phase4a_evidence"
mkdir -p "${EVID}"
LOG="${EVID}/launch_phase4a.log"
rm -f "${EVID}/command_state_trace.jsonl" "${EVID}/phase4a_summary.json" "${LOG}"

# Launch in a new session so the whole tree can receive SIGINT like Ctrl-C.
setsid bash -c \
  "echo \$\$ > /tmp/phase4a.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
   gui:=false use_rsp:=true enable_phase4a:=true" > "${LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/phase4a.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/phase4a.pgid)
echo "launch pgid=${PGID}"

# Make sure the ROS CLI daemon serves this graph/domain before querying.
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
sleep 2

# Wait until the finite probe is publishing (shows the /joint_command publisher).
for _ in $(seq 1 200); do
  if ros2 topic info /joint_command 2>/dev/null | grep -q "Publisher count: 1"; then
    break
  fi
  sleep 0.3
done

{
  echo "### ros2 topic info /joint_command --verbose (probe active)"
  ros2 topic info /joint_command --verbose
  echo
  echo "### ros2 node list"
  ros2 node list
  echo
  echo "### ros2 topic list"
  ros2 topic list
  echo
  echo "### ros2 topic info /joint_states --verbose"
  ros2 topic info /joint_states --verbose
} > "${EVID}/runtime_topology.txt" 2>&1

# Wait for the trace recorder to finalise (record_duration_sec of sim time).
for _ in $(seq 1 150); do
  [ -s "${EVID}/phase4a_summary.json" ] && break
  sleep 1
done
if [ -s "${EVID}/phase4a_summary.json" ]; then
  echo "summary: OK"
else
  echo "summary: MISSING"
fi

# Graceful stop: SIGINT the entire launch process group.
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
