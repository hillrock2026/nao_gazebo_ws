#!/usr/bin/env bash
# Task 4 Phase 4A default-launch negative check.
#
# Starts the unchanged default launch (enable_phase4a:=false) and records that:
#   * no Phase 4A actuator or probe node exists;
#   * /joint_command does not exist and has no publisher/subscriber;
#   * the Task 3 public interfaces are still produced by their original nodes.
set +u

WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

export GZ_PARTITION="${GZ_PARTITION:-t4p4a_default}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-79}"

EVID="${WS}/docs/task4_phase4a_evidence"
LOG="${EVID}/launch_default.log"
OUT="${EVID}/negative_default_launch.txt"
rm -f "${LOG}" "${OUT}"

setsid bash -c \
  "echo \$\$ > /tmp/phase4a_def.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
   gui:=false use_rsp:=true" > "${LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/phase4a_def.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/phase4a_def.pgid)
echo "launch pgid=${PGID}"
sleep 18

{
  echo "### ros2 node list (default launch)"
  ros2 daemon stop >/dev/null 2>&1
  ros2 daemon start >/dev/null 2>&1
  sleep 2
  ros2 node list
  echo
  echo "### ros2 topic list (default launch)"
  ros2 topic list
  echo
  echo "### ros2 topic info /joint_command (default launch)"
  ros2 topic info /joint_command 2>&1
  echo
  echo "### phase4a node/subscription occurrences"
  echo "nao_phase4a_actuator in node list: $(ros2 node list 2>/dev/null | grep -c nao_phase4a_actuator)"
  echo "phase4a_probe in node list      : $(ros2 node list 2>/dev/null | grep -c phase4a_probe)"
  echo "/joint_command in topic list    : $(ros2 topic list 2>/dev/null | grep -c '^/joint_command$')"
} > "${OUT}" 2>&1

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
