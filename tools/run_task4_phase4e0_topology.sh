#!/usr/bin/env bash
# Task 4 Phase 4E-0 runtime topology capture (schema v1.0.1).
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"
EVID="${WS}/docs/task4_phase4e0_evidence"
mkdir -p "${EVID}"

export GZ_PARTITION=t4p4e0_topo ROS_DOMAIN_ID=157
set -m
rm -f /tmp/p4e0_topo.pgid
setsid bash -c "echo \$\$ > /tmp/p4e0_topo.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false \
  phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=true \
  phase4b_gait_method:=walk phase4b_duration_sec:=25.0 phase4b_record_duration:=0.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=/tmp/p4e0_topo_ds phase4d_summary_path:=/tmp/p4e0_topo_summary.json \
  phase4d_duration_sec:=20.0 phase4d_overwrite:=true" > /tmp/p4e0_topology_launch.log 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4e0_topo.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4e0_topo.pgid)

# Give discovery ample time; do NOT restart the daemon.
sleep 14
{
  echo "### Phase 4E-0 regression runtime topology"
  echo "date: $(date -Iseconds)"
  echo
  echo "\$ ros2 node list"
  ros2 node list
  echo
  echo "\$ ros2 topic info /joint_command --verbose"
  ros2 topic info /joint_command --verbose
  echo
  echo "\$ ros2 topic info /joint_states --verbose"
  ros2 topic info /joint_states --verbose
} > "${EVID}/runtime_topology.txt" 2>&1

kill -INT -- "-${PGID}" 2>/dev/null
sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi
rm -rf /tmp/p4e0_topo_ds /tmp/p4e0_topo_summary.json
echo "topology capture done"
grep -cE "Publisher count: 1" "${EVID}/runtime_topology.txt" || true
