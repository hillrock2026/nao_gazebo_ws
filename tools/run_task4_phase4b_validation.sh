#!/usr/bin/env bash
# Task 4 Phase 4B validation runner.
#
# Runs the minimal periodic gait producer N times in isolated partitions,
# captures the runtime topology (run1), saves each run's raw traces and summary
# under docs/task4_phase4b_evidence/runN/, and records the shutdown/ps evidence.
set +u

WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

EVID="${WS}/docs/task4_phase4b_evidence"
RUNS="${1:-3}"
DURATION="${PHASE4B_DURATION:-22.0}"
RECORD="$(python3 -c "print($DURATION+6.0)")"
STABILITY="${EVID}/stability_check.txt"
: > "${STABILITY}"

for i in $(seq 1 "${RUNS}"); do
  export GZ_PARTITION="t4p4b_run${i}"
  export ROS_DOMAIN_ID="$((100 + i))"
  RUN_DIR="${EVID}/run${i}"
  mkdir -p "${RUN_DIR}"
  LOG="${EVID}/launch_run${i}.log"
  rm -f "${RUN_DIR}"/*.jsonl "${RUN_DIR}/phase4b_summary.json" "${LOG}"

  echo "=== run ${i}: partition=${GZ_PARTITION} domain=${ROS_DOMAIN_ID} ===" | tee -a "${STABILITY}"
  setsid bash -c "echo \$\$ > /tmp/p4b_run.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
    gui:=false use_rsp:=true \
    enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
    enable_phase4b:=true phase4b_run_gait:=true phase4b_gait_method:=walk \
    phase4b_record_duration:=${RECORD} \
    phase4b_duration_sec:=${DURATION} \
    actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
    phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 \
    phase4b_swing_height_m:=0.008 phase4b_double_support_ratio:=0.3 \
    phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
    phase4b_evidence_dir:=${RUN_DIR}" > "${LOG}" 2>&1 &
  sleep 1
  for _ in $(seq 1 30); do [ -s /tmp/p4b_run.pgid ] && break; sleep 0.2; done
  PGID=$(cat /tmp/p4b_run.pgid)

  # Wait until the producer is publishing.
  for _ in $(seq 1 120); do
    if ros2 topic info /joint_command 2>/dev/null | grep -q "Publisher count: 1"; then
      break
    fi
    sleep 0.5
  done

  if [ "${i}" = "1" ]; then
    ros2 daemon stop >/dev/null 2>&1
    ros2 daemon start >/dev/null 2>&1
    sleep 2
    {
      echo "### ros2 topic list"
      ros2 topic list
      echo
      echo "### ros2 node list"
      ros2 node list
      echo
      echo "### ros2 topic info /joint_command --verbose"
      ros2 topic info /joint_command --verbose
      echo
      echo "### ros2 topic info /joint_states --verbose"
      ros2 topic info /joint_states --verbose
      echo
      echo "### gz topic -l (contact + joint + clock)"
      gz topic -l 2>/dev/null | grep -iE "contact|joint|clock|pose"
    } > "${EVID}/runtime_topology.txt" 2>&1
  fi

  for _ in $(seq 1 150); do
    [ -s "${RUN_DIR}/phase4b_summary.json" ] && break
    sleep 1
  done

  kill -INT -- "-${PGID}" 2>/dev/null
  sleep 6
  if pgrep -x gz >/dev/null 2>&1; then
    kill -TERM -- "-${PGID}" 2>/dev/null
    sleep 4
  fi
  RESIDUAL=$(ps -eo cmd | grep -E \
    "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot" \
    | grep -v grep | wc -l)
  {
    echo "residual_process_count=${RESIDUAL}"
    echo "ps snapshot after shutdown:"
    ps -eo pid,cmd | grep -E \
      "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot" \
      | grep -v grep || true
    echo
  } >> "${STABILITY}"
  echo "run ${i}: residual=${RESIDUAL}" | tee -a "${STABILITY}"
done
