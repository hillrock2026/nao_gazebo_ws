#!/usr/bin/env bash
# Task 4 Phase 4C validation runner.
#
# Runs the existing Phase 4B producer in isolated partitions and captures each
# run's raw traces plus a per-run run_config.json:
#
#   bash tools/run_task4_phase4c_validation.sh baseline   # static stand baseline
#   bash tools/run_task4_phase4c_validation.sh walk 3     # 3 independent gait runs
#
# All runs use the SAME physics engine, spawn height, world, trace rate and
# duration; only the gait method differs for the baseline. The read-only trace
# is finalised by its own record_duration, then the process group is stopped and
# the residual-process count is appended to stability_check.txt.
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

EVID="${WS}/docs/task4_phase4c_evidence"
mkdir -p "${EVID}"
MODE="${1:-walk}"
RUNS="${2:-3}"
DURATION="${PHASE4C_DURATION:-22.0}"
RECORD="$(python3 -c "print($DURATION+6.0)")"
STABILITY="${EVID}/stability_check.txt"
PGID_FILE="/tmp/p4c_validation.pgid"

write_config () {
  python3 - "$1" "$2" "$DURATION" <<'PY'
import json, sys
d, method, dur = sys.argv[1], sys.argv[2], float(sys.argv[3])
cfg = {
    "gait_method": method,
    "start_delay_sec": 0.5,
    "gait_blend_sec": 1.5,
    "duration_sec": dur,
    "stop_ramp_sec": 1.5,
    "hold_final_sec": 2.0,
    "stand_sec": dur + 2.0,
    "step_frequency_hz": 0.5,
    "command_hold_max": 0.05,
    "supplementary_clearance_m": 0.004,
    "command_rate_hz": 100.0,
    "step_length_m": 0.02,
    "swing_height_m": 0.008,
    "double_support_ratio": 0.3,
    "sway_amplitude_rad": 0.06,
    "arm_swing_rad": 0.08,
    "actuator_p_gain": 15.0,
    "actuator_max_velocity": 4.0,
    "hold_on_start": True,
}
with open(d + "/run_config.json", "w") as fh:
    json.dump(cfg, fh, indent=2)
PY
}

run_one () {
  local label="$1" run_dir="$2" partition="$3" domain="$4" method="$5" topo="$6"
  mkdir -p "${run_dir}"
  rm -f "${run_dir}"/*.jsonl "${run_dir}"/phase4b_summary.json
  write_config "${run_dir}" "${method}"
  local log="${EVID}/launch_${label}.log"
  rm -f "${log}" "${run_dir}/phase4b_summary.json"

  export GZ_PARTITION="${partition}" ROS_DOMAIN_ID="${domain}"
  set -m
  setsid bash -c "echo \$\$ > ${PGID_FILE}; exec ros2 launch nao_gz_sim nao_sim.launch.py \
    gui:=false use_rsp:=true \
    enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
    enable_phase4b:=true phase4b_run_gait:=true phase4b_gait_method:=${method} \
    phase4b_record_duration:=${RECORD} phase4b_duration_sec:=${DURATION} \
    phase4b_stand_sec:=$(python3 -c "print(${DURATION}+2.0)") \
    phase4b_hold_final_sec:=2.0 \
    actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
    phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 \
    phase4b_swing_height_m:=0.008 phase4b_double_support_ratio:=0.3 \
    phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
    phase4b_evidence_dir:=${run_dir}" > "${log}" 2>&1 &
  sleep 1
  for _ in $(seq 1 30); do [ -s "${PGID_FILE}" ] && break; sleep 0.2; done
  local pgid
  pgid=$(cat "${PGID_FILE}")

  # Wait for the read-only trace to finalise (bounded; surface a hard failure).
  local waited=0
  for _ in $(seq 1 240); do
    [ -s "${run_dir}/phase4b_summary.json" ] && break
    sleep 1
    waited=$((waited + 1))
  done
  if [ ! -s "${run_dir}/phase4b_summary.json" ]; then
    echo "ERROR: ${label} produced no phase4b_summary.json after ${waited}s" | tee -a "${STABILITY}"
  fi

  if [ "${topo}" = "yes" ]; then
    {
      echo "### run ${label} runtime topology (captured after trace finalise)"
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
    } > "${EVID}/runtime_topology_${label}.txt" 2>&1
  fi

  kill -INT -- "-${pgid}" 2>/dev/null
  sleep 6
  if pgrep -x gz >/dev/null 2>&1; then
    kill -TERM -- "-${pgid}" 2>/dev/null
    sleep 4
  fi
  local residual
  residual=$(ps -eo cmd | grep -E \
    "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot" \
    | grep -v grep | wc -l)
  {
    echo "=== ${label}: partition=${partition} domain=${domain} method=${method} ==="
    echo "residual_process_count=${residual}"
    ps -eo pid,cmd | grep -E \
      "gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot" \
      | grep -v grep || true
    echo
  } >> "${STABILITY}"
  echo "${label}: residual=${residual}"
}

if [ "${MODE}" = "baseline" ]; then
  run_one "baseline" "${EVID}/baseline" "t4p4c_base" "110" "stand" "no"
else
  for i in $(seq 1 "${RUNS}"); do
    run_one "run${i}" "${EVID}/run${i}" "t4p4c_run${i}" "$((110 + i))" "walk" "no"
  done
fi
