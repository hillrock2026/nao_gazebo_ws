#!/usr/bin/env bash
# Task 4 Phase 4E: empirical ShoulderPitch sign / neutral-pose probe.
#
# For each requested LShoulderPitch target it launches the existing Phase 4A
# actuator + finite probe (headless), lets the servo settle, then reads the
# world/torso-relative hand position via TF (`base_link` -> `l_wrist`).
# Publishes nothing itself; uses the existing probe tool only.
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"
EVID="${WS}/docs/task4_phase4e_visual_evidence"
mkdir -p "${EVID}"
OUT="${EVID}/arm_sign_probe.txt"
: > "${OUT}"

DOMAIN=190
for TARGET in 0.0 1.5 -1.0; do
  DOMAIN=$((DOMAIN + 1))
  export GZ_PARTITION="t4v_probe_${TARGET//./_}" ROS_DOMAIN_ID=${DOMAIN}
  rm -f /tmp/t4v_probe.pgid
  setsid bash -c "echo \$\$ > /tmp/t4v_probe.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
    gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=true \
    phase4a_run_trace:=false enable_phase4b:=false \
    phase4a_joint:=LShoulderPitch phase4a_target_rad:=${TARGET} \
    phase4a_publish_count:=400 phase4a_publish_period:=0.05 \
    actuator_hold_on_start:=false actuator_p_gain:=15.0 \
    actuator_max_velocity:=4.0" > /tmp/t4v_probe_${TARGET}.log 2>&1 &
  sleep 1
  for _ in $(seq 1 30); do [ -s /tmp/t4v_probe.pgid ] && break; sleep 0.2; done
  PGID=$(cat /tmp/t4v_probe.pgid)
  sleep 12
  {
    echo "==== LShoulderPitch target = ${TARGET} rad ===="
    echo "\$ ros2 run tf2_ros tf2_echo base_link l_wrist"
    timeout 8 ros2 run tf2_ros tf2_echo base_link l_wrist 2>/dev/null \
      | grep -A2 "Translation" | head -3
    echo
  } >> "${OUT}"
  kill -INT -"${PGID}" 2>/dev/null; sleep 4
  kill -TERM -"${PGID}" 2>/dev/null; sleep 3
  pkill -x gz 2>/dev/null
  sleep 1
done
echo "arm sign probe done" >> "${OUT}"
cat "${OUT}"
