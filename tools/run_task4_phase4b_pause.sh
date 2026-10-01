#!/usr/bin/env bash
# Task 4 Phase 4B pause/resume + long-run stability check.
#
# Runs one gait episode, pauses Gazebo for ~3 sim s mid-run, resumes, and checks
# that command stamps never go backwards or duplicate and that the run finishes
# with no controller error or residual process.
set +u

WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"

export GZ_PARTITION=t4p4b_pause
export ROS_DOMAIN_ID=122
EVID="${WS}/docs/task4_phase4b_evidence"
RUN_DIR="${EVID}/pause_run"
LOG="${EVID}/launch_pause.log"
OUT="${EVID}/stability_check.txt"
mkdir -p "${RUN_DIR}"
rm -f "${RUN_DIR}"/*.jsonl "${RUN_DIR}/phase4b_summary.json" "${LOG}"

setsid bash -c "echo \$\$ > /tmp/p4b_pause.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
  enable_phase4b:=true phase4b_run_gait:=true phase4b_gait_method:=walk \
  phase4b_duration_sec:=16.0 phase4b_record_duration:=22.0 \
  actuator_hold_on_start:=true actuator_p_gain:=15.0 actuator_max_velocity:=4.0 \
  phase4b_step_length_m:=0.02 phase4b_step_frequency_hz:=0.5 phase4b_swing_height_m:=0.008 \
  phase4b_double_support_ratio:=0.3 phase4b_sway_amplitude_rad:=0.06 phase4b_arm_swing_rad:=0.08 \
  phase4b_evidence_dir:=${RUN_DIR}" > "${LOG}" 2>&1 &
sleep 1
for _ in $(seq 1 30); do [ -s /tmp/p4b_pause.pgid ] && break; sleep 0.2; done
PGID=$(cat /tmp/p4b_pause.pgid)

# Wait until the producer is publishing and the gait has started (sim > 4).
sleep 12
{
  echo "### pause/resume run"
  echo "pause at wall $(date +%s)"
} >> "${OUT}"
gz service -s /world/nao_world/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true' >> "${OUT}" 2>&1
sleep 4
gz service -s /world/nao_world/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >> "${OUT}" 2>&1
echo "resume at wall $(date +%s)" >> "${OUT}"

for _ in $(seq 1 150); do
  [ -s "${RUN_DIR}/phase4b_summary.json" ] && break
  sleep 1
done

python3 - "${RUN_DIR}" <<'PY' >> "${OUT}" 2>&1
import json, sys, os
d = sys.argv[1]
s = json.load(open(os.path.join(d, 'phase4b_summary.json')))
print('pause/run producer_contract_pass:', s.get('producer_contract_pass'))
print('pause/run walking_pass          :', s.get('walking_pass'))
print('pause/run command_stamps_increasing:', s.get('command_stamps_strictly_increasing'))
print('pause/run command_accepted      :', s.get('command_accepted'))
print('pause/run command_rate_hz       :', s.get('command_rate_hz'))
print('pause/run body_net_forward_m    :', s.get('body_net_forward_m'))
print('pause/run body_max_roll_deg     :', s.get('body_max_roll_deg'))
# detect the sim-time gap caused by the pause in the command trace
stamps = []
for line in open(os.path.join(d, 'command_trace.jsonl')):
    r = json.loads(line)
    if r.get('ok'):
        stamps.append(r['t'])
gaps = [(b - a, a, b) for a, b in zip(stamps, stamps[1:]) if b - a > 0.5]
print('pause/run sim-time gaps > 0.5s in command stamps:', gaps)
print('pause/run strictly increasing:', all(b > a for a, b in zip(stamps, stamps[1:])))
PY

kill -INT -- "-${PGID}" 2>/dev/null; sleep 6
if pgrep -x gz >/dev/null 2>&1; then kill -TERM -- "-${PGID}" 2>/dev/null; sleep 4; fi
{
  echo "pause/run residual=$(ps -eo cmd | grep -E 'gz sim|nao_sim.launch|install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge' | grep -v grep | wc -l)"
  echo
} >> "${OUT}"
