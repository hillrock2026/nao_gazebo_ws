#!/usr/bin/env bash
# Task 4 Phase 4E-0 commit-failure / output-collision runtime tests.
#
#   A. injected pre-commit validator failure -> dataset NOT published
#   B. output collision (overwrite=false)     -> refuses, existing files intact
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"
EVID="${WS}/docs/task4_phase4e0_evidence"
OUT="${EVID}/commit_failure_tests.txt"
FAILVALIDATOR="${WS}/tools/phase4e0_failing_validator.py"
mkdir -p "${EVID}"
: > "${OUT}"

wait_pgid () {
  for _ in $(seq 1 30); do [ -s "$1" ] && break; sleep 0.2; done
  cat "$1"
}
wait_summary () {
  for _ in $(seq 1 120); do
    [ -s "$1" ] && break
    sleep 1
  done
}
stop_group () {
  local pgid="$1"
  kill -INT -- "-${pgid}" 2>/dev/null || true
  sleep 6
  kill -TERM -- "-${pgid}" 2>/dev/null || true
  sleep 4
}

echo "### A. injected pre-commit validator failure" | tee -a "${OUT}"
export GZ_PARTITION=t4p4e0_commitfail ROS_DOMAIN_ID=155
SUM_A="${EVID}/commit_failure_summary.json"
DS_A="${EVID}/commit_failure_dataset"
rm -rf "${DS_A}"; rm -f "${SUM_A}"
set -m
rm -f /tmp/p4e0_cf.pgid
setsid bash -c "echo \$\$ > /tmp/p4e0_cf.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS_A} phase4d_summary_path:=${SUM_A} \
  phase4d_validator_script:=${FAILVALIDATOR} \
  phase4d_duration_sec:=4.0 phase4d_overwrite:=true" > "${EVID}/launch_commit_failure.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4e0_cf.pgid)
wait_summary "${SUM_A}"
stop_group "${PGID}"
python3 - "${SUM_A}" "${DS_A}" "${OUT}" <<'PY' 2>&1 | tee -a "${OUT}"
import json, os, sys
summ, ds, out = sys.argv[1], sys.argv[2], sys.argv[3]
s = json.load(open(summ))
published = os.path.isdir(ds)
staging = [n for n in os.listdir(os.path.dirname(ds))
           if n.startswith('.phase4d_staging_')]
ok = (not published and not staging and s.get('dataset_committed') is False
      and s.get('success') is False)
print('dataset_published:', published)
print('staging_residual:', staging)
print('dataset_committed:', s.get('dataset_committed'))
print('process_completed:', s.get('process_completed'))
print('RESULT:', 'PASS' if ok else 'FAIL')
PY

echo | tee -a "${OUT}"
echo "### B. output collision refused (overwrite=false)" | tee -a "${OUT}"
export GZ_PARTITION=t4p4e0_collision ROS_DOMAIN_ID=156
DS_B="${EVID}/collision_dataset"
SUM_B="${EVID}/collision_summary.json"
rm -rf "${DS_B}"; rm -f "${SUM_B}"
mkdir -p "${DS_B}"
printf 'PRE-EXISTING\n' > "${DS_B}/manifest.json"
rm -f /tmp/p4e0_col.pgid
setsid bash -c "echo \$\$ > /tmp/p4e0_col.pgid; exec ros2 launch nao_gz_sim nao_sim.launch.py \
  gui:=false use_rsp:=true enable_phase4d:=true phase4d_run_collector:=true \
  phase4d_output_dir:=${DS_B} phase4d_summary_path:=${SUM_B} \
  phase4d_duration_sec:=4.0 phase4d_overwrite:=false" > "${EVID}/launch_collision.log" 2>&1 &
PGID=$(wait_pgid /tmp/p4e0_col.pgid)
sleep 8
stop_group "${PGID}"
python3 - "${DS_B}" "${OUT}" <<'PY' 2>&1 | tee -a "${OUT}"
import os, sys
ds, out = sys.argv[1], sys.argv[2]
content = open(os.path.join(ds, 'manifest.json')).read().strip()
ok = (content == 'PRE-EXISTING' and sorted(os.listdir(ds)) == ['manifest.json'])
print('collision_files:', sorted(os.listdir(ds)))
print('pre_existing_preserved:', content == 'PRE-EXISTING')
print('RESULT:', 'PASS' if ok else 'FAIL')
PY
