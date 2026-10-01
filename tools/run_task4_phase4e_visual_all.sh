#!/usr/bin/env bash
# Task 4 Phase 4E-Visual: run the four Gazebo GUI demonstration runs and
# aggregate the evidence.  Fixed, Phase 4B/4C-validated gait parameters only.
set +u

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
WS=/home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
cd "${WS}"
EVID="${WS}/docs/task4_phase4e_visual_evidence"
mkdir -p "${EVID}"
rm -rf "${EVID}/walk_side" "${EVID}/walk_front" "${EVID}/walk_foot" \
       "${EVID}/stand_baseline"

echo "=== run 1/4: walk, world-fixed side view (with pause/resume) ==="
python3 tools/run_task4_phase4e_visual.py --view side --mode walk \
  --duration 24 --pause --pause-at 9 \
  --outdir "${EVID}/walk_side" --ros-domain-id 180 --gz-partition t4v_side

echo "=== run 2/4: walk, front/quasi-front view ==="
python3 tools/run_task4_phase4e_visual.py --view front --mode walk \
  --duration 24 --outdir "${EVID}/walk_front" \
  --ros-domain-id 181 --gz-partition t4v_front

echo "=== run 3/4: walk, foot close-up (collision view on) ==="
python3 tools/run_task4_phase4e_visual.py --view foot --mode walk \
  --duration 24 --collisions --outdir "${EVID}/walk_foot" \
  --ros-domain-id 182 --gz-partition t4v_foot

echo "=== run 4/4: stand baseline (same world/physics/spawn) ==="
python3 tools/run_task4_phase4e_visual.py --view side --mode stand \
  --stand-sec 24 --outdir "${EVID}/stand_baseline" \
  --ros-domain-id 183 --gz-partition t4v_stand

echo "=== aggregate evidence ==="
python3 tools/phase4e_visual_aggregate.py "${EVID}"

echo "done"
