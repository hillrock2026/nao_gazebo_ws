#!/usr/bin/env python3
"""Task 4 Phase 4E-Visual single Gazebo GUI demonstration run.

Launches the real Phase 4B producer + Phase 4A actuator with the frozen,
Phase 4B/4C-validated gait parameters under a Gazebo GUI, sets a world-fixed
camera, records region video, logs sim/wall events, optionally performs one
pause/resume, runs the read-only Phase 4B trace, then stops everything through
the process group.

Screenshots are extracted from the recorded video by
``tools/phase4e_visual_postprocess.py`` because the Gazebo OpenGL client window
does not expose reliable per-frame X grabs in this environment; the continuous
ximagesrc recording does reflect the live content.

Nothing here changes gait parameters, physics, spawn pose or the model; the only
interaction with the GUI is camera placement / view toggles (allowed).

See docs/Task4_Phase4E-Visual_prompt_opencode_deepseek.md.
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4e_visual_capture as cap  # noqa: E402

WS = os.path.dirname(_TOOLS)

FIXED = {
    'gait_method': 'walk',
    'walking_direction': 'forward',
    'step_length_m': 0.02,
    'step_frequency_hz': 0.5,
    'swing_height_m': 0.008,
    'double_support_ratio': 0.30,
    'leg_extension_m': 0.24401,
    'sway_amplitude_rad': 0.06,
    'arm_swing_rad': 0.12,
    'arm_neutral_pitch_rad': 1.50,
    'arm_neutral_roll_rad': 0.0,
    'elbow_bend_rad': 0.30,
    'arm_phase_offset_rad': 0.0,
    'command_rate_hz': 100.0,
    'gait_blend_sec': 1.5,
    'stop_ramp_sec': 1.5,
    'hold_final_sec': 2.0,
    'start_delay_sec': 0.5,
    'actuator_p_gain': 15.0,
    'actuator_max_velocity': 4.0,
    'actuator_hold_on_start': True,
    'command_hold_max': 0.05,
    'supplementary_clearance_m': 0.004,
}

VIEWS = {
    'side': {'pos': (0.4, -2.6, 0.80), 'look': (0.4, 0.0, 0.32)},
    'front': {'pos': (1.6, 0.9, 0.85), 'look': (0.4, 0.0, 0.32)},
    'foot': {'pos': (0.1, -0.85, 0.16), 'look': (0.1, 0.0, 0.05)},
}


def log(msg):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)


def read_last_sim(path):
    try:
        with open(path, 'r') as fh:
            last = None
            for line in fh:
                if line.strip():
                    last = line
            return None if last is None else json.loads(last)['sim']
    except (OSError, ValueError):
        return None


def wait_sim(path, target, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = read_last_sim(path)
        if s is not None and s >= target:
            return s
        time.sleep(0.1)
    return None


def pause_sim(paused):
    req = 'pause: true' if paused else 'pause: false'
    return cap.run(['gz', 'service', '-s', '/world/nao_world/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '4000',
                    '--req', req], timeout=15)


def stop_group(pgid):
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        for _ in range(40):
            time.sleep(0.2)
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--view', required=True, choices=sorted(VIEWS))
    ap.add_argument('--mode', default='walk', choices=['walk', 'stand'])
    ap.add_argument('--outdir', required=True)
    ap.add_argument('--duration', type=float, default=24.0)
    ap.add_argument('--stand-sec', type=float, default=24.0)
    ap.add_argument('--pause', action='store_true')
    ap.add_argument('--pause-at', type=float, default=10.0)
    ap.add_argument('--collisions', action='store_true')
    ap.add_argument('--ros-domain-id', type=int, default=170)
    ap.add_argument('--gz-partition', default='t4v_visual')
    args = ap.parse_args()

    outdir = os.path.abspath(args.outdir)
    trace_dir = os.path.join(outdir, 'trace')
    os.makedirs(outdir, exist_ok=True)
    os.makedirs(trace_dir, exist_ok=True)
    for name in os.listdir(outdir):
        if name.endswith(('.webm', '.mp4', '.gst.log')):
            os.remove(os.path.join(outdir, name))
    clock_log = os.path.join(outdir, 'clock_wall.jsonl')
    if os.path.exists(clock_log):
        os.remove(clock_log)

    env = dict(os.environ)
    env['GZ_PARTITION'] = args.gz_partition
    env['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    cap.set_env(env)

    cfg = dict(FIXED)
    cfg['gait_method'] = args.mode
    cfg['duration_sec'] = args.duration
    cfg['stand_sec'] = args.stand_sec
    with open(os.path.join(outdir, 'run_config.json'), 'w') as fh:
        json.dump(cfg, fh, indent=2, sort_keys=True)

    launch_cmd = [
        'ros2', 'launch', 'nao_gz_sim', 'nao_sim.launch.py',
        'gui:=true', 'use_rsp:=true',
        'enable_phase4a:=true', 'phase4a_run_probe:=false',
        'phase4a_run_trace:=false',
        'enable_phase4b:=true', 'phase4b_run_gait:=true',
        'phase4b_run_trace:=true', 'phase4b_record_duration:=0.0',
        'enable_phase4d:=false',
        'actuator_hold_on_start:=true', 'actuator_p_gain:=15.0',
        'actuator_max_velocity:=4.0',
        'phase4b_gait_method:=%s' % args.mode,
        'phase4b_walking_direction:=forward',
        'phase4b_step_length_m:=0.02', 'phase4b_step_frequency_hz:=0.5',
        'phase4b_swing_height_m:=0.008', 'phase4b_double_support_ratio:=0.30',
        'phase4b_leg_extension_m:=0.24401', 'phase4b_gait_blend_sec:=1.5',
        'phase4b_sway_amplitude_rad:=0.06', 'phase4b_arm_swing_rad:=0.12',
        'phase4b_arm_neutral_pitch_rad:=1.50',
        'phase4b_arm_neutral_roll_rad:=0.0',
        'phase4b_elbow_bend_rad:=0.30',
        'phase4b_arm_phase_offset_rad:=0.0',
        'phase4b_command_rate_hz:=100.0',
        'phase4b_stop_ramp_sec:=1.5', 'phase4b_hold_final_sec:=2.0',
        'phase4b_start_delay_sec:=0.5', 'phase4b_stand_sec:=%.1f' % args.stand_sec,
        'phase4b_duration_sec:=%.1f' % args.duration,
        'phase4b_evidence_dir:=%s' % trace_dir,
    ]
    with open(os.path.join(outdir, 'launch_cmd.txt'), 'w') as fh:
        fh.write(' '.join(launch_cmd) + '\n')
    launch_log = open(os.path.join(outdir, 'launch.log'), 'w')
    log('starting launch')
    launch = subprocess.Popen(launch_cmd, stdout=launch_log,
                              stderr=subprocess.STDOUT, cwd=WS, env=env,
                              start_new_session=True)
    pgid = launch.pid
    logger = subprocess.Popen(
        ['python3', os.path.join(_TOOLS, 'phase4e_visual_clock_logger.py'),
         '--out', clock_log],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, cwd=WS, env=env)

    timeline = []
    record_proc = None
    result = {'view': args.view, 'mode': args.mode, 'outdir': outdir}
    webm = os.path.join(outdir, '%s_%s.webm' % (args.mode, args.view))
    try:
        win = cap.wait_for_window(timeout=90)
        if win is None:
            raise RuntimeError('Gazebo GUI window did not appear')
        log('gui window %s at (%d,%d) %dx%d'
            % (win['xid'], win['ax'], win['ay'], win['w'], win['h']))
        result['gui_window'] = win
        with open(os.path.join(outdir, 'gui_window.txt'), 'w') as fh:
            fh.write(json.dumps(win) + '\n')

        # record from GUI-up through the whole run (world-fixed camera view)
        record_proc, record_info = cap.start_record(webm, fps=15)
        record_wall = time.time()
        result['record_start_wall'] = record_wall
        timeline.append({'event': 'gui_ready', 'wall': record_wall,
                         'sim': read_last_sim(clock_log)})

        move_resp = cap.move_camera(VIEWS[args.view]['pos'], VIEWS[args.view]['look'])
        result['camera_service_response'] = move_resp.stdout.strip()
        if args.collisions:
            coll_resp = cap.view_flag('/gui/view/collisions', True)
            result['collision_view_response'] = coll_resp.stdout.strip()
            result['collision_view_stderr'] = coll_resp.stderr.strip()[:300]
        timeline.append({'event': 'camera_set', 'wall': time.time(),
                         'sim': read_last_sim(clock_log)})
        log('camera set to view=%s' % args.view)

        ready = None
        deadline = time.time() + 90
        while time.time() < deadline:
            try:
                text = open(os.path.join(outdir, 'launch.log')).read()
            except OSError:
                text = ''
            m = re.search(r'model ready at sim ([\d.]+)', text)
            if m:
                ready = float(m.group(1))
                break
            time.sleep(0.2)
        if ready is None:
            ready = 1.0
        gait_start = ready + FIXED['start_delay_sec']
        end_sim = (gait_start + args.duration + FIXED['stop_ramp_sec']
                   + FIXED['hold_final_sec'] if args.mode == 'walk'
                   else ready + args.stand_sec)
        result.update({'ready_sim': ready, 'gait_start_sim': gait_start,
                       'end_sim': end_sim})
        log('ready=%.2f gait_start=%.2f end=%.2f' % (ready, gait_start, end_sim))

        # target simulation times for the named frames
        if args.mode == 'walk':
            plan = [('settled_stance', max(0.0, gait_start - 0.4)),
                    ('steady_early', gait_start + FIXED['gait_blend_sec'] + 1.0),
                    ('stop_ramp', gait_start + args.duration + 0.5),
                    ('final_stance', gait_start + args.duration
                     + FIXED['stop_ramp_sec'] + FIXED['hold_final_sec'] - 0.3)]
        else:
            plan = [('settled_stance', max(0.0, ready + 1.0)),
                    ('final_stance', ready + args.stand_sec - 0.5)]
        result['frame_plan'] = plan

        pause_done = False
        topo_done = False
        last_frame_sim = -1.0
        while True:
            s = read_last_sim(clock_log)
            if s is None:
                time.sleep(0.1)
                continue
            if s >= end_sim + 1.5:
                break
            if not topo_done and s >= gait_start + 4.0:
                topo = cap.run(['bash', '-c',
                                'ros2 node list; echo; '
                                'ros2 topic info /joint_command --verbose; echo; '
                                'ros2 topic info /joint_states --verbose'],
                               timeout=40)
                with open(os.path.join(outdir, 'topology.txt'), 'w') as fh:
                    fh.write(topo.stdout + topo.stderr)
                topo_done = True
                timeline.append({'event': 'topology', 'wall': time.time(), 'sim': s})
                log('topology captured at sim %.2f' % s)
            if not pause_done and args.pause and s >= gait_start + args.pause_at:
                log('pause at sim %.3f' % s)
                pause_sim(True)
                timeline.append({'event': 'pause_issued', 'wall': time.time(),
                                 'sim': read_last_sim(clock_log)})
                time.sleep(3.0)
                timeline.append({'event': 'pause_frozen', 'wall': time.time(),
                                 'sim': read_last_sim(clock_log)})
                pause_sim(False)
                timeline.append({'event': 'resume', 'wall': time.time(),
                                 'sim': read_last_sim(clock_log)})
                pause_done = True
                last_frame_sim = s
                continue
            if s - last_frame_sim >= 1.0:
                timeline.append({'event': 'frame', 'wall': time.time(), 'sim': s})
                last_frame_sim = s
            time.sleep(0.1)
        log('captures done')
    except Exception as exc:  # noqa: BLE001
        log('ERROR: %s' % exc)
        result['error'] = str(exc)
    finally:
        if record_proc is not None:
            log('stopping recording')
            cap.stop_record(record_proc)
        log('stopping launch')
        stop_group(pgid)
        try:
            logger.send_signal(signal.SIGINT)
            logger.wait(timeout=10)
        except Exception:  # noqa: BLE001
            logger.kill()
        launch_log.close()
        time.sleep(2)
        resid = cap.run(['bash', '-c',
                         "ps -eo cmd | grep -E 'gz sim|nao_sim.launch|"
                         "install/nao_gz_sim/lib/nao_gz_sim|"
                         "ros_gz_bridge/parameter_bridge|"
                         "phase4e_visual_clock_logger' | grep -v grep | wc -l"],
                        timeout=20)
        try:
            result['residual_processes'] = int(resid.stdout.strip())
        except (ValueError, AttributeError):
            result['residual_processes'] = None
        with open(os.path.join(outdir, 'capture_timeline.json'), 'w') as fh:
            json.dump(timeline, fh, indent=2)

    if os.path.exists(webm) and os.path.getsize(webm) > 0:
        mp4 = webm[:-5] + '.mp4'
        try:
            if cap.convert_to_mp4(webm, mp4, fps=15):
                result['video_mp4'] = os.path.relpath(mp4, outdir)
                log('mp4 written: %s' % mp4)
        except Exception as exc:  # noqa: BLE001
            log('mp4 conversion failed: %s' % exc)

    try:
        proc = subprocess.run(
            ['python3', os.path.join(_TOOLS, 'phase4c_recompute.py'), 'analyze',
             trace_dir, '--config', os.path.join(outdir, 'run_config.json'),
             '--out', os.path.join(outdir, 'phase4c_summary.json')],
            capture_output=True, text=True, cwd=WS, env=env, timeout=300)
        result['recompute_returncode'] = proc.returncode
        if proc.returncode != 0:
            result['recompute_stderr'] = proc.stderr[-2000:]
    except Exception as exc:  # noqa: BLE001
        result['recompute_error'] = str(exc)

    with open(os.path.join(outdir, 'run_result.json'), 'w') as fh:
        json.dump(result, fh, indent=2, sort_keys=True)
    log('run result: %s' % json.dumps({k: result.get(k) for k in
                                       ('view', 'mode', 'error',
                                        'recompute_returncode')}))
    return 0 if 'error' not in result else 1


if __name__ == '__main__':
    sys.exit(main())
