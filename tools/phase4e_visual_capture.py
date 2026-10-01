#!/usr/bin/env python3
"""Read-only helpers for the Task 4 Phase 4E-Visual Gazebo GUI demonstration.

Provides window discovery, foreign-window screenshots, GUI camera control via
gz-transport services, and region video recording via gstreamer. Nothing here
publishes ROS topics or changes physics; the only state changed is the GUI
camera / view toggles, which is explicitly allowed by the Phase 4E-Visual spec.

CLI (for interactive testing)::

    python3 tools/phase4e_visual_capture.py window
    python3 tools/phase4e_visual_capture.py shot OUT.png
    python3 tools/phase4e_visual_capture.py camera PX PY PZ TX TY TZ
    python3 tools/phase4e_visual_capture.py follow NAME_OR_none
    python3 tools/phase4e_visual_capture.py view collisions on|off
    python3 tools/phase4e_visual_capture.py record OUT.webm SECONDS
"""

import math
import os
import re
import signal
import subprocess
import sys
import time

import gi

gi.require_version('Gdk', '3.0')
gi.require_version('GdkX11', '3.0')
from gi.repository import Gdk, GdkX11  # noqa: E402

WINDOW_RE = re.compile(
    r'^\s*(0x[0-9a-fA-F]+)\s+"Gazebo Sim":\s+\("gz-sim-gui".*?'
    r'(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)')


_DEFAULT_ENV = None


def set_env(env):
    """Set the process env (incl. GZ_PARTITION / ROS_DOMAIN_ID) for gz services."""
    global _DEFAULT_ENV
    _DEFAULT_ENV = dict(env)


def _gz_env():
    if _DEFAULT_ENV is not None:
        return dict(_DEFAULT_ENV)
    return dict(os.environ)


def run(cmd, timeout=10, env=None):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                          env=env or _gz_env())


def find_gui_window():
    """Return {'xid', 'w', 'h', 'ax', 'ay'} for the gz-sim-gui client window."""
    out = run(['xwininfo', '-root', '-tree']).stdout
    for line in out.splitlines():
        m = WINDOW_RE.match(line)
        if m:
            return {'xid': m.group(1),
                    'w': int(m.group(2)), 'h': int(m.group(3)),
                    'ax': int(m.group(6)), 'ay': int(m.group(7))}
    return None


def wait_for_window(timeout=90.0, poll=1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        win = find_gui_window()
        if win and win['w'] > 100 and win['h'] > 100:
            return win
        time.sleep(poll)
    return None


def _black_fraction(pb):
    """Fraction of (near) pure-black pixels in the right/bottom band."""
    try:
        import numpy as np
    except ImportError:
        return 0.0
    data = pb.get_pixels()
    arr = np.frombuffer(data, dtype=np.uint8)
    arr = arr.reshape(pb.get_height(), pb.get_rowstride())
    arr = arr[:, :pb.get_width() * pb.get_n_channels()]
    h, w = pb.get_height(), pb.get_width()
    right = arr[:, (w * 3 * 3) // 4:w * 3]
    return float((right < 8).all(axis=1).mean()) if right.size else 0.0


def _grab_screen_png(path, timeout=20):
    """Grab one full-screen frame to PNG via gstreamer (reflects live content)."""
    proc = subprocess.run(
        ['gst-launch-1.0', '-q', 'ximagesrc', 'use-damage=false',
         'num-buffers=1', '!', 'videoconvert', '!', 'pngenc', '!',
         'filesink', 'location=%s' % path],
        capture_output=True, text=True, timeout=timeout, env=_gz_env())
    return proc.returncode == 0 and os.path.exists(path) and os.path.getsize(path) > 0


def shot(path, crop_to_window=True):
    """Capture the Gazebo GUI window content to ``path`` (PNG).

    Uses a full-screen gstreamer grab (which reflects the live OpenGL content)
    then crops to the current ``gz-sim-gui`` window geometry. Falls back to a
    direct foreign-window pixbuf capture if the grab fails.
    """
    win = find_gui_window()
    full = path + '.full.png'
    if _grab_screen_png(full):
        try:
            from PIL import Image
            img = Image.open(full).convert('RGB')
            if crop_to_window and win is not None:
                x0 = max(0, win['ax'])
                y0 = max(0, win['ay'])
                x1 = min(img.width, win['ax'] + win['w'])
                y1 = min(img.height, win['ay'] + win['h'])
                img = img.crop((x0, y0, x1, y1))
            img.save(path)
        finally:
            try:
                os.remove(full)
            except OSError:
                pass
        return {'w': img.width, 'h': img.height,
                'window': win, 'method': 'gstreamer'}
    if win is None:
        raise RuntimeError('gz-sim-gui window not found')
    display = Gdk.Display.get_default()
    gwin = GdkX11.X11Window.foreign_new_for_display(display, int(win['xid'], 16))
    pb = Gdk.pixbuf_get_from_window(gwin, 0, 0, gwin.get_width(), gwin.get_height())
    if pb is None:
        raise RuntimeError('pixbuf capture failed')
    pb.savev(path, 'png', [], [])
    return {'w': pb.get_width(), 'h': pb.get_height(),
            'window': win, 'method': 'pixbuf'}


def quat_lookat(p, t):
    f = [t[i] - p[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in f))
    f = [v / n for v in f]
    up = [0.0, 0.0, 1.0]
    if abs(sum(f[i] * up[i] for i in range(3))) > 0.999:
        up = [0.0, 1.0, 0.0]
    z = [up[i] - f[i] * sum(up[j] * f[j] for j in range(3)) for i in range(3)]
    nz = math.sqrt(sum(v * v for v in z))
    z = [v / nz for v in z]
    y = [z[1] * f[2] - z[2] * f[1], z[2] * f[0] - z[0] * f[2],
         z[0] * f[1] - z[1] * f[0]]
    m = [[f[0], y[0], z[0]], [f[1], y[1], z[1]], [f[2], y[2], z[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        yq = (m[0][2] - m[2][0]) / s
        zq = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        yq = (m[0][1] + m[1][0]) / s
        zq = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        yq = 0.25 * s
        zq = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        yq = (m[1][2] + m[2][1]) / s
        zq = 0.25 * s
    return w, x, yq, zq


def move_camera(pos, look):
    w, x, y, z = quat_lookat(pos, look)
    req = ('pose: {position: {x: %.5f, y: %.5f, z: %.5f}, '
           'orientation: {x: %.7f, y: %.7f, z: %.7f, w: %.7f}}'
           % (pos[0], pos[1], pos[2], x, y, z, w))
    return run(['gz', 'service', '-s', '/gui/move_to/pose',
                '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '4000', '--req', req], timeout=15)


def follow(name):
    return run(['gz', 'service', '-s', '/gui/follow',
                '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '4000', '--req', 'data: "%s"' % name], timeout=15)


def view_flag(service, on):
    value = 'true' if on else 'false'
    return run(['gz', 'service', '-s', service,
                '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '4000', '--req', 'data: "%s"' % value], timeout=15)


def start_record(out_path, fps=15, region=None):
    """Record to webm.  ``region`` = (x, y, ex, ey); None records the screen."""
    cmd = ['gst-launch-1.0', '-e', '-q', 'ximagesrc', 'use-damage=false']
    if region is not None:
        x, y, ex, ey = region
        cmd += ['startx=%d' % x, 'starty=%d' % y,
                'endx=%d' % ex, 'endy=%d' % ey]
    cmd += ['!', 'videoconvert', '!',
            'videorate', '!', 'video/x-raw,framerate=%d/1' % fps, '!',
            'vp8enc', 'deadline=1', 'cpu-used=8', 'keyframe-max-dist=30', '!',
            'webmmux', '!', 'filesink', 'location=%s' % out_path]
    log = open(out_path + '.gst.log', 'w')
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    return proc, {'fps': fps, 'region': region, 'cmd': cmd,
                  'log': out_path + '.gst.log'}


def stop_record(proc, timeout=20):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


def convert_to_mp4(webm, mp4, fps=15):
    """Re-encode a recorded webm to mp4 (mp4v) with OpenCV."""
    import cv2
    cap = cv2.VideoCapture(webm)
    if not cap.isOpened():
        return False
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(mp4, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    ok = writer.isOpened()
    n = 0
    while ok:
        ret, frame = cap.read()
        if not ret:
            break
        writer.write(frame)
        n += 1
    cap.release()
    writer.release()
    return ok and n > 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    cmd = sys.argv[1]
    if cmd == 'window':
        print(find_gui_window())
    elif cmd == 'shot':
        print(shot(sys.argv[2]))
    elif cmd == 'camera':
        pos = [float(a) for a in sys.argv[2:5]]
        look = [float(a) for a in sys.argv[5:8]]
        print(move_camera(pos, look).stdout.strip())
    elif cmd == 'follow':
        print(follow('' if sys.argv[2] == 'none' else sys.argv[2]).stdout.strip())
    elif cmd == 'view':
        print(view_flag('/gui/view/%s' % sys.argv[2], sys.argv[3] == 'on').stdout.strip())
    elif cmd == 'record':
        proc, info = start_record(sys.argv[2], fps=int(sys.argv[4]) if len(sys.argv) > 4 else 15)
        print(info)
        time.sleep(float(sys.argv[3]))
        stop_record(proc)
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
