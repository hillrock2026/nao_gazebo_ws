#!/usr/bin/env python3
"""P0-a / Stage-0 machine-readable environment capture (offline, read-only).

Records the full interpreter + ROS runtime so a result can be reproduced:

  * Python version / executable / pip version;
  * key Python distributions and versions;
  * ROS distribution, ROS/AMENT/COLCON/GZ environment variables;
  * ROS workspace, colcon version;
  * OS, kernel, architecture, libc;
  * git commit/hash (and an explicit ``is_git_repo: false`` fallback);
  * the run command, working directory, UTC+local time and timezone;
  * selected environment variables with secrets redacted.

Secrets are never emitted: any variable whose name matches token/password/
secret/key/credential/cookie/aws/google/private is replaced with ``"<redacted>"``.

Usage:
  python3 tools/p0a_env_capture.py --out env.json [--command "python3 ..."]
"""

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time

if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p0a_output_guard as OG  # noqa: E402

TOOL_VERSION = 'p0a_env_capture:2.0'
KEY_PACKAGES = ('numpy', 'scipy', 'matplotlib', 'pytest', 'launch', 'rclpy',
                'ros2cli', 'rosdistro', 'catkin_pkg', 'empy', 'lark')
SECRET_PATTERN = re.compile(
    r'(token|password|passwd|secret|credential|cookie|private|api[_-]?key|'
    r'access[_-]?key|aws_|google_|ssh_|^GNUPG)', re.IGNORECASE)
ENV_PREFIXES = ('ROS_', 'AMENT_', 'COLCON_', 'GAZEBO_', 'GZ_', 'RMW_', 'PYTHON')


def _redact(name, value):
    if SECRET_PATTERN.search(name):
        return '<redacted>'
    return value


def pip_version():
    try:
        out = subprocess.run([sys.executable, '-m', 'pip', '--version'],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True, timeout=30)
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def package_versions():
    try:
        from importlib import metadata
    except ImportError:  # pragma: no cover - py<3.8
        return {}
    out = {}
    for name in KEY_PACKAGES:
        try:
            out[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            out[name] = 'missing'
        except Exception:
            out[name] = 'unknown'
    return out


def git_info(workspace):
    def run(args):
        try:
            proc = subprocess.run(args, cwd=workspace, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, text=True,
                                  timeout=30)
            return proc.stdout.strip() if proc.returncode == 0 else None
        except (OSError, subprocess.SubprocessError):
            return None
    commit = run(['git', 'rev-parse', 'HEAD'])
    if commit:
        return {'is_git_repo': True, 'commit': commit,
                'branch': run(['git', 'rev-parse', '--abbrev-ref', 'HEAD']),
                'dirty': bool(run(['git', 'status', '--porcelain']))}
    return {'is_git_repo': False,
            'note': 'workspace is not a git repository; file SHA-256 hashes '
                    'are the provenance tracker'}


def colcon_version():
    return probe_tool([[sys.executable, '-m', 'pip', 'show', 'colcon-core'],
                       ['colcon', 'version-check']],
                      parse=_version_from_pip_show)


def _version_from_pip_show(text):
    m = re.search(r'^Version:\s*(\S+)', text or '', re.MULTILINE)
    return m.group(1) if m else None


def _first_version(text):
    m = re.search(r'(\d+\.\d+[0-9A-Za-z.\-+]*)', text or '')
    return m.group(1) if m else ((text or '').strip().splitlines() or [None])[0]


def probe_tool(commands, parse=None):
    """Probe a tool with an ordered list of argv candidates.

    Always returns a structured result; never fabricates a version:
    ``{'available': bool, 'version': str|None, 'command': list|None,
       'attempts': [{command, available, exit_code?, version?,
                     stdout_head?, stderr_summary?, error?}]}``.
    """
    attempts = []
    for argv in commands:
        exe = shutil.which(argv[0])
        if exe is None:
            attempts.append({'command': argv, 'available': False,
                             'note': 'executable not found on PATH'})
            continue
        try:
            proc = subprocess.run([exe] + list(argv[1:]), stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True, timeout=30)
        except (OSError, subprocess.SubprocessError) as exc:
            attempts.append({'command': argv, 'available': True,
                             'error': type(exc).__name__})
            continue
        out = (proc.stdout or '').strip()
        err = (proc.stderr or '').strip()
        entry = {'command': argv, 'available': True,
                 'exit_code': proc.returncode}
        if proc.returncode == 0:
            version = parse(out) if parse else (out.splitlines()[0] if out else None)
            entry['version'] = version
            entry['stdout_head'] = out[:200] or None
            attempts.append(entry)
            if version:
                return {'available': True, 'version': version,
                        'command': argv, 'attempts': attempts}
        else:
            entry['stdout_head'] = out[:200] or None
            entry['stderr_summary'] = err[:200] or None
            attempts.append(entry)
    return {'available': any(a.get('available') for a in attempts),
            'version': None, 'command': None, 'attempts': attempts,
            'note': 'unavailable'}


def gazebo_version():
    return probe_tool([['gz', 'sim', '--version'],
                       ['gz', '--version'],
                       ['gazebo', '--version']], parse=_first_version)


def catkin_version():
    return probe_tool([['catkin_make', '--version'],
                       ['catkin', '--version'],
                       ['catkin_make_isolated', '--version']],
                      parse=_first_version)


def _libc():
    try:
        return platform.libc_ver()[1] or None
    except Exception:
        return None


def capture(command=None, workspace=None):
    workspace = workspace or os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))
    now = time.time()
    env_vars = {}
    for name in sorted(os.environ):
        if name.startswith(ENV_PREFIXES) or SECRET_PATTERN.search(name):
            env_vars[name] = _redact(name, os.environ[name])
    return {
        'artifact': 'Stage-0 environment capture',
        'tool': TOOL_VERSION,
        'captured_at_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ',
                                         time.gmtime(now)),
        'captured_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z',
                                           time.localtime(now)),
        'epoch_sec': now,
        'timezone': os.environ.get('TZ') or _timezone_name(),
        'python': {
            'version': sys.version,
            'version_info': list(sys.version_info[:3]),
            'executable': sys.executable,
            'implementation': platform.python_implementation(),
            'pip': pip_version(),
            'packages': package_versions(),
        },
        'ros': {
            'distro': os.environ.get('ROS_DISTRO'),
            'distribution': os.environ.get('ROS_DISTRO'),
            'version': os.environ.get('ROS_VERSION'),
            'colcon': colcon_version(),
            'catkin': catkin_version(),
        },
        'gazebo': gazebo_version(),
        'platform': {
            'system': platform.system(),
            'release': platform.release(),
            'machine': platform.machine(),
            'processor': platform.processor(),
            'libc': _libc(),
            'os': _os_pretty(),
            'kernel': platform.uname().release,
            'architecture': platform.machine(),
        },
        'workspace': {
            'path': workspace,
            'cwd': os.getcwd(),
            'git': git_info(workspace),
        },
        'run': {
            'command': command or ' '.join(sys.argv),
            'argv': list(sys.argv),
        },
        'env': env_vars,
        'env_redaction_rule': ('names matching %s are replaced by <redacted>'
                               % SECRET_PATTERN.pattern),
    }


def _timezone_name():
    try:
        with open('/etc/timezone', 'r', encoding='utf-8') as fh:
            return fh.read().strip() or None
    except OSError:
        return time.tzname[0] if time.tzname else None


def _os_pretty():
    try:
        with open('/etc/os-release', 'r', encoding='utf-8') as fh:
            for line in fh:
                if line.startswith('PRETTY_NAME='):
                    return line.split('=', 1)[1].strip().strip('"')
    except OSError:
        pass
    return platform.platform()


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--command', default=None,
                    help='command string that produced the result set')
    ap.add_argument('--workspace', default=None)
    args = ap.parse_args(argv)
    data = capture(args.command, args.workspace)
    try:
        OG.write_json(args.out, data, allow_existing=False)
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_env_capture: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    print('wrote %s' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
