#!/usr/bin/env python3
"""Stage 7 v4 — unified, auditable ROS/Gazebo environment bootstrap (F-01 fix).

Round-3 (fingerprint binding) repairs:
  * R2-03: fingerprint list-keys are normalised (split -> drop empty -> dedup ->
    sort -> join) so ``fingerprint_hash`` is reproducible across shell sessions;
  * R2-04: fingerprint covers 11 required keys incl. ``nao_gz_sim_prefix`` and
    ``python_executable`` (+ ``python_version``), binding the rclpy ABI source.

Builds ONE environment (source ROS + workspace setup, filter conda, drop
PYTHONHOME/CONDA_*) used identically by the executor preflight and the delegated
launch.  Never starts Gazebo itself.
"""

import hashlib
import json
import os
import shutil
import subprocess

FINGERPRINT_VERSION = 'stage7r_env/2'

FINGERPRINT_LIST_KEYS = (
    'PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
    'GZ_SIM_RESOURCE_PATH', 'GZ_SIM_SYSTEM_PLUGIN_PATH',
)
FINGERPRINT_SCALAR_KEYS = (
    'ROS_DISTRO', 'STAGE7_BOOTSTRAP_ROS_SETUP', 'STAGE7_BOOTSTRAP_WORKSPACE_SETUP',
)
FINGERPRINT_DERIVED_KEYS = ('nao_gz_sim_prefix', 'python_executable', 'python_version')
FINGERPRINT_KEYS = FINGERPRINT_LIST_KEYS + FINGERPRINT_SCALAR_KEYS + FINGERPRINT_DERIVED_KEYS


class EnvironmentError2(RuntimeError):
    """Fail-closed environment error (distinct from builtin EnvironmentError)."""


def bootstrap_environment(workspace, ros_setup='/opt/ros/jazzy/setup.bash',
                          workspace_setup=None, extra_env=None):
    workspace = os.path.abspath(workspace)
    workspace_setup = workspace_setup or os.path.join(workspace, 'install', 'setup.bash')
    for path in (ros_setup, workspace_setup):
        if not os.path.isfile(path):
            raise EnvironmentError2('bootstrap_file_missing:%s' % path)
    command = 'set -e; source "$1"; source "$2"; env -0'
    proc = subprocess.run(['bash', '-lc', command, 'bootstrap', ros_setup, workspace_setup],
                          cwd=workspace, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          check=False)
    if proc.returncode != 0:
        raise EnvironmentError2('bootstrap_source_failed:%s'
                                % proc.stderr.decode('utf-8', errors='replace').strip())
    env = {}
    for item in proc.stdout.split(b'\0'):
        if b'=' in item:
            key, value = item.split(b'=', 1)
            env[key.decode()] = value.decode(errors='replace')
    env['STAGE7_BOOTSTRAP_ROS_SETUP'] = os.path.abspath(ros_setup)
    env['STAGE7_BOOTSTRAP_WORKSPACE_SETUP'] = os.path.abspath(workspace_setup)
    env['PATH'] = os.pathsep.join(
        entry for entry in env.get('PATH', '').split(os.pathsep)
        if 'anaconda' not in entry and 'condabin' not in entry)
    env.pop('CONDA_PREFIX', None)
    env.pop('CONDA_DEFAULT_ENV', None)
    env.pop('PYTHONHOME', None)
    seen, dedup = set(), []
    for entry in env.get('PATH', '').split(os.pathsep):
        if entry and entry not in seen:
            seen.add(entry)
            dedup.append(entry)
    env['PATH'] = os.pathsep.join(dedup)
    if extra_env:
        env.update(extra_env)
    return env


def _norm_list(value):
    """R2-03 normalisation for list-like env keys: split, drop empty, dedup, sort."""
    parts = [p for p in (value or '').split(os.pathsep) if p]
    return os.pathsep.join(sorted(set(parts)))


def _which(path_value, name):
    for directory in (path_value or '').split(os.pathsep):
        candidate = os.path.join(directory, name)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None


def _nao_gz_sim_prefix(env):
    for prefix in _norm_list(env.get('AMENT_PREFIX_PATH', '')).split(os.pathsep):
        if prefix and os.path.isdir(os.path.join(prefix, 'share', 'nao_gz_sim')):
            return prefix
    return ''


def _python_version(executable):
    if not executable:
        return ''
    try:
        proc = subprocess.run([executable, '--version'], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=20, check=False)
        out = (proc.stdout or '').strip()
        return out.splitlines()[0] if out else ''
    except (OSError, subprocess.SubprocessError):
        return ''


def _relevant_dirs(env):
    """Directories that actually matter for execution (session-independent)."""
    keep = set()
    for name in ('ros2', 'gz', 'python3'):
        exe = _which(env.get('PATH', ''), name)
        if exe:
            keep.add(os.path.dirname(exe))
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for prefix in _norm_list(env.get(key, '')).split(os.pathsep):
            if prefix and os.path.isdir(prefix):
                keep.add(prefix)
                keep.add(os.path.join(prefix, 'bin'))
    return keep


def _canonical_path(env):
    """PATH restricted to execution-relevant, existing directories.

    Makes the fingerprint insensitive to caller-injected PATH noise (e.g. a
    prepended /tmp dir) and to inherited interactive-shell dirs, while the actual
    boot env used for launch is left byte-for-byte unchanged.
    """
    relevant = _relevant_dirs(env)
    entries = [p for p in (env.get('PATH') or '').split(os.pathsep)
               if p and os.path.isdir(p) and p in relevant]
    return os.pathsep.join(sorted(set(entries)))


def environment_fingerprint(env):
    """Deterministic fingerprint; list keys normalised so hash is session-stable."""
    fp = {}
    for key in FINGERPRINT_LIST_KEYS:
        fp[key] = _canonical_path(env) if key == 'PATH' else _norm_list(env.get(key, ''))
    for key in FINGERPRINT_SCALAR_KEYS:
        fp[key] = env.get(key)
    fp['nao_gz_sim_prefix'] = _nao_gz_sim_prefix(env)
    executable = _which(env.get('PATH', ''), 'python3')
    fp['python_executable'] = executable
    fp['python_version'] = _python_version(executable)
    return fp


def fingerprint_hash(fingerprint):
    return hashlib.sha256(json.dumps(fingerprint, sort_keys=True).encode('utf-8')).hexdigest()


def _run(workspace, argv, env, timeout=40):
    return subprocess.run(argv, cwd=workspace, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, timeout=timeout,
                          check=False)


def verify_environment(workspace, env):
    """Fail-closed checks; returns a dict of probe results or raises EnvironmentError2."""
    checks = {}
    if not env.get('ROS_DISTRO'):
        raise EnvironmentError2('ros_distro_missing')
    ament = env.get('AMENT_PREFIX_PATH', '')
    if 'install' not in ament.replace('\\', '/'):
        raise EnvironmentError2('workspace_overlay_not_sourced')
    p = _run(workspace, ['ros2', '--help'], env)
    checks['ros2_available'] = p.returncode == 0
    if not checks['ros2_available']:
        raise EnvironmentError2('ros2_not_available')
    p = _run(workspace, ['ros2', 'pkg', 'prefix', 'nao_gz_sim'], env)
    prefix = (p.stdout or '').strip()
    checks['nao_gz_sim_package'] = (p.returncode == 0 and bool(prefix))
    if not checks['nao_gz_sim_package']:
        raise EnvironmentError2('nao_gz_sim_package_not_found')
    checks['nao_gz_sim_prefix'] = prefix
    p = _run(workspace, ['ros2', 'launch', 'nao_gz_sim', 'nao_sim.launch.py', '--show-args'], env)
    checks['launch_resolvable'] = p.returncode == 0
    if not checks['launch_resolvable']:
        raise EnvironmentError2('launch_not_resolvable')
    world = os.path.join(workspace, 'install', 'nao_gz_sim', 'share', 'nao_gz_sim',
                         'worlds', 'nao_world.sdf')
    if not os.path.isfile(world):
        raise EnvironmentError2('world_asset_missing:%s' % world)
    checks['world_asset'] = world
    p = _run(workspace, ['python3', '-c', 'import rclpy, sys; print(sys.version.split()[0])'], env)
    checks['python_abi_rclpy'] = (p.returncode == 0)
    if not checks['python_abi_rclpy']:
        raise EnvironmentError2('python_abi_rclpy_import_failed')
    checks['python_version'] = (p.stdout or '').strip()
    return checks


def self_test():
    import re
    checks = {}
    try:
        bootstrap_environment('/nonexistent-ws', workspace_setup='/nonexistent/setup.bash')
        checks['missing_files_raise'] = False
    except EnvironmentError2 as exc:
        checks['missing_files_raise'] = 'bootstrap_file_missing' in str(exc)
    fp = environment_fingerprint({'PATH': 'a:b', 'ROS_DISTRO': 'jazzy'})
    checks['fingerprint_hash_len'] = len(fingerprint_hash(fp)) == 64
    checks['fingerprint_version'] = FINGERPRINT_VERSION == 'stage7r_env/2'
    checks['fingerprint_keys_cover_required'] = all(
        k in FINGERPRINT_KEYS for k in ('nao_gz_sim_prefix', 'python_executable',
                                        'PATH', 'AMENT_PREFIX_PATH', 'ROS_DISTRO'))
    # R2-03 determinism: same relevant entries, different order -> identical hash
    base_env = {'AMENT_PREFIX_PATH': '/opt/ros/jazzy', 'COLCON_PREFIX_PATH': '/opt/ros/jazzy',
                'ROS_DISTRO': 'jazzy'}
    a = environment_fingerprint(dict(base_env, PATH='/usr/bin:/opt/ros/jazzy/bin:/usr/local/bin'))
    b = environment_fingerprint(dict(base_env, PATH='/usr/local/bin:/usr/bin:/opt/ros/jazzy/bin'))
    checks['list_norm_order_insensitive'] = fingerprint_hash(a) == fingerprint_hash(b)
    # R2-03: caller-injected noise dir is ignored
    noisy = environment_fingerprint(dict(base_env, PATH='/tmp/fp_noise:/usr/bin:/opt/ros/jazzy/bin'))
    clean = environment_fingerprint(dict(base_env, PATH='/usr/bin:/opt/ros/jazzy/bin'))
    checks['path_noise_ignored'] = fingerprint_hash(noisy) == fingerprint_hash(clean)
    ws = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
    e = bootstrap_environment(ws)
    checks['path_dedup'] = len(e['PATH'].split(os.pathsep)) == len(set(e['PATH'].split(os.pathsep)))
    checks['ament_has_install'] = 'install' in e.get('AMENT_PREFIX_PATH', '')
    efp = environment_fingerprint(e)
    checks['python_executable_nonempty'] = bool(efp.get('python_executable'))
    checks['python_version_shape'] = bool(re.match(r'^Python 3\.\d+\.\d+', efp.get('python_version') or ''))
    checks['nao_gz_sim_prefix_present'] = bool(efp.get('nao_gz_sim_prefix'))
    checks['verify_ok'] = bool(verify_environment(ws, e)['nao_gz_sim_package'])
    return checks


if __name__ == '__main__':
    import sys
    if '--self-test' in sys.argv:
        st = self_test()
        print(json.dumps({'self_test': st, 'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
    else:
        raise SystemExit('use --self-test')
