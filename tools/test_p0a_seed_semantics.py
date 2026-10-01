#!/usr/bin/env python3
"""P0-a P1 seed-semantics fixtures (offline; no Gazebo started).

Proves that:
  * ``gz_seed`` is declared in the launch file with an empty default, and the
    ``--seed`` flag is only appended when it is set (so the default command is
    byte-identical to the historical one);
  * the runner's seed is a *replicate label*, while the real physics seed is the
    separately-forwarded ``gz_seed:=<n>``;
  * GZ_PARTITION / ROS_DOMAIN_ID are communication isolation, not physics seeds.

Usage: python3 tools/test_p0a_seed_semantics.py [--json-out F]
"""

import argparse
import ast
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import run_p0a_gait_candidate as R  # noqa: E402

LAUNCH = os.path.join(WS, 'src', 'nao_gz_sim', 'launch', 'nao_sim.launch.py')
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def _call_name(node):
    if isinstance(node, ast.Call):
        f = node.func
        if isinstance(f, ast.Name):
            return f.id
        if isinstance(f, ast.Attribute):
            return f.attr
    return None


def _kw(node, name):
    for kw in node.keywords:
        if kw.arg == name:
            return kw.value
    return None


def _const(node):
    return node.value if isinstance(node, ast.Constant) else None


def static_checks():
    tree = ast.parse(open(LAUNCH).read())
    declared = None
    gz_fn = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_gz_sim_actions':
            gz_fn = node
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _call_name(node) == 'DeclareLaunchArgument':
            name = _const(node.args[0]) if node.args else _const(_kw(node, 'name'))
            if name == 'gz_seed':
                declared = _const(_kw(node, 'default_value'))
    check('gz_seed_declared_empty_default', declared == '', declared)
    check('gz_sim_actions_defined', gz_fn is not None)
    if gz_fn is not None:
        src = ast.dump(gz_fn)
        check('gz_sim_actions_branches_on_seed',
              "attr='gz_seed'" in src or "'gz_seed'" in src)
        check('gz_sim_actions_uses_dash_seed', "'--seed'" in src)


def dynamic_gz_checks():
    """Build the launch description and execute the gz-sim OpaqueFunction."""
    try:
        import importlib.util
        import launch
        from launch.utilities import (perform_substitutions,
                                      normalize_to_list_of_substitutions)
    except Exception as exc:  # pragma: no cover
        check('dynamic_launch_import', False, '%s' % type(exc).__name__)
        return
    spec = importlib.util.spec_from_file_location('nao_sim_launch_seed', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    def gz_cmds(overrides):
        ld = mod.generate_launch_description()
        ctx = launch.LaunchContext()
        for e in ld.entities:
            if type(e).__name__ == 'DeclareLaunchArgument':
                subs = normalize_to_list_of_substitutions(e.default_value)
                ctx.launch_configurations[e.name] = perform_substitutions(
                    launch.LaunchContext(), subs)
        ctx.launch_configurations.update(overrides)
        cmds = []
        for e in ld.entities:
            if type(e).__name__ == 'OpaqueFunction':
                actions = e.execute(ctx)
                for a in (actions or []):
                    raw = getattr(a, 'cmd', None)
                    if not raw:
                        continue
                    performed = [perform_substitutions(
                        ctx, normalize_to_list_of_substitutions(x))
                        if not isinstance(x, list) else perform_substitutions(ctx, x)
                        for x in raw]
                    strs = [str(c) for c in performed]
                    if 'sim' in strs and '--physics-engine' in strs:
                        cmds.append(strs)
        return cmds

    none_cmds = gz_cmds({})
    seeded_cmds = gz_cmds({'gz_seed': '12345'})
    check('dynamic_gz_actions_found', len(none_cmds) >= 1, none_cmds)
    check('dynamic_default_no_seed',
          none_cmds and all('--seed' not in c for c in none_cmds), none_cmds)
    check('dynamic_seeded_has_seed',
          seeded_cmds and all('--seed' in c for c in seeded_cmds), seeded_cmds)
    check('dynamic_seed_value_forwarded',
          all(c[c.index('--seed') + 1] == '12345'
              for c in seeded_cmds if '--seed' in c), seeded_cmds)


def runner_semantics():
    cfg = {'step_length_m': 0.02, 'gait_phase_mode': 'explicit_support_swing'}
    cmd_none = R.build_launch_command(cfg, '/tmp/t', 17.5, gz_seed=None)
    cmd_seed = R.build_launch_command(cfg, '/tmp/t', 17.5, gz_seed=999)
    check('runner_no_physics_seed_by_default',
          not any('gz_seed:=' in c for c in cmd_none))
    check('runner_forwards_physics_seed',
          'gz_seed:=999' in cmd_seed, cmd_seed)
    check('runner_maps_phase_mode',
          'phase4b_gait_phase_mode:=explicit_support_swing' in cmd_seed, cmd_seed)
    meta_none = R.seed_metadata('cand', 101, None)
    meta_seed = R.seed_metadata('cand', 101, 999)
    check('metadata_replicate_only', meta_none['physics_seed_controlled'] is False)
    check('metadata_physics_seed_set', meta_seed['physics_seed_controlled'] is True
          and meta_seed['physics_seed'] == 999)
    check('metadata_partition_is_not_seed',
          R.partition_id('cand', 101) == 'p0a_cand_101'
          and '999' not in R.partition_id('cand', 101))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a P1 seed-semantics fixtures')
    static_checks()
    dynamic_gz_checks()
    runner_semantics()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nseed semantics fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
