#!/usr/bin/env python3
"""Gate B3.0 F-7 launch wiring fixtures (no Gazebo, no ROS node started).

Verifies that `phase4b_foot_pitch_swing_rad` is declared with default 0.10 and
is forwarded as a float ParameterValue to the phase4b_producer parameter
`foot_pitch_swing_rad`.

Two independent checks:
  * a static AST parse of src/nao_gz_sim/launch/nao_sim.launch.py (always run);
  * a live build of the launch description (imports the launch module and
    evaluates the producer parameter dict; no process is launched). This needs
    /opt/ros/jazzy and install/setup.bash sourced.

Usage: bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash \
       && python3.12 tools/test_task4_phase4e_b3_launch.py'
"""

import ast
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
LAUNCH = os.path.join(WS, 'src', 'nao_gz_sim', 'launch', 'nao_sim.launch.py')

RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %s' % (detail,)))


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
    param_value = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        cname = _call_name(node)
        if cname == 'DeclareLaunchArgument':
            name_node = _kw(node, 'name') or (node.args[0] if node.args else None)
            n = _const(name_node)
            if n == 'phase4b_foot_pitch_swing_rad':
                declared = _const(_kw(node, 'default_value'))
        if cname == 'ParameterValue':
            conf = node.args[0] if node.args else None
            if (_call_name(conf) == 'LaunchConfiguration' and conf.args
                    and _const(conf.args[0]) == 'phase4b_foot_pitch_swing_rad'):
                vt = _kw(node, 'value_type')
                param_value = {
                    'value_type': vt.id if isinstance(vt, ast.Name) else None,
                    'launch_config': _const(conf.args[0])}
    check('static_arg_declared_default_0.10', declared == '0.10', declared)
    check('static_paramvalue_float',
          param_value is not None and param_value['value_type'] == 'float',
          param_value)

    # confirm the ParameterValue sits under the producer param key
    found_key = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for k, v in zip(node.keys, node.values):
                if _const(k) == 'foot_pitch_swing_rad' and _call_name(v) == 'ParameterValue':
                    found_key = True
    check('static_producer_param_key_wired', found_key)


def dynamic_checks():
    try:
        import importlib.util
        import launch
        from launch.utilities import (perform_substitutions,
                                      normalize_to_list_of_substitutions)
        import launch_ros.utilities as lu
    except Exception as exc:  # pragma: no cover
        check('dynamic_launch_import', False, '%s: %s' % (type(exc).__name__, exc))
        return False

    spec = importlib.util.spec_from_file_location('nao_sim_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    def producer_params(overrides):
        ld = mod.generate_launch_description()
        node = None
        for e in ld.entities:
            if type(e).__name__ == 'Node' and getattr(
                    e, 'node_executable', None) == 'phase4b_producer.py':
                node = e
                break
        if node is None:
            return None
        tmp = launch.LaunchContext()
        ctx = launch.LaunchContext()
        for e in ld.entities:
            if type(e).__name__ == 'DeclareLaunchArgument':
                subs = normalize_to_list_of_substitutions(e.default_value)
                ctx.launch_configurations[e.name] = perform_substitutions(tmp, subs)
        ctx.launch_configurations.update(overrides)
        return lu.evaluate_parameters(ctx, list(node._Node__parameters))[0]

    params = producer_params({})
    check('dynamic_default_0.10_float',
          params is not None and params.get('foot_pitch_swing_rad') == 0.10
          and isinstance(params.get('foot_pitch_swing_rad'), float),
          params.get('foot_pitch_swing_rad') if params else None)
    custom = producer_params({'phase4b_foot_pitch_swing_rad': '0.13'})
    check('dynamic_custom_0.13_float',
          custom is not None and custom.get('foot_pitch_swing_rad') == 0.13
          and isinstance(custom.get('foot_pitch_swing_rad'), float),
          custom.get('foot_pitch_swing_rad') if custom else None)
    return True


def main():
    print('F-7 launch wiring fixtures')
    static_checks()
    dynamic_checks()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nlaunch wiring fixtures: %d/%d passed' % (passed, total))
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
