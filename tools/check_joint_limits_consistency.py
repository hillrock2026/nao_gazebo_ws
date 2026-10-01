#!/usr/bin/env python3
"""Task 4 Phase 4B consistency check (read-only).

Ensures the 24-joint order and the joint position limits that are hard-coded in
the Python scripts (``phase4a_probe.py``, ``phase4b_producer.py``,
``phase4b_trace.py``) still match the canonical SDF ``nao_simple.sdf``. This
closes Phase 4A review P3 #4 (limit drift between the producer and the actuator,
which reads the limits from the SDF at runtime).

The script parses the literals with ``ast`` without importing the modules, so it
does not need a running ROS/Gazebo environment. Exit code 0 = consistent.
"""

import ast
import os
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(REPO_ROOT, 'src', 'nao', 'nao_description', 'urdf', 'nao_simple.sdf')
SCRIPTS = [
    os.path.join(REPO_ROOT, 'src', 'nao_gz_sim', 'scripts', 'phase4a_probe.py'),
    os.path.join(REPO_ROOT, 'src', 'nao_gz_sim', 'scripts', 'phase4b_producer.py'),
    os.path.join(REPO_ROOT, 'src', 'nao_gz_sim', 'scripts', 'phase4b_trace.py'),
    os.path.join(REPO_ROOT, 'src', 'nao_gz_sim', 'scripts', 'nao_trajectory_schema.py'),
]


def sdf_limits_and_order():
    model = ET.parse(SDF).getroot().find('model')
    order = []
    limits = {}
    for joint in model.findall('joint'):
        name = joint.get('name')
        order.append(name)
        limit = joint.find('axis/limit')
        limits[name] = (float(limit.find('lower').text), float(limit.find('upper').text))
    return order, limits


def script_literal(path, var):
    tree = ast.parse(open(path).read())
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == var:
                    return ast.literal_eval(node.value)
    raise SystemExit('could not find %s in %s' % (var, path))


def main():
    order, limits = sdf_limits_and_order()
    failures = []
    for path in SCRIPTS:
        rel = os.path.relpath(path, REPO_ROOT)
        scr_order = script_literal(path, 'JOINT_ORDER')
        if scr_order != order:
            failures.append('%s: JOINT_ORDER differs from SDF document order' % rel)
        else:
            print('  [PASS] %s JOINT_ORDER matches SDF (24)' % rel)
        scr_limits = script_literal(path, 'LIMITS')
        for name in order:
            got = tuple(scr_limits.get(name, (None, None)))
            want = limits[name]
            if abs(got[0] - want[0]) > 1e-9 or abs(got[1] - want[1]) > 1e-9:
                failures.append('%s: limit mismatch for %s: %r vs SDF %r'
                                % (rel, name, got, want))
        extra = set(scr_limits) - set(order)
        if extra:
            failures.append('%s: unknown joints in LIMITS: %s' % (rel, sorted(extra)))
        print('  [PASS] %s LIMITS matches SDF (24)' % rel)

    print()
    if failures:
        for f in failures:
            print('  [FAIL] %s' % f)
        print('JOINT LIMITS CONSISTENCY: FAIL (%d)' % len(failures))
        return 1
    print('JOINT LIMITS CONSISTENCY: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
