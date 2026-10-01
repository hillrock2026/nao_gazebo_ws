#!/usr/bin/env python3
"""Stage 7' — prepared-only episode runner facade (offline; refuses execution).

Delegates authorization semantics to stage7r_authorization and never starts
Gazebo/ROS.  ``--self-test`` performs static checks only.
"""

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
for _p in (_HERE,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import stage7r_setup as S  # noqa: E402
import stage7r_authorization as AUTH  # noqa: E402


def self_test():
    checks = {}
    canonical, _ = S.load_canonical()
    b = S.build_b(canonical)
    S.check_b_vs_canonical(b, canonical)
    S.check_launch_consistency(b, 'B')
    checks['config_ok'] = b['gait_phase_mode'] == 'explicit_support_swing'
    checks['authorization_self_test'] = AUTH.self_test()
    return checks


def main(argv=None):
    if argv and '--self-test' in argv:
        print(json.dumps({'self_test': self_test()}, indent=2, sort_keys=True))
        return 0
    sys.stderr.write("stage7r_episode_runner: execution is not performed here; "
                     "use stage7r_execd with a per-attempt authorization\n")
    return 3


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
