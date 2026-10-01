#!/usr/bin/env python3
"""Read-only reference validator and bounded cache for the frozen Task 4
``/joint_command`` contract (``docs/Task4_Trajectory_Schema.md`` v1.0.0).

Task 4 Phase 4D extracted the single shared implementation into
``src/nao_gz_sim/scripts/nao_trajectory_schema.py`` (installed with the package
so the collector and offline validators use the exact same rules). This module
is kept as a thin, API-compatible re-export so existing Phase 3 tooling keeps
working unchanged.

Frozen contract (Task4_Trajectory_Schema.md section 2.3 / 5.4 / 6):
  topic: /joint_command
  type:  sensor_msgs/msg/JointState
  semantics: 24-dimensional physical joint position target
  joint order: exactly the frozen /joint_states order (24 names)
  position unit: rad
  velocity[]: must be empty
  effort[]: must be empty
  header.frame_id: ""
  header.stamp: gazebo simulation time, non-zero, strictly increasing
  hold-last: command.stamp <= sample.timestamp and
             sample.timestamp - command.stamp <= 0.05 s
"""

import os
import sys

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'nao_gz_sim', 'scripts'))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from nao_trajectory_schema import (  # noqa: E402,F401
    JOINT_ORDER,
    LIMITS,
    REASON_BACK,
    REASON_DIM,
    REASON_DUP,
    REASON_EFF,
    REASON_FRAME,
    REASON_LIMIT,
    REASON_NEGATIVE,
    REASON_NONFINITE,
    REASON_ORDER,
    REASON_VEL,
    REASON_ZERO,
    REASONS,
    ValidationResult,
    CommandCache,
    CommandHistory,
    stamp_is_negative,
    stamp_is_zero,
    stamp_pair,
    stamp_to_sec,
    validate_command,
)

__all__ = [
    'JOINT_ORDER', 'LIMITS',
    'REASON_DIM', 'REASON_ORDER', 'REASON_VEL', 'REASON_EFF', 'REASON_FRAME',
    'REASON_ZERO', 'REASON_NEGATIVE', 'REASON_NONFINITE', 'REASON_LIMIT',
    'REASON_DUP', 'REASON_BACK',
    'REASONS', 'ValidationResult', 'CommandCache', 'CommandHistory',
    'stamp_pair', 'stamp_to_sec', 'stamp_is_negative', 'stamp_is_zero',
    'validate_command',
]
