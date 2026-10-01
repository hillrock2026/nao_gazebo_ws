#!/usr/bin/env python3
# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Pure, ROS-free explicit gait phase model (P0-a P1).

A single gait cycle is split into four mutually exclusive, gapless phases, in
cycle order::

    LEFT_SWING            phase in [0,               s)
    DS_TO_LEFT_SUPPORT    phase in [s,               0.5)
    RIGHT_SWING           phase in [0.5,             1 - ds/2)
    DS_TO_RIGHT_SUPPORT   phase in [1 - ds/2,        1)

with ``s = 0.5 - ds/2`` the per-leg swing fraction and ``ds`` the total
double-support fraction (``double_support_ratio``). The sum of the two double
support halves equals ``ds`` and the four intervals tile ``[0, 1)`` with no gap
or overlap for ``0 < ds < 0.6``.

This module defines only phase scheduling / role assignment and the
phase-aligned lateral sway *profile*; it performs no COM/ZMP control (that is
P2). It is deliberately free of ROS/Gazebo imports so both the Phase 4B
producer and the offline analysis/tests can share exactly one definition.
"""

import math

GAIT_PHASE_MODES = ('legacy', 'explicit_support_swing')
PHASE_NAMES = ('LEFT_SWING', 'DS_TO_LEFT_SUPPORT', 'RIGHT_SWING',
               'DS_TO_RIGHT_SUPPORT')
DS_RATIO_MIN = 0.0
DS_RATIO_MAX = 0.6


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def smoothstep(p):
    p = clamp(p, 0.0, 1.0)
    return p * p * (3.0 - 2.0 * p)


def swing_ratio(ds_ratio):
    """Per-leg swing fraction of the cycle, matching the producer geometry."""
    return 0.5 - ds_ratio / 2.0


def validate_ds_ratio(ds_ratio):
    if not (isinstance(ds_ratio, (int, float)) and not isinstance(ds_ratio, bool)
            and math.isfinite(float(ds_ratio))
            and DS_RATIO_MIN <= ds_ratio < DS_RATIO_MAX):
        raise ValueError('double_support_ratio out of range: %r' % (ds_ratio,))
    return float(ds_ratio)


def validate_phase_mode(mode):
    if mode not in GAIT_PHASE_MODES:
        raise ValueError('invalid gait_phase_mode: %r' % (mode,))
    return mode


def classify(phase, ds_ratio):
    """Return ``(name, progress, left_role, right_role)`` for a cycle phase.

    ``phase`` is taken modulo 1. ``progress`` is the normalized progress inside
    the phase, in ``[0, 1)``. Roles are ``'support'`` or ``'swing'``.
    """
    ds = validate_ds_ratio(ds_ratio)
    s = swing_ratio(ds)
    h = ds / 2.0
    ph = phase % 1.0
    if ph < s:
        return ('LEFT_SWING', ph / s, 'swing', 'support')
    if h <= 0.0:
        # No double support: phase in [s, 1) is the right swing.
        return ('RIGHT_SWING', (ph - s) / s, 'support', 'swing')
    if ph < 0.5:
        return ('DS_TO_LEFT_SUPPORT', (ph - s) / h, 'support', 'support')
    if ph < 1.0 - h:
        return ('RIGHT_SWING', (ph - 0.5) / s, 'support', 'swing')
    return ('DS_TO_RIGHT_SUPPORT', (ph - (1.0 - h)) / h, 'support', 'support')


def phase_name(phase, ds_ratio):
    return classify(phase, ds_ratio)[0]


def leg_foot_target(phase, side, ds_ratio, step, leg_ext, swing_h,
                    foot_pitch_swing, direction_sign=1.0):
    """Sagittal sole target ``(x, z, foot_pitch)`` relative to the hip.

    This is algebraically identical to the historical Phase 4B mapping (so the
    legacy foot trajectory is unchanged): each leg follows a swing arc lifted by
    ``swing_h`` then a straight stance line, 180 deg out of phase. The mapping is
    continuous at every phase boundary (swing end == stance start).
    """
    if side not in ('L', 'R'):
        raise ValueError('side must be L or R: %r' % (side,))
    s = swing_ratio(ds_ratio)
    offset = 0.0 if side == 'L' else 0.5
    phi = (phase + offset) % 1.0
    if phi < s:
        p = phi / s
        x = -step / 2.0 + step * smoothstep(p)
        z = -(leg_ext - swing_h * math.sin(math.pi * p))
        pitch = foot_pitch_swing * math.sin(math.pi * p)
    else:
        p = (phi - s) / (1.0 - s)
        x = step / 2.0 - step * p
        z = -leg_ext
        pitch = 0.0
    x *= direction_sign
    return x, z, pitch


def sway_profile(phase, ds_ratio):
    """Phase-aligned lateral sway profile in ``[-1, +1]``.

    ``+1`` means "shift the pelvis toward the right support foot" and ``-1``
    toward the left support foot. The profile holds at the support side through
    the whole opposite swing and crosses over smoothly during double support,
    so the weight transfer completes *before* the swing foot lifts (unlike the
    legacy ``-sin(2*pi*phase)``, which is still near zero when the swing
    starts).
    """
    ds = validate_ds_ratio(ds_ratio)
    name, progress, _, _ = classify(phase, ds)
    if name == 'LEFT_SWING':
        return 1.0
    if name == 'RIGHT_SWING':
        return -1.0
    if name == 'DS_TO_LEFT_SUPPORT':
        return 1.0 - 2.0 * smoothstep(progress)
    return -1.0 + 2.0 * smoothstep(progress)
