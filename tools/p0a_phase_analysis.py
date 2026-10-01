#!/usr/bin/env python3
"""P0-a P1 per-phase / per-leg support-swing analysis (offline, deterministic).

Reads one recorded run (raw traces only) and reports, per explicit phase and per
leg, the evidence required by ``docs/task_p0a_gait_search/p1_support_swing/
P1_charter.md``:

  * phase sample counts / durations;
  * double-support both-contact rate;
  * during LEFT_SWING the right (support) contact rate, and vice versa;
  * swing-foot airborne fraction and max clearance;
  * per commanded support interval, the support-foot world x/y drift (slip
    proxy -- NOT a tangential force measurement);
  * zero-contact interval count / longest duration;
  * contact onset delay after each swing->support boundary;
  * the frozen locomotion metrics (via analyze_measured_gait);
  * explicit anomalies / not-measurable reasons.

Evidence-level discipline: bool contact only proves contact *timing*; base/pelvis
body pose is not a whole-body COM; support-foot drift is a slip proxy. None of
these establish ZMP stability or absence of tangential slip.

Usage:
  python3 tools/p0a_phase_analysis.py --run <run_dir> [--run ...] --out <json>
"""

import argparse
import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(os.path.dirname(_TOOLS), 'src', 'nao_gz_sim', 'scripts')
for _p in (_TOOLS, _SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import analyze_measured_gait as M  # noqa: E402
import phase4e_analysis as A  # noqa: E402
import nao_gait_phase as G  # noqa: E402

DEFAULT_SWING_CLEARANCE_M = 0.003
JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]
LEG_CHAIN = {
    'L': ((0.0, 0.707106, -0.707106), (0.0, 0.05, -0.085)),
    'R': ((0.0, 0.707106, 0.707106), (0.0, -0.05, -0.085)),
}


class PhaseAnalysisError(Exception):
    pass


class GateContractError(PhaseAnalysisError):
    """Raised when a frozen-gate file violates the v2 wrapper / flat contract."""


# Frozen v2 hard-gate keys (docs/task_p0a_gait_search/p1r_support_swing/
# frozen_gates_v2.json -> hard_gates).  ``epsilon_sec`` is optional.
REQUIRED_HARD_GATES = (
    'min_forward', 'max_lateral_ratio', 'max_yaw_deg', 'max_tilt_deg',
    'min_duration', 'max_missing', 'max_path_over_net', 'min_contact_alt',
    'min_support_ground', 'min_ds_both_ground', 'min_airborne',
    'min_clearance', 'max_support_drift_m',
)
OPTIONAL_HARD_GATES = ('epsilon_sec',)
# Gate files may be either the v2 wrapper (``{"version":..., "hard_gates":{...}}``)
# or the legacy flat form (the thresholds at top level).  Both are accepted and
# normalised to the flat mapping by ``load_gates``; unknown *wrapper* sections
# are ignored, unknown top-level keys are rejected so a typo cannot be silently
# read as "no threshold".
GATE_WRAPPER_KEY = 'hard_gates'
# Internal format metadata attached by ``load_gates``.  These are NEVER
# thresholds and are stripped before validation so they can neither be mistaken
# for unknown threshold keys nor exempt a mapping from validation.
GATE_META_KEYS = ('_gate_form', '_gate_source', '_gate_version')


def normalize_gates(data, source='<dict>'):
    """Normalise a parsed gate mapping (wrapper or flat) to flat thresholds.

    Raises :class:`GateContractError` for the same contract violations as
    :func:`load_gates`.  Idempotent: an already-normalised mapping is returned
    unchanged.  The ``_gate_form``/``_gate_source``/``_gate_version`` metadata
    keys are stripped first and never bypass validation; the returned mapping
    contains only thresholds (``REQUIRED_HARD_GATES`` + ``epsilon_sec``).
    """
    if isinstance(data, (str, bytes, os.PathLike)):
        return load_gates(data)
    if not isinstance(data, dict):
        raise GateContractError('gate spec must be a mapping: %r' % (source,))
    # ``_gate_form`` etc. are format metadata, not thresholds: strip them so the
    # unknown-key check cannot be bypassed and is not tripped by our own markers.
    cleaned = {k: v for k, v in data.items() if k not in GATE_META_KEYS}
    allowed_keys = set(REQUIRED_HARD_GATES) | set(OPTIONAL_HARD_GATES)
    if GATE_WRAPPER_KEY in cleaned:
        wrapper_keys = set(cleaned) - {GATE_WRAPPER_KEY, 'version', 'diagnostic',
                                       'onset_hard_requirement', 'preseed'}
        if wrapper_keys:
            raise GateContractError(
                'unexpected top-level keys in wrapper gate %s: %s'
                % (source, sorted(wrapper_keys)))
        hard = cleaned[GATE_WRAPPER_KEY]
        if not isinstance(hard, dict):
            raise GateContractError(
                '%r must be an object in %s' % (GATE_WRAPPER_KEY, source))
        form = 'wrapper'
    else:
        hard = cleaned
        form = 'flat'
    # Flat-schema symmetry: unknown threshold keys (typos) are rejected for both
    # the wrapper's ``hard_gates`` object and a top-level flat mapping.
    unknown = sorted(set(hard) - allowed_keys)
    if unknown:
        raise GateContractError(
            'unexpected threshold keys in %s gate %s: %s'
            % (form, source, unknown))
    missing = [k for k in REQUIRED_HARD_GATES if k not in hard]
    if missing:
        raise GateContractError(
            'gate spec %s (%s form) missing required thresholds: %s'
            % (source, form, sorted(missing)))
    for key, value in hard.items():
        if key in REQUIRED_HARD_GATES or key in OPTIONAL_HARD_GATES:
            if not M.finite(value):
                raise GateContractError(
                    'gate %r is not a finite number in %s: %r'
                    % (key, source, value))
    resolved = {k: hard[k] for k in REQUIRED_HARD_GATES}
    resolved['epsilon_sec'] = hard.get('epsilon_sec', 0.0)
    return resolved


def load_gates(source):
    """Load and strictly validate a frozen gate specification.

    ``source`` is a path (str/bytes/os.PathLike) to a JSON file.  Accepts the v2
    wrapper (``hard_gates`` nested) and the legacy flat mapping, returning an
    equivalent flat ``dict`` of thresholds.  Raises :class:`GateContractError`
    with an explicit message for missing files, malformed JSON, missing/typo'd
    keys, non-numeric values or a wrapper whose ``hard_gates`` is not an object.
    """
    path = os.fspath(source)
    if not os.path.isfile(path):
        raise GateContractError('gate file missing: %s' % path)
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            data = json.load(fh)
    except ValueError as exc:
        raise GateContractError('gate file is not valid JSON: %s (%s)'
                                % (path, exc))
    if not isinstance(data, dict):
        raise GateContractError('gate file must be a JSON object: %s' % path)
    resolved = normalize_gates(data, source=path)
    # Attach non-threshold format metadata for consumers (never used as a
    # threshold and stripped again by ``normalize_gates`` on re-entry).
    resolved['_gate_source'] = path
    resolved['_gate_form'] = 'wrapper' if GATE_WRAPPER_KEY in data else 'flat'
    resolved['_gate_version'] = (data.get('version')
                                 if GATE_WRAPPER_KEY in data else None)
    return resolved



def resolve_phase_mode(cfg, run_dir=None):
    """Return ``(mode, source)`` for a recorded run config.

    * an explicit non-empty ``gait_phase_mode`` that is a known mode is used;
    * a *missing* mode is only legal for the historical legacy schema, which is
      identified by the presence of the legacy ``gait_method`` key and the
      documented absence of the newer mode field.  It resolves to ``legacy``
      with source ``historical_legacy_schema`` (never a silent fallback);
    * an empty string, wrong type, unknown mode, or a config with neither
      ``gait_phase_mode`` nor ``gait_method`` raises :class:`PhaseAnalysisError`.
    """
    tag = '' if run_dir is None else ' in %s' % run_dir
    if 'gait_phase_mode' in cfg:
        mode = cfg['gait_phase_mode']
        if not isinstance(mode, str) or not mode:
            raise PhaseAnalysisError(
                'gait_phase_mode must be a non-empty string%s: %r'
                % (tag, mode))
        if mode not in G.GAIT_PHASE_MODES:
            raise PhaseAnalysisError(
                'invalid recorded gait_phase_mode%s: %r' % (tag, mode))
        return mode, 'explicit'
    if 'gait_method' in cfg:
        return 'legacy', 'historical_legacy_schema'
    raise PhaseAnalysisError(
        'gait_phase_mode missing and no legacy gait_method key%s; cannot '
        'establish a phase schedule' % tag)


# --------------------------------------------------------------------- linear alg
def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _axis_angle(axis, q):
    n = math.sqrt(sum(a * a for a in axis))
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = (a / n for a in axis)
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def _quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise PhaseAnalysisError('zero quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def sole_position_base(q, side):
    """Exact serial-chain FK of l_sole/r_sole in base_link (matches 4B/4C)."""
    yp, origin = LEG_CHAIN[side]
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    t = list(origin)
    R = _mm(R, _axis_angle(yp, q[side + 'HipYawPitch']))
    R = _mm(R, _axis_angle((1, 0, 0), q[side + 'HipRoll']))
    R = _mm(R, _axis_angle((0, 1, 0), q[side + 'HipPitch']))
    d = _mv(R, [0.0, 0.0, -0.100])
    t = [t[i] + d[i] for i in range(3)]
    R = _mm(R, _axis_angle((0, 1, 0), q[side + 'KneePitch']))
    d = _mv(R, [0.0, 0.0, -0.1029])
    t = [t[i] + d[i] for i in range(3)]
    R = _mm(R, _axis_angle((0, 1, 0), q[side + 'AnklePitch']))
    R = _mm(R, _axis_angle((1, 0, 0), q[side + 'AnkleRoll']))
    d = _mv(R, [0.0, 0.0, -0.04511])
    t = [t[i] + d[i] for i in range(3)]
    return t


# --------------------------------------------------------------------------- io
def _load_jsonl(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _nearest(series, t, key='t'):
    best = None
    for row in series:
        if row.get(key) is not None and row[key] <= t:
            best = row
        else:
            break
    return best


def _contact_state_fn(events):
    per = {'left': [], 'right': []}
    for t, side, value in events:
        per[side].append((t, value))

    def state(side, t):
        seq = per[side]
        val = False
        for ts, v in seq:
            if ts <= t:
                val = v
            else:
                break
        return val
    return state


CONTACT_ORDERING = {
    'rule': 'global sort by (t, side_rank, value_rank); side_rank '
            'left=0,right=1; value_rank false=0,true=1',
    'stable': True,
    'input_order_independent': True,
    'tie_break': 'content-based total order (no file/row/dict order)',
}
# Duplicate contact events: an *exact* duplicate shares (t, side, value) and is
# collapsed to one event (counted, never silently ignored).  Two events with the
# same (t, side) but a different value are a *conflict* and fail explicitly.
# Time-near duplicates (same side+value, 0 < |dt| <= tol) are reported but never
# removed; the default tolerance is 0.0 so only exact duplicates are collapsed.
DEFAULT_DUPLICATE_TIME_TOL_SEC = 0.0


def validate_contacts(trace_dir,
                      duplicate_time_tol=DEFAULT_DUPLICATE_TIME_TOL_SEC):
    """Load, validate, order and de-duplicate ``contact_trace.jsonl``.

    Returns ``(events, diagnostics)`` where ``events`` is the canonical,
    order-independent list of ``(t, side, value)`` tuples and ``diagnostics``
    records the ordering rule, duplicate statistics and near-duplicate count.
    Raises :class:`PhaseAnalysisError` on missing/empty trace, invalid field or
    a same-time conflicting value.
    """
    path = os.path.join(trace_dir, 'contact_trace.jsonl')
    if not os.path.isfile(path):
        raise PhaseAnalysisError('contact trace missing: %s' % path)
    rows = _load_jsonl(path)
    if not rows:
        raise PhaseAnalysisError('empty contact trace: %s' % path)
    raw = []
    for i, r in enumerate(rows):
        t = r.get('t')
        side = r.get('side')
        value = r.get('value')
        if not M.finite(t):
            raise PhaseAnalysisError('contact row %d invalid t in %s' % (i, path))
        if side not in ('left', 'right'):
            raise PhaseAnalysisError('contact row %d invalid side %r in %s'
                                     % (i, side, path))
        if not isinstance(value, bool):
            raise PhaseAnalysisError('contact row %d invalid value %r in %s'
                                     % (i, value, path))
        raw.append((float(t), side, value))

    # exact duplicates (same t, side, value) collapse to one; a same (t, side)
    # with a different value is an ambiguity and must not be silently resolved.
    seen = {}
    conflicts = []
    exact_duplicates = 0
    for t, side, value in raw:
        key = (t, side)
        if key in seen:
            if seen[key] == value:
                exact_duplicates += 1
            else:
                conflicts.append({'t': t, 'side': side,
                                  'first': seen[key], 'second': value})
        else:
            seen[key] = value
    if conflicts:
        raise PhaseAnalysisError(
            'conflicting contact values at identical time in %s: %r'
            % (path, conflicts))
    events = [(t, side, value) for (t, side), value in seen.items()]

    def side_rank(s):
        return 0 if s == 'left' else 1

    def key_of(e):
        return (e[0], side_rank(e[1]), 1 if e[2] else 0)

    events.sort(key=key_of)
    input_order_violations = sum(
        1 for a, b in zip(raw, raw[1:]) if key_of(a) > key_of(b))
    input_already_canonical = all(
        key_of(a) <= key_of(b) for a, b in zip(raw, raw[1:]))
    if duplicate_time_tol > 0:
        near = 0
        for a, b in zip(events, events[1:]):
            if (a[1] == b[1] and a[2] == b[2]
                    and 0.0 < abs(b[0] - a[0]) <= duplicate_time_tol):
                near += 1
    else:
        near = 0
    diagnostics = {
        'trace': path,
        'n_rows': len(raw),
        'n_unique_events': len(events),
        'n_exact_duplicates_removed': exact_duplicates,
        'n_conflicting_duplicates': 0,
        'n_near_duplicates_retained': near,
        'duplicate_key': '(t, side, value)',
        'near_duplicate_tol_sec': duplicate_time_tol,
        'near_duplicate_policy': 'retained (reported only; tol<=0 => none)',
        'ordering': CONTACT_ORDERING,
        'input_already_canonical': input_already_canonical,
        'n_input_order_violations': input_order_violations,
    }
    return events, diagnostics


def _validated_contacts(trace_dir):
    """Backwards-compatible wrapper returning only the canonical events."""
    events, _ = validate_contacts(trace_dir)
    return events



def _onset_delays(recs, cstate, boundaries, freq):
    """Contact-onset delay after each commanded swing->support boundary.

    ``boundaries`` maps side -> phase at which that side's support interval
    begins. Scans forward at most one cycle; an onset that never occurs inside
    the cycle is counted as ``n_missing`` (never silently dropped). If contact is
    already true at the boundary the delay is defined as 0.0.
    """
    period = 1.0 / freq
    delays = []
    missing = 0
    n_boundaries = 0
    for side, b in boundaries.items():
        side_l = 'left' if side == 'L' else 'right'
        for i in range(1, len(recs)):
            prev, cur = recs[i - 1], recs[i]
            if not (prev['phase'] < b <= cur['phase']):
                continue
            # A boundary too close to the window end cannot be evaluated within
            # one full cycle; exclude it rather than counting a spurious missing.
            if recs[-1]['t'] - cur['t'] < period:
                continue
            n_boundaries += 1
            if cstate(side_l, cur['t']):
                delays.append(0.0)
                continue
            onset = None
            for j in range(i, len(recs)):
                if recs[j]['t'] - cur['t'] > period:
                    break
                if cstate(side_l, recs[j]['t']):
                    onset = recs[j]['t']
                    break
            if onset is None:
                missing += 1
            else:
                delays.append(onset - cur['t'])
    return {'n_boundaries': n_boundaries, 'n_measured': len(delays),
            'n_missing': missing,
            'median_sec': _median(delays),
            'max_sec': max(delays) if delays else None}


def _intervals(mask, times):
    """Contiguous True runs as (i0, i1) index pairs."""
    out = []
    i = 0
    n = len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def _max_drift(series, i0, i1):
    xs = [series[k][0] for k in range(i0, i1 + 1)]
    ys = [series[k][1] for k in range(i0, i1 + 1)]
    if not xs:
        return None
    cx, cy = xs[0], ys[0]
    return max(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))


# ------------------------------------------------------------------------ core
def analyse(run_dir, phase_mode=None,
            swing_clearance_m=DEFAULT_SWING_CLEARANCE_M):
    cfg_path = os.path.join(run_dir, 'run_config.json')
    if not os.path.isfile(cfg_path):
        raise PhaseAnalysisError('run_config.json missing: %s' % run_dir)
    with open(cfg_path, 'r', encoding='utf-8') as fh:
        cfg = json.load(fh)
    for key in ('duration_sec', 'start_delay_sec', 'gait_blend_sec',
                'step_frequency_hz'):
        if not M.finite(cfg.get(key)):
            raise PhaseAnalysisError('run_config missing numeric %r' % key)
    if cfg['step_frequency_hz'] <= 0:
        raise PhaseAnalysisError('non-positive step_frequency_hz')
    ds = cfg.get('double_support_ratio')
    if not M.finite(ds):
        raise PhaseAnalysisError('run_config missing double_support_ratio')
    # The analysis phase mode comes from the run config; a requested mode that
    # disagrees with the recorded one is an explicit failure (never silently
    # analysed with the wrong schedule).  A *missing* mode is only legal for the
    # documented historical legacy schema and is recorded as inferred.
    recorded_mode, mode_source = resolve_phase_mode(cfg, run_dir)
    if phase_mode is None:
        phase_mode = recorded_mode
    elif phase_mode != recorded_mode:
        raise PhaseAnalysisError(
            'phase mode mismatch: requested %r != recorded %r in %s'
            % (phase_mode, recorded_mode, run_dir))
    trace = os.path.join(run_dir, 'trace')
    ready = M.ready_time(run_dir, trace)
    bodies = _load_jsonl(os.path.join(trace, 'body_pose_trace.jsonl'))
    states = _load_jsonl(os.path.join(trace, 'state_trace.jsonl'))
    if not states:
        raise PhaseAnalysisError('no state trace: %s' % run_dir)
    names_seen = False
    order_contract = {
        'names_provided': False,
        'names_inferred': False,
        'order_assumed': False,
        'canonical_order_verified': False,
        'order_unverifiable': False,
        'order_source': None,
        'schema_version': None,
        'n_joints': 24,
        'declared_names': None,
    }
    for s in states:
        if not (M.finite(s.get('t')) and isinstance(s.get('position'), list)
                and len(s['position']) == 24):
            raise PhaseAnalysisError('invalid state row in %s' % run_dir)
        for v in s['position']:
            if not M.finite(v):
                raise PhaseAnalysisError('non-finite joint in %s' % run_dir)
        names = s.get('name')
        if names is not None:
            if not isinstance(names, list) or len(names) != 24:
                raise PhaseAnalysisError(
                    'state joint names count mismatch in %s (got %r)'
                    % (run_dir, names))
            if list(names) != JOINT_ORDER:
                raise PhaseAnalysisError(
                    'state joint names/order do not match the canonical order '
                    'in %s' % run_dir)
            names_seen = True
    if names_seen:
        order_contract.update({
            'names_provided': True,
            'order_assumed': False,
            'canonical_order_verified': True,
            'order_source': 'state_trace.name (explicit canonical order)',
        })
    else:
        # Historical phase4b_trace.on_state schema persists only {t, position};
        # the producer validates /joint_states against the frozen 24-joint order
        # before recording, so the order is assumed, not file-verified.
        order_contract.update({
            'names_provided': False,
            'order_assumed': True,
            'canonical_order_verified': False,
            'order_unverifiable': False,
            'order_source': ('phase4b_trace.on_state historical schema: '
                             'frozen 24-joint order, name not persisted'),
            'schema_version': 'phase4b_trace.on_state/position24 (no name)',
        })

    lo, hi = A.steady_window(cfg, ready)
    gait_start = ready + float(cfg['start_delay_sec'])
    freq = float(cfg['step_frequency_hz'])
    idx = A.select_indices([s['t'] for s in states], lo, hi)
    if not idx:
        raise PhaseAnalysisError('zero window state samples: %s' % run_dir)

    recs = []
    for i in idx:
        s = states[i]
        t = s['t']
        body = _nearest(bodies, t)
        if body is None:
            raise PhaseAnalysisError('no body pose at state sample in %s' % run_dir)
        for f in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'):
            if not M.finite(body.get(f)):
                raise PhaseAnalysisError('non-finite body %s in %s' % (f, run_dir))
        q = {n: s['position'][k] for k, n in enumerate(JOINT_ORDER)}
        R = _quat_to_R(body['qx'], body['qy'], body['qz'], body['qw'])
        bpos = (body['x'], body['y'], body['z'])
        feet = {}
        for side in ('L', 'R'):
            pb = sole_position_base(q, side)
            feet[side] = [bpos[c] + sum(R[c][j] * pb[j] for j in range(3))
                          for c in range(3)]
        phase = ((t - gait_start) * freq) % 1.0
        name, progress, lrole, rrole = G.classify(phase, ds)
        recs.append({'t': t, 'phase': phase, 'name': name,
                     'progress': progress, 'lrole': lrole, 'rrole': rrole,
                     'l': feet['L'], 'r': feet['R']})

    contacts, contact_diag = validate_contacts(trace)
    cstate = _contact_state_fn(contacts)

    phase_counts = {name: 0 for name in G.PHASE_NAMES}
    for r in recs:
        phase_counts[r['name']] += 1
    total = len(recs)
    duration = hi - lo
    phase_stats = {}
    for name in G.PHASE_NAMES:
        n = phase_counts[name]
        phase_stats[name] = {'n_samples': n,
                             'fraction': n / total,
                             'approx_duration_sec': duration * n / total}

    ground = {}
    for side in ('L', 'R'):
        ground[side] = min(r[side.lower()][2] for r in recs)
    airborne = {}
    for side in ('L', 'R'):
        airborne[side] = [r[side.lower()][2] > ground[side] + swing_clearance_m
                          for r in recs]

    cc = [cstate('left', r['t']) for r in recs]
    cr = [cstate('right', r['t']) for r in recs]

    def rate(mask, sel):
        if not sel:
            return None
        return sum(1 for i in sel if mask[i]) / len(sel)

    ds_sel = [i for i, r in enumerate(recs) if r['name'] in
              ('DS_TO_LEFT_SUPPORT', 'DS_TO_RIGHT_SUPPORT')]
    both = [cstate('left', r['t']) and cstate('right', r['t']) for r in recs]
    ds_both_rate = rate(both, ds_sel)

    ls = [i for i, r in enumerate(recs) if r['name'] == 'LEFT_SWING']
    rs = [i for i, r in enumerate(recs) if r['name'] == 'RIGHT_SWING']
    left_swing_support_rate = rate(cr, ls)     # right foot contact bool during L swing
    right_swing_support_rate = rate(cc, rs)    # left foot contact bool during R swing
    left_swing_airborne = rate(airborne['L'], ls)
    right_swing_airborne = rate(airborne['R'], rs)

    # FK-ground support/swing indicators. The main-collision Bool is known (Phase
    # 4C) to be TRUE mostly for a *tilted* (swing/toe) foot and FALSE for a flat
    # support foot, so it is anti-correlated with support here; the sole-height
    # indicator below is the physically meaningful support/swing evidence.
    ground_lo = {side: ground[side] + swing_clearance_m for side in ('L', 'R')}
    l_support = [i for i, r in enumerate(recs) if r['lrole'] == 'support']
    r_support = [i for i, r in enumerate(recs) if r['rrole'] == 'support']
    left_support_ground = rate(
        [recs[i]['l'][2] <= ground_lo['L'] for i in range(total)], l_support)
    right_support_ground = rate(
        [recs[i]['r'][2] <= ground_lo['R'] for i in range(total)], r_support)
    ds_both_ground = rate(
        [recs[i]['l'][2] <= ground_lo['L'] and recs[i]['r'][2] <= ground_lo['R']
         for i in range(total)], ds_sel)

    clearance = {}
    for side in ('L', 'R'):
        sel = ls if side == 'L' else rs
        if sel:
            clearance[side] = max(recs[i][side.lower()][2] for i in sel) - ground[side]
        else:
            clearance[side] = None

    # support-foot world drift over commanded support intervals
    drift = {}
    support_intervals = {}
    for side, role_key in (('L', 'lrole'), ('R', 'rrole')):
        mask = [r[role_key] == 'support' for r in recs]
        ivals = _intervals(mask, [r['t'] for r in recs])
        support_intervals[side] = len(ivals)
        xy = [(r[side.lower()][0], r[side.lower()][1]) for r in recs]
        drifts = [_max_drift(xy, a, b) for a, b in ivals]
        drift[side] = {'n_intervals': len(ivals),
                       'max_m': max(drifts) if drifts else None,
                       'median_m': _median(drifts)}

    # zero contact intervals
    neither = [not (cstate('left', r['t']) or cstate('right', r['t']))
               for r in recs]
    zc = _intervals(neither, [r['t'] for r in recs])
    zc_durations = [recs[b]['t'] - recs[a]['t'] for a, b in zc]
    zero_contact = {'n_intervals': len(zc),
                    'max_duration_sec': max(zc_durations) if zc_durations else 0.0}

    # Method-B onset: the correct swing->support boundaries are the swing-end
    # phases (L: s = 0.5 - ds/2, R: 1 - ds/2). Kept separately from the renamed
    # M0 diagnostic so the two definitions/thresholds are never mixed.
    swing_to_support_onset = _onset_delays(
        recs, cstate, {'L': 0.5 - ds / 2.0, 'R': 1.0 - ds / 2.0}, freq)
    # M0 diagnostic (historical P1 definition); diagnostic only, not a gate.
    m0_contact_delay = _onset_delays(
        recs, cstate, {'L': 1.0 - ds / 2.0, 'R': 0.5}, freq)

    phase_zero = [name for name in G.PHASE_NAMES if phase_counts[name] == 0]
    locomotion = M.compute_run(run_dir, 'p1_%s' % phase_mode)
    measurable = bool(locomotion.get('measurable')) and not phase_zero
    result = {
        'run_dir': run_dir,
        'phase_mode_recorded': recorded_mode,
        'phase_mode_analysis': phase_mode,
        'phase_mode_source': mode_source,
        'phase_mode_assumed': mode_source == 'historical_legacy_schema',
        'joint_order': order_contract,
        'contact_diagnostics': contact_diag,
        'phase_zero_sample_names': phase_zero,
        'double_support_ratio': ds,
        'window': {'lo': lo, 'hi': hi, 'n_window_samples': total,
                   'approx_duration_sec': duration},
        'swing_clearance_threshold_m': swing_clearance_m,
        'phase_stats': phase_stats,
        'ds_both_contact_rate': ds_both_rate,
        'left_swing_right_support_contact_rate': left_swing_support_rate,
        'right_swing_left_support_contact_rate': right_swing_support_rate,
        'left_swing_airborne_fraction': left_swing_airborne,
        'right_swing_airborne_fraction': right_swing_airborne,
        'left_support_ground_fraction': left_support_ground,
        'right_support_ground_fraction': right_support_ground,
        'ds_both_ground_fraction': ds_both_ground,
        'swing_clearance_max_m': clearance,
        'ground_sole_z_m': ground,
        'support_foot_drift': drift,
        'support_intervals': support_intervals,
        'zero_contact': zero_contact,  # diagnostic (main-collision Bool artifact)
        'swing_to_support_onset_delay_sec': swing_to_support_onset,
        'm0_legacy_contact_delay_sec': m0_contact_delay,  # diagnostic only
        'locomotion': locomotion,
        'anomalies': locomotion.get('anomalies', []),
        'measurable': measurable,
    }
    return result


def _median(values):
    vals = sorted(float(v) for v in values if M.finite(v))
    if not vals:
        return None
    n = len(vals)
    return vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])


def evaluate_gates(result, gates):
    """Apply the frozen P1 structural + locomotion gates.

    ``gates`` may be the flat threshold mapping or the v2 wrapper; both are
    normalised so a wrapper file can be consumed directly by the CLI.
    """
    gates = normalize_gates(gates)
    loco = result.get('locomotion') or {}
    forward = loco.get('forward_displacement_m')
    lateral = loco.get('lateral_displacement_m')
    checks = {
        'measurable': result.get('measurable') is True,
        'locomotion_forward': M.finite(forward) and forward > gates['min_forward'],
        'lateral_ratio': (M.finite(forward) and M.finite(lateral) and forward > 0
                          and abs(lateral) / forward <= gates['max_lateral_ratio']),
        'yaw': M.finite(loco.get('yaw_change_deg'))
        and abs(loco['yaw_change_deg']) <= gates['max_yaw_deg'],
        'tilt': M.finite(loco.get('torso_tilt_max_deg'))
        and loco['torso_tilt_max_deg'] <= gates['max_tilt_deg'],
        'no_fall': loco.get('fall') is False,
        'duration': M.finite(loco.get('effective_duration_sec'))
        and loco['effective_duration_sec'] >= gates['min_duration'],
        'missing_duration': M.finite(loco.get('missing_duration_sec'))
        and loco['missing_duration_sec'] <= gates['max_missing'],
        'path_over_net': M.finite(loco.get('path_over_net'))
        and loco['path_over_net'] <= gates['max_path_over_net'],
        'contact_alternations': M.finite((loco.get('contact') or {}).get('alternation_count'))
        and loco['contact']['alternation_count'] >= gates['min_contact_alt'],
        'left_support_ground': (result.get('left_support_ground_fraction') is not None
                                and result['left_support_ground_fraction']
                                >= gates['min_support_ground']),
        'right_support_ground': (result.get('right_support_ground_fraction') is not None
                                 and result['right_support_ground_fraction']
                                 >= gates['min_support_ground']),
        'ds_both_ground': (result.get('ds_both_ground_fraction') is not None
                           and result['ds_both_ground_fraction']
                           >= gates['min_ds_both_ground']),
        'left_swing_airborne': (result.get('left_swing_airborne_fraction') is not None
                                and result['left_swing_airborne_fraction']
                                >= gates['min_airborne']),
        'right_swing_airborne': (result.get('right_swing_airborne_fraction') is not None
                                 and result['right_swing_airborne_fraction']
                                 >= gates['min_airborne']),
        'both_swing_clearance': (result.get('swing_clearance_max_m', {}).get('L') is not None
                                 and result['swing_clearance_max_m']['R'] is not None
                                 and result['swing_clearance_max_m']['L'] > gates['min_clearance']
                                 and result['swing_clearance_max_m']['R'] > gates['min_clearance']),
        'support_drift': (result.get('support_foot_drift', {}).get('L', {}).get('max_m') is not None
                          and result['support_foot_drift']['R']['max_m'] is not None
                          and result['support_foot_drift']['L']['max_m'] <= gates['max_support_drift_m']
                          and result['support_foot_drift']['R']['max_m'] <= gates['max_support_drift_m']),
        # Contact-onset *measurability* is a hard gate (every commanded
        # swing->support boundary must have a measured onset; missing => FAIL).
        # The onset delay *magnitude* is diagnostic only (Bool artifact).
        'swing_to_support_onset_measured': (
            isinstance(result.get('swing_to_support_onset_delay_sec'), dict)
            and result['swing_to_support_onset_delay_sec'].get('n_boundaries', 0) >= 1
            and result['swing_to_support_onset_delay_sec'].get('n_missing', 1) == 0),
    }
    eps = gates.get('epsilon_sec', 0.0)
    # time-unit gates use a pre-registered epsilon well below one sample period
    checks['duration'] = (M.finite(loco.get('effective_duration_sec'))
                          and loco['effective_duration_sec'] >= gates['min_duration'] - eps)
    checks['missing_duration'] = (M.finite(loco.get('missing_duration_sec'))
                                  and loco['missing_duration_sec'] <= gates['max_missing'] + eps)
    # Diagnostic-only quantities: the Bool contact is anti-correlated with flat
    # support (Phase 4C: the main collision box sits ~1.66 mm above the FSR, so a
    # flat support foot does not register), so zero_contact / M0 / Method-B onset
    # are reported but are NOT hard gates.
    diagnostics = {
        'zero_contact_max_sec': (result.get('zero_contact') or {}).get('max_duration_sec'),
        'm0_legacy_contact_delay_max_sec':
            (result.get('m0_legacy_contact_delay_sec') or {}).get('max_sec'),
        'swing_to_support_onset_delay_sec':
            result.get('swing_to_support_onset_delay_sec'),
        'ds_both_contact_rate': result.get('ds_both_contact_rate'),
        'left_swing_right_support_contact_rate':
            result.get('left_swing_right_support_contact_rate'),
        'right_swing_left_support_contact_rate':
            result.get('right_swing_left_support_contact_rate'),
        'note': ('diagnostic only: main-collision Bool is anti-correlated with '
                 'flat support (Phase 4C geometry)'),
    }
    return {'checks': checks, 'pass': all(checks.values()),
            'diagnostics': diagnostics,
            'contact_trace_valid': True}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', action='append', required=True)
    ap.add_argument('--phase-mode', default=None,
                    help='expected phase mode; default = the run_config value; '
                         'a mismatch with run_config is an explicit failure')
    ap.add_argument('--gates', default=None,
                    help='JSON file with the frozen P1-R gate thresholds (v2)')
    ap.add_argument('--out', required=True)
    args = ap.parse_args(argv)
    out = {'artifact': 'P0-a P1-R phase analysis',
           'definitions': {
               'phase_model': 'src/nao_gz_sim/scripts/nao_gait_phase.py',
               'swing_clearance_m': DEFAULT_SWING_CLEARANCE_M,
               'support_drift': 'max world-xy displacement from interval start over '
                                'each commanded support interval (slip proxy)',
               'swing_to_support_onset_delay_sec':
                   'Method B: delay from the commanded swing-end boundary '
                   '(L=0.5-ds/2, R=1-ds/2) to the first Bool contact=true of the '
                   'now-support foot within one cycle; already-true => 0; no '
                   'onset within the cycle is counted as n_missing (hard FAIL)',
               'm0_legacy_contact_delay_sec':
                   'diagnostic only (historical P1 M0 boundaries L=1-ds/2, R=0.5)',
               'zero_contact': 'diagnostic only (main-collision Bool artifact)',
               'evidence_levels': {
                   'fk_sole_height': 'kinematic consistency only (no contact sensing)',
                   'support_foot_world_drift': 'slip proxy, not a tangential force',
                   'bool_contact': 'main-collision contact timing only',
                   'phase_stats': 'nominal phase reconstructed from time/config, not measured',
                   'body_pose': 'base/pelvis world pose, not whole-body COM',
               },
           },
           'runs': []}
    gates = None
    try:
        if args.gates:
            gates = load_gates(args.gates)  # wrapper or flat, explicit failure
        for run in args.run:
            r = analyse(run, args.phase_mode)
            if gates:
                r['gates'] = evaluate_gates(r, gates)
            out['runs'].append(r)
    except PhaseAnalysisError as exc:
        sys.stderr.write('p0a_phase_analysis: error: %s\n' % exc)
        return 2
    out_dir = os.path.dirname(os.path.abspath(args.out))
    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError as exc:
        sys.stderr.write('p0a_phase_analysis: cannot create output dir %s: %s\n'
                         % (out_dir, exc))
        return 2
    try:
        with open(args.out, 'w', encoding='utf-8') as fh:
            # strict: NaN/Inf must never be serialised into an evidence file
            json.dump(out, fh, indent=2, sort_keys=True, allow_nan=False)
    except (OSError, ValueError) as exc:
        sys.stderr.write('p0a_phase_analysis: error writing %s: %s\n'
                         % (args.out, exc))
        return 2
    print('wrote %s (%d runs)' % (args.out, len(out['runs'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
