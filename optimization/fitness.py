"""MVP fitness evaluator (Phase OPT-0).

Input : a `result_p0a.json` produced by the frozen runner (or its run dir).
Output: a single float, maximized by the optimizer.

Scoring contract (task-specified):

    invalid (could not measure)      -> -2.0
    fall                             -> -1.0
    normal                           -> bounded weighted score in [0, 1]

Ordering invariant: any normal score (>= 0) is better than a fall (-1), which is
better than an invalid episode (-2), so a strictly-monotone ranking exists for
CMA-ES even when it lands on a fall or a broken episode.

The evaluator also scans the FULL body_pose trace (not just the steady window)
for fall evidence, so a fall outside the scored window still counts.
"""

import json
import math
import os

# Frozen fall thresholds (identical to tools/analyze_measured_gait.py).
FALL_Z_M = 0.25
FALL_TILT_DEG = 60.0

DEFAULT_REFS = {
    'V_ref_mps': 0.05,
    'tilt_ref_deg': 30.0,
    'yaw_ref_deg': 45.0,
    'pon_ref': 20.0,
    'step_ref_m': 0.01,
}
DEFAULT_WEIGHTS = {
    'w_velocity': 1.0,
    'w_instability': 0.3,
    'w_energy': 0.1,
    'w_slip': 0.1,
}
# Squash scale for the bounded normal score: fitness = 0.5*(tanh(raw/scale)+1).
# tanh keeps the score in (0,1) (always above fall=-1) WITHOUT saturating, so
# the optimizer still sees a gradient between two acceptable gaits.
DEFAULT_SCORE_SCALE = 0.3
PENALTY_FALL = -1.0
PENALTY_INVALID = -2.0


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


class FitnessResult:
    def __init__(self, fitness, status, terms, detail):
        self.fitness = float(fitness)
        self.status = status          # 'ok' | 'fall' | 'invalid'
        self.terms = dict(terms)
        self.detail = dict(detail)

    def to_dict(self):
        return {'fitness': self.fitness, 'status': self.status,
                'terms': self.terms, 'detail': self.detail}


class FitnessEvaluator:
    def __init__(self, refs=None, weights=None,
                 penalty_fall=PENALTY_FALL, penalty_invalid=PENALTY_INVALID,
                 score_scale=DEFAULT_SCORE_SCALE):
        self.refs = dict(DEFAULT_REFS)
        if refs:
            self.refs.update(refs)
        self.weights = dict(DEFAULT_WEIGHTS)
        if weights:
            self.weights.update(weights)
        self.penalty_fall = float(penalty_fall)
        self.penalty_invalid = float(penalty_invalid)
        self.score_scale = float(score_scale) if _finite(score_scale) and score_scale > 0 else DEFAULT_SCORE_SCALE

    # ------------------------------------------------------------------ public
    def evaluate(self, result_path):
        """Return the fitness float (convenience wrapper)."""
        return self.evaluate_result(result_path).fitness

    def evaluate_result(self, result_path):
        """Return a FitnessResult for a result_p0a.json path or a run dir."""
        result_json, run_dir = _resolve(result_path)
        if result_json is None:
            return FitnessResult(self.penalty_invalid, 'invalid', {},
                                 {'reason': 'result_p0a.json not found',
                                  'path': str(result_path)})
        try:
            with open(result_json, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
        except (OSError, ValueError) as exc:
            return FitnessResult(self.penalty_invalid, 'invalid', {},
                                 {'reason': 'unreadable result: %s' % exc})

        measured = data.get('measured')
        if not isinstance(measured, dict) or not measured:
            return FitnessResult(self.penalty_invalid, 'invalid', {},
                                 {'reason': 'no measured block',
                                  'ok': data.get('ok'),
                                  'run_reason': data.get('reason')})
        if measured.get('measurable') is not True:
            return FitnessResult(self.penalty_invalid, 'invalid', {},
                                 {'reason': 'not measurable',
                                  'measured_reason': measured.get('reason')})

        # Full-trace fall scan (independent of the steady window).
        trace_fall, trace_detail = _scan_trace_fall(run_dir, measured)

        fall = bool(measured.get('fall')) or trace_fall
        if fall:
            evidence = list(measured.get('fall_evidence') or [])
            if trace_fall:
                evidence.append(trace_detail)
            return FitnessResult(self.penalty_fall, 'fall', {},
                                 {'fall_evidence': evidence,
                                  'measured_fall': bool(measured.get('fall')),
                                  'trace_fall': trace_fall})

        return self._normal_score(measured)

    # ------------------------------------------------------------------ normal
    def _normal_score(self, measured):
        r = self.refs

        speed = measured.get('actual_speed_mean_mps')
        vel = _clip(speed / r['V_ref_mps'], 0.0, 1.0) if _finite(speed) else 0.0

        tilt = measured.get('torso_tilt_max_deg')
        tilt_n = _clip(tilt / r['tilt_ref_deg'], 0.0, 1.0) if _finite(tilt) else 1.0
        yaw = measured.get('yaw_change_deg')
        yaw_n = _clip(abs(yaw) / r['yaw_ref_deg'], 0.0, 1.0) if _finite(yaw) else 1.0
        instability = 0.5 * tilt_n + 0.5 * yaw_n

        pon = measured.get('path_over_net')
        energy = (_clip((pon - 1.0) / (r['pon_ref'] - 1.0), 0.0, 1.0)
                  if _finite(pon) else 1.0)

        step = measured.get('max_single_step_m')
        slip = _clip(step / r['step_ref_m'], 0.0, 1.0) if _finite(step) else 1.0

        w = self.weights
        raw = (w['w_velocity'] * vel
               - w['w_instability'] * instability
               - w['w_energy'] * energy
               - w['w_slip'] * slip)
        # Bounded, non-saturating map: (0,1) strictly above the fall penalty.
        fitness = 0.5 * (math.tanh(raw / self.score_scale) + 1.0)

        terms = {'velocity': vel, 'instability': instability,
                 'energy': energy, 'slip': slip}
        detail = {'raw_metrics': {
                      'actual_speed_mean_mps': speed,
                      'torso_tilt_max_deg': tilt,
                      'yaw_change_deg': yaw,
                      'path_over_net': pon,
                      'max_single_step_m': step,
                      'forward_displacement_m': measured.get('forward_displacement_m')},
                  'refs': dict(r), 'weights': dict(w),
                  'score_scale': self.score_scale, 'raw_score': raw}
        return FitnessResult(fitness, 'ok', terms, detail)


# --------------------------------------------------------------------- helpers
def _resolve(path):
    """Accept a result_p0a.json path or a run dir; return (json_path, run_dir)."""
    if path is None:
        return None, None
    p = str(path)
    if os.path.isdir(p):
        run_dir = p
        cand = os.path.join(p, 'result_p0a.json')
        return (cand if os.path.isfile(cand) else None), run_dir
    if os.path.isfile(p):
        return p, os.path.dirname(p)
    return None, os.path.dirname(p)


def _scan_trace_fall(run_dir, measured):
    """Scan the FULL body_pose trace for fall evidence. Returns (bool, detail)."""
    if not run_dir:
        return False, {'reason': 'no run dir'}
    trace = os.path.join(run_dir, 'trace', 'body_pose_trace.jsonl')
    if not os.path.isfile(trace):
        return False, {'reason': 'no body_pose trace'}
    min_z = None
    max_tilt = None
    try:
        with open(trace, 'r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                z = row.get('z')
                if _finite(z):
                    min_z = z if min_z is None else min(min_z, z)
                roll, pitch = row.get('roll'), row.get('pitch')
                if _finite(roll) and _finite(pitch):
                    tilt = math.degrees(max(abs(roll), abs(pitch)))
                    max_tilt = tilt if max_tilt is None else max(max_tilt, tilt)
    except OSError as exc:
        return False, {'reason': 'trace read error: %s' % exc}

    ev = {'type': 'full_trace_scan', 'min_z_m': min_z, 'max_tilt_deg': max_tilt,
          'thresholds': {'z_m': FALL_Z_M, 'tilt_deg': FALL_TILT_DEG}}
    fall = ((min_z is not None and min_z <= FALL_Z_M)
            or (max_tilt is not None and max_tilt >= FALL_TILT_DEG))
    return fall, ev
