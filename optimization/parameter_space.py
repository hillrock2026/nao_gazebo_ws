"""MVP gait parameter space (Phase OPT-0).

Maps a NORMALIZED optimization vector x in [0,1]^5 onto the gait-generator
override keys that the frozen runner (`tools/run_p0a_gait_candidate.py`)
accepts. The generator itself is never touched; all optimization happens in the
normalized space, and the linear projection keeps every candidate inside the
producer's own parameter-validation ranges.

    x[0] step_length        -> step_length_m          [0.02, 0.08] m
    x[1] frequency          -> step_frequency_hz      [0.40, 1.60] Hz
    x[2] swing_height       -> swing_height_m         [0.005, 0.040] m
    x[3] double_support     -> double_support_ratio   [0.05, 0.45]
    x[4] sway_amplitude     -> sway_amplitude_rad     [0.05, 0.30] rad
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Axis:
    name: str            # logical name
    override_key: str    # key accepted by run_p0a_gait_candidate overrides
    unit: str
    low: float
    high: float
    anchor_value: float  # frozen baseline value


# Order defines the vector layout. Ranges are strictly inside the producer's
# _validate_parameters ranges (see docs/gait_optimization/CURRENT_IMPLEMENTATION_AUDIT.md §3.1).
AXES = (
    Axis('step_length', 'step_length_m', 'm', 0.02, 0.08, 0.02),
    Axis('frequency', 'step_frequency_hz', 'Hz', 0.40, 1.60, 0.5),
    Axis('swing_height', 'swing_height_m', 'm', 0.005, 0.040, 0.008),
    Axis('double_support', 'double_support_ratio', '-', 0.05, 0.45, 0.3),
    Axis('sway_amplitude', 'sway_amplitude_rad', 'rad', 0.05, 0.30, 0.06),
)


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


class ParameterSpace:
    def __init__(self, axes=AXES):
        self.axes = tuple(axes)

    # -- shape ---------------------------------------------------------------
    @property
    def dim(self):
        return len(self.axes)

    @property
    def names(self):
        return tuple(a.name for a in self.axes)

    def bounds(self):
        """(lower, upper) in normalized space -> all [0, 1]."""
        return ([0.0] * self.dim, [1.0] * self.dim)

    def clip_normalized(self, x):
        return tuple(_clip(float(v), 0.0, 1.0) for v in x)

    # -- mapping -------------------------------------------------------------
    def from_normalized(self, x):
        """Normalized [0,1]^dim -> override dict {override_key: value}.

        Keys are the raw config keys the frozen runner expects (it adds the
        ``phase4b_`` prefix itself). Non-finite input is clamped, never emitted.
        """
        if len(x) != self.dim:
            raise ValueError('expected %d values, got %d' % (self.dim, len(x)))
        out = {}
        for axis, raw in zip(self.axes, x):
            v = float(raw)
            if v != v:  # NaN
                v = 0.0
            out[axis.override_key] = axis.low + _clip(v, 0.0, 1.0) * (
                axis.high - axis.low)
        return out

    def to_normalized(self, overrides):
        """Override/actual value dict -> normalized vector (inverse of above)."""
        vec = []
        for axis in self.axes:
            value = overrides.get(axis.override_key, axis.anchor_value)
            span = axis.high - axis.low
            vec.append(_clip((float(value) - axis.low) / span, 0.0, 1.0))
        return tuple(vec)

    def anchor_normalized(self):
        return self.to_normalized(
            {a.override_key: a.anchor_value for a in self.axes})

    # -- introspection -------------------------------------------------------
    def describe(self):
        return [{'name': a.name, 'override_key': a.override_key, 'unit': a.unit,
                 'low': a.low, 'high': a.high, 'anchor_value': a.anchor_value}
                for a in self.axes]

    def to_named(self, x):
        """Normalized vector -> {'name': actual_value} for logging."""
        raw = self.from_normalized(x)
        return {a.name: raw[a.override_key] for a in self.axes}
