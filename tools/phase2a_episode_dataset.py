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
"""
Phase 2A episode-level dataset writer / reader / validator.

This module organizes the **frozen** Task 4 sample schema (v1.0.1, see
``docs/Task4_Trajectory_Schema.md`` and ``nao_trajectory_schema.py``) into an
episode-level dataset without touching the per-sample contract:

    dataset/
      manifest.json            dataset-level index of the whole collection
      index.jsonl              one line per episode
      episodes/
        episode_000001/
          samples.jsonl        BYTE-IDENTICAL copy of a frozen run's samples
          metadata.json        episode-level metadata (evaluation + provenance)
      splits/
        train.jsonl
        val.jsonl
        eval_fixed.jsonl

Design boundaries (frozen):

  * The original sample file is copied byte for byte and is never re-serialized,
    so every sample keeps its fields, order, types and JSON formatting.
  * Episode-level information (``episode_id``, ``success``, ``param_set``, ...)
    lives ONLY in ``metadata.json`` / the dataset index -- never inside a sample.
  * Every source episode MUST be a valid frozen run directory (``manifest.json``
    + ``samples.jsonl``) that passes the independent golden validator
    ``tools/validate_task4_phase4d_dataset.py``. Legacy frozen runs therefore
    keep working unchanged; this module only reads them.

The dataset is published atomically: everything is written into an instance
unique staging directory next to the target, self-validated, then swapped into
place with a single ``os.replace``. A failed build leaves no half dataset.

No ROS dependency.
"""

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
import random
import re
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

# ---------------------------------------------------------------------------
# Frozen contract constants
# ---------------------------------------------------------------------------
DATASET_FORMAT = 'nao_episode_dataset'
DATASET_FORMAT_VERSION = '1.0.0'
# Phase 2E: metadata schema 1.1.0 makes the ``usable`` semantics explicit
# (sample completeness, independent of success). 1.0.0 predates this change.
METADATA_SCHEMA_VERSION = '1.1.0'
SUPPORTED_METADATA_SCHEMA_VERSIONS = ('1.0.0', '1.1.0')
USABLE_SEMANTICS = ('success_bound', 'sample_completeness')

EPISODE_ID_RE = re.compile(r'^episode_[0-9]{6}$')

SPLITS = ('train', 'val', 'eval_fixed')

MANIFEST_FILE = 'manifest.json'
INDEX_FILE = 'index.jsonl'
EPISODES_DIR = 'episodes'
SPLITS_DIR = 'splits'
SAMPLES_FILE = 'samples.jsonl'
METADATA_FILE = 'metadata.json'
SAMPLE_MANIFEST_FILE = 'manifest.json'

# Canonical gait parameters. ``param_set`` is a flat, queryable object; these
# keys are the stable contract (see docs/Phase2A_Episode_Dataset_Format.md).
CANONICAL_PARAM_KEYS = (
    'step_length_m',
    'step_frequency_hz',
    'swing_height_m',
    'double_support_ratio',
    'sway_amplitude_rad',
    'torso_pitch_rad',
)

REQUIRED_METADATA_KEYS = (
    'episode_id',
    'controller_name',
    'seed',
    'param_set',
    'duration_sec',
    'fell',
    'fall_time',
    'mean_speed_x_mps',
    'max_torso_tilt_deg',
    'energy_qdot',
    'success',
    'usable',
    'failure_reason',
)

# Optional metadata keys recognised (and type-checked) by this module. Unknown
# extra keys are tolerated so the format can grow without a hard migration, but
# the writer only ever emits the keys listed here plus ``sample_manifest``.
OPTIONAL_METADATA_SCALARS = {
    'schema_version': str,
    'controller_version': str,
    'created_at': str,
    'notes': str,
}
OPTIONAL_METADATA_INTS = ('sample_count',)

INDEX_KEYS = ['episode_id', 'split', 'metadata', 'samples', 'sample_count',
              'usable', 'success']


class EpisodeDatasetError(Exception):
    """Base error for every explicit, non-silent dataset failure."""


class MetadataValidationError(EpisodeDatasetError):
    def __init__(self, errors):
        self.errors = list(errors)
        super().__init__('episode metadata invalid: %s' % (self.errors,))


class PathSafetyError(EpisodeDatasetError):
    pass


# ---------------------------------------------------------------------------
# small typed helpers (bool is never an int/float here)
# ---------------------------------------------------------------------------
def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_finite(value):
    return is_number(value) and math.isfinite(float(value))


def is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _is_nonempty_str(value):
    return isinstance(value, str) and value != ''


def _is_meaningful_str(value):
    """A string carrying information (not empty and not whitespace-only)."""
    return isinstance(value, str) and value.strip() != ''


_SHA256_RE = re.compile(r'^[0-9a-f]{64}$')


# ---------------------------------------------------------------------------
# param_set validation
# ---------------------------------------------------------------------------
def validate_param_set(param_set):
    """
    Return a list of ``{'field', 'reason'}`` dicts (empty = valid).

    The six canonical keys are the frozen numeric contract: they must exist and
    hold a finite JSON number (bool is explicitly rejected even though Python's
    bool is an int subclass). Extension keys stay duck-typed scalars.
    """
    errors = []
    if not isinstance(param_set, dict):
        return [{'field': 'param_set', 'reason': 'not_an_object'}]
    for key in CANONICAL_PARAM_KEYS:
        if key not in param_set:
            errors.append({'field': 'param_set.%s' % key, 'reason': 'missing'})
            continue
        value = param_set[key]
        if not is_number(value):
            errors.append({'field': 'param_set.%s' % key,
                           'reason': 'canonical_not_number'})
        elif not math.isfinite(float(value)):
            errors.append({'field': 'param_set.%s' % key, 'reason': 'non_finite'})
    for key, value in param_set.items():
        if key in CANONICAL_PARAM_KEYS:
            continue
        if not isinstance(key, str) or key == '':
            errors.append({'field': 'param_set', 'reason': 'empty_or_non_string_key'})
            continue
        if isinstance(value, bool):
            continue
        if isinstance(value, str):
            if value == '':
                errors.append({'field': 'param_set.%s' % key,
                               'reason': 'empty_string_value'})
            continue
        if is_number(value):
            if not math.isfinite(float(value)):
                errors.append({'field': 'param_set.%s' % key,
                               'reason': 'non_finite'})
            continue
        errors.append({'field': 'param_set.%s' % key, 'reason': 'unsupported_type'})
    return errors


# ---------------------------------------------------------------------------
# metadata validation
# ---------------------------------------------------------------------------
def validate_metadata(meta, expected_episode_id=None):
    """
    Validate one episode metadata object.

    Returns a list of ``{'field', 'reason', 'detail'}`` dicts; empty = valid.
    All checks are explicit and deterministic -- no silent coercion.
    """
    errors = []
    if not isinstance(meta, dict):
        return [{'field': '<metadata>', 'reason': 'not_an_object'}]

    for key in REQUIRED_METADATA_KEYS:
        if key not in meta:
            errors.append({'field': key, 'reason': 'missing'})

    episode_id = meta.get('episode_id')
    if 'episode_id' in meta:
        if not isinstance(episode_id, str) or not EPISODE_ID_RE.match(episode_id):
            errors.append({'field': 'episode_id', 'reason': 'invalid_id',
                           'detail': episode_id})
        elif expected_episode_id is not None and episode_id != expected_episode_id:
            errors.append({'field': 'episode_id', 'reason': 'id_mismatch',
                           'detail': {'expected': expected_episode_id,
                                      'got': episode_id}})

    if 'controller_name' in meta and not _is_nonempty_str(meta['controller_name']):
        errors.append({'field': 'controller_name', 'reason': 'not_nonempty_string'})

    if 'seed' in meta and not is_int(meta['seed']):
        errors.append({'field': 'seed', 'reason': 'not_an_integer'})

    if 'param_set' in meta:
        errors.extend(validate_param_set(meta['param_set']))

    if 'duration_sec' in meta:
        value = meta['duration_sec']
        if not is_finite(value) or float(value) <= 0.0:
            errors.append({'field': 'duration_sec', 'reason': 'not_positive_finite'})

    fell = meta.get('fell')
    if 'fell' in meta and not isinstance(fell, bool):
        errors.append({'field': 'fell', 'reason': 'not_boolean'})

    if 'fall_time' in meta:
        fall_time = meta['fall_time']
        if not is_finite(fall_time):
            errors.append({'field': 'fall_time', 'reason': 'not_finite_number'})
        elif isinstance(fell, bool):
            if fell:
                if float(fall_time) < 0.0:
                    errors.append({'field': 'fall_time',
                                   'reason': 'fell_true_requires_nonnegative'})
                if is_finite(meta.get('duration_sec')) and \
                        float(fall_time) > float(meta['duration_sec']) + 1e-9:
                    errors.append({'field': 'fall_time',
                                   'reason': 'after_duration'})
            elif float(fall_time) != -1.0:
                errors.append({'field': 'fall_time',
                               'reason': 'not_fell_requires_minus_one'})

    for key, reason in (('mean_speed_x_mps', 'not_finite_number'),
                        ('max_torso_tilt_deg', 'not_nonnegative_finite'),
                        ('energy_qdot', 'not_nonnegative_finite')):
        if key in meta:
            value = meta[key]
            if not is_finite(value):
                errors.append({'field': key, 'reason': reason})
            elif key in ('max_torso_tilt_deg', 'energy_qdot') and float(value) < 0.0:
                errors.append({'field': key, 'reason': reason})

    success = meta.get('success')
    usable = meta.get('usable')
    if 'success' in meta and not isinstance(success, bool):
        errors.append({'field': 'success', 'reason': 'not_boolean'})
    if 'usable' in meta and not isinstance(usable, bool):
        errors.append({'field': 'usable', 'reason': 'not_boolean'})

    failure_reason = meta.get('failure_reason')
    if 'failure_reason' in meta:
        if failure_reason is not None and not _is_meaningful_str(failure_reason):
            errors.append({'field': 'failure_reason',
                           'reason': 'not_null_or_nonempty_string'})

    # ---- state relationships (only when the relevant types are sound) ----
    # NOTE (Phase 2D): ``usable`` means "the episode's samples are complete and
    # auditable", NOT "the gait succeeded". A failed-but-complete episode is
    # legitimately (success=False, usable=True); the earlier usable=>success
    # invariant was relaxed for that cross-phase requirement. Success still
    # forbids a fall and still requires a null failure_reason.
    if isinstance(success, bool) and isinstance(fell, bool):
        if success and fell:
            errors.append({'field': 'success',
                           'reason': 'success_forbids_fell'})
    if isinstance(success, bool):
        if success and failure_reason is not None:
            errors.append({'field': 'failure_reason',
                           'reason': 'success_requires_null'})
        if not success and not _is_meaningful_str(failure_reason):
            errors.append({'field': 'failure_reason',
                           'reason': 'failure_requires_reason'})

    # ---- optional typed fields ----
    for key, typ in OPTIONAL_METADATA_SCALARS.items():
        if key in meta and not isinstance(meta[key], typ):
            errors.append({'field': key, 'reason': 'wrong_type'})
    for key in OPTIONAL_METADATA_INTS:
        if key in meta and (not is_int(meta[key]) or meta[key] < 0):
            errors.append({'field': key, 'reason': 'not_nonnegative_integer'})
    if 'sample_manifest' in meta and not isinstance(meta['sample_manifest'], dict):
        errors.append({'field': 'sample_manifest', 'reason': 'not_an_object'})
    # ---- schema version + usable semantics (Phase 2E) ----
    if 'schema_version' in meta:
        version = meta['schema_version']
        if version not in SUPPORTED_METADATA_SCHEMA_VERSIONS:
            errors.append({'field': 'schema_version',
                           'reason': 'unknown_schema_version', 'detail': version})
        elif version == '1.1.0':
            # 1.1.0 must declare the sample-completeness usable semantics.
            if meta.get('usable_semantics') != 'sample_completeness':
                errors.append({'field': 'usable_semantics',
                               'reason': 'required_for_v1_1_0'})
    if 'usable_semantics' in meta and \
            meta['usable_semantics'] not in USABLE_SEMANTICS:
        errors.append({'field': 'usable_semantics',
                       'reason': 'invalid_usable_semantics',
                       'detail': meta['usable_semantics']})
    if 'samples_sha256' in meta and \
            not (isinstance(meta['samples_sha256'], str)
                 and _SHA256_RE.match(meta['samples_sha256'])):
        errors.append({'field': 'samples_sha256', 'reason': 'invalid_sha256'})

    return errors


def build_metadata(episode_id, controller_name, seed, param_set, duration_sec,
                   fell, fall_time, mean_speed_x_mps, max_torso_tilt_deg,
                   energy_qdot, success, usable, failure_reason=None,
                   **optional):
    """Assemble and validate an episode metadata dict; raise on any error."""
    meta = {
        'schema_version': optional.pop('schema_version', METADATA_SCHEMA_VERSION),
        'usable_semantics': optional.pop('usable_semantics',
                                         'sample_completeness'),
        'episode_id': episode_id,
        'controller_name': controller_name,
        'seed': seed,
        'param_set': copy.deepcopy(param_set),
        'duration_sec': duration_sec,
        'fell': fell,
        'fall_time': fall_time,
        'mean_speed_x_mps': mean_speed_x_mps,
        'max_torso_tilt_deg': max_torso_tilt_deg,
        'energy_qdot': energy_qdot,
        'success': success,
        'usable': usable,
        'failure_reason': failure_reason,
    }
    for key, value in optional.items():
        if value is not None:
            meta[key] = value
    errors = validate_metadata(meta, expected_episode_id=episode_id)
    if errors:
        raise MetadataValidationError(errors)
    return meta


def param_set_from_producer(params, torso_pitch_rad=0.0):
    """
    Compatibility mapping: Phase 4B producer parameters -> canonical param_set.

    The frozen Phase 4B producer uses its own ROS parameter names. Only the
    shared, physiologically meaningful gait parameters are mapped to the
    canonical keys; ``torso_pitch_rad`` is a reserved Phase 2A key that the
    frozen Phase 4B gait does not vary (default 0.0, passed explicitly by the
    caller, never silently invented). Extra producer parameters are preserved
    as scalar extras for provenance.
    """
    if not isinstance(params, dict):
        raise MetadataValidationError(
            [{'field': 'params', 'reason': 'not_an_object'}])
    mapped = {
        'step_length_m': params.get('step_length_m'),
        'step_frequency_hz': params.get('step_frequency_hz'),
        'swing_height_m': params.get('swing_height_m'),
        'double_support_ratio': params.get('double_support_ratio'),
        'sway_amplitude_rad': params.get('sway_amplitude_rad'),
        'torso_pitch_rad': torso_pitch_rad,
    }
    for key, value in params.items():
        if key in mapped or value is None:
            continue
        if isinstance(value, (bool, str)) or is_number(value):
            mapped[key] = value
    errors = validate_param_set(mapped)
    if errors:
        raise MetadataValidationError(errors)
    return mapped


# ---------------------------------------------------------------------------
# path safety
# ---------------------------------------------------------------------------
def normalize_relative_path(rel):
    """
    Validate a dataset-relative POSIX path and return its normalized form.

    Rejects absolute paths, empty paths, Windows drive prefixes, NUL/backslash,
    empty/'.'/'..' components. This is purely lexical; ``resolve_within`` adds
    the filesystem/symlink containment check.
    """
    if not isinstance(rel, str) or rel == '':
        raise PathSafetyError('path must be a non-empty string: %r' % (rel,))
    if '\x00' in rel:
        raise PathSafetyError('path contains NUL: %r' % (rel,))
    if '\\' in rel:
        raise PathSafetyError('backslash not allowed in dataset paths: %r' % (rel,))
    if os.path.isabs(rel) or (len(rel) >= 2 and rel[1] == ':'):
        raise PathSafetyError('absolute path not allowed: %r' % (rel,))
    parts = []
    for part in rel.split('/'):
        if part in ('', '.'):
            continue
        if part == '..':
            raise PathSafetyError('parent traversal not allowed: %r' % (rel,))
        parts.append(part)
    if not parts:
        raise PathSafetyError('path resolves to nothing: %r' % (rel,))
    return '/'.join(parts)


def resolve_within(root, rel):
    """Return the absolute path of ``rel`` under ``root`` or raise."""
    normalized = normalize_relative_path(rel)
    root_real = os.path.realpath(root)
    candidate = os.path.join(root_real, *normalized.split('/'))
    candidate_real = os.path.realpath(candidate)
    prefix = root_real + os.sep
    if candidate_real != root_real and not candidate_real.startswith(prefix):
        raise PathSafetyError('path escapes dataset root: %r' % (rel,))
    return candidate


# ---------------------------------------------------------------------------
# structural helpers
# ---------------------------------------------------------------------------
def _strict_dumps(obj):
    return json.dumps(obj, ensure_ascii=False, separators=(',', ':'),
                      allow_nan=False)


def _reject_constant(name):
    raise ValueError('non-finite JSON constant: %s' % name)


def _load_json(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh, parse_constant=_reject_constant)


def _load_jsonl(path):
    rows = []
    with open(path, 'r', encoding='utf-8') as fh:
        for lineno, line in enumerate(fh, start=1):
            if not line.strip():
                raise EpisodeDatasetError(
                    '%s:%d empty line' % (path, lineno))
            rows.append(json.loads(line, parse_constant=_reject_constant))
    return rows


def _atomic_write_text(path, text):
    directory = os.path.dirname(path) or '.'
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.tmp_', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _fsync_dir(path):
    try:
        fd = os.open(path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


def _sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def _count_sample_lines(path):
    """Count non-empty JSONL records; empty lines are a structural error."""
    count = 0
    with open(path, 'r', encoding='utf-8') as fh:
        for lineno, line in enumerate(fh, start=1):
            if not line.endswith('\n'):
                raise EpisodeDatasetError(
                    '%s:%d truncated line (no trailing newline)' % (path, lineno))
            if not line.strip():
                raise EpisodeDatasetError('%s:%d empty line' % (path, lineno))
            count += 1
    return count


# ---------------------------------------------------------------------------
# split planning (deterministic, order independent, mutually exclusive)
# ---------------------------------------------------------------------------
def plan_splits(episode_ids, seed=0, train_ratio=0.8, val_ratio=0.1):
    """
    Deterministically partition episode ids into train / val / eval_fixed.

    The ids are sorted first, then shuffled with ``random.Random(seed)``, so the
    result never depends on the caller's iteration order. ``eval_fixed`` takes
    the remainder after floor(train*n) and floor(val*n), so every id is assigned
    to exactly one split.
    """
    if not is_int(seed):
        raise EpisodeDatasetError('split seed must be an integer')
    for name, ratio in (('train_ratio', train_ratio), ('val_ratio', val_ratio)):
        if not is_finite(ratio) or float(ratio) < 0.0 or float(ratio) > 1.0:
            raise EpisodeDatasetError('%s must be in [0, 1]' % name)
    if float(train_ratio) + float(val_ratio) > 1.0 + 1e-9:
        raise EpisodeDatasetError('train_ratio + val_ratio must be <= 1')
    ids = list(episode_ids)
    if len(ids) != len(set(ids)):
        raise EpisodeDatasetError('duplicate episode_id in split input')
    ordered = sorted(ids)
    random.Random(seed).shuffle(ordered)
    n = len(ordered)
    n_train = int(n * float(train_ratio))
    n_val = int(n * float(val_ratio))
    n_eval = n - n_train - n_val
    plan = {}
    for eid in ordered[:n_train]:
        plan[eid] = 'train'
    for eid in ordered[n_train:n_train + n_val]:
        plan[eid] = 'val'
    for eid in ordered[n_train + n_val:n_train + n_val + n_eval]:
        plan[eid] = 'eval_fixed'
    return plan


def validate_split_plan(plan):
    """Return error dicts for a split plan that is not a total partition."""
    errors = []
    if not isinstance(plan, dict):
        return [{'field': 'splits', 'reason': 'not_an_object'}]
    for eid, split in plan.items():
        if split not in SPLITS:
            errors.append({'field': eid, 'reason': 'illegal_split',
                           'detail': split})
    return errors


# ---------------------------------------------------------------------------
# dataset writer
# ---------------------------------------------------------------------------
def _load_golden_validator():
    path = os.path.join(_TOOLS, 'validate_task4_phase4d_dataset.py')
    if not os.path.isfile(path):
        raise EpisodeDatasetError('golden validator not found: %s' % path)
    spec = importlib.util.spec_from_file_location(
        'phase2a_golden_validator', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate_frozen_run(run_dir):
    """Validate a frozen run dir with the independent golden validator."""
    validator = _load_golden_validator()
    report = validator.validate_dataset(os.path.abspath(run_dir))
    if not report.get('valid'):
        raise EpisodeDatasetError(
            'source run is not a valid frozen dataset: %s\nerrors=%s'
            % (run_dir, json.dumps(report.get('errors', []), sort_keys=True)))
    return report


def _read_frozen_manifest(run_dir):
    path = os.path.join(run_dir, SAMPLE_MANIFEST_FILE)
    manifest = _load_json(path)
    if not isinstance(manifest, dict):
        raise EpisodeDatasetError('frozen manifest is not an object: %s' % path)
    for key in ('schema_name', 'schema_version'):
        if not _is_nonempty_str(manifest.get(key)):
            raise EpisodeDatasetError(
                'frozen manifest missing %s: %s' % (key, path))
    return manifest


class EpisodeInput:
    """One episode to add to a dataset."""

    def __init__(self, episode_id, run_dir, metadata, split=None):
        self.episode_id = episode_id
        self.run_dir = os.path.abspath(run_dir)
        self.metadata = metadata
        self.split = split


class EpisodeDataset:
    """Reader, validator and atomic writer for a Phase 2A episode dataset."""

    def __init__(self, root):
        self.root = os.path.abspath(root)

    # ------------------------------------------------------------- paths
    def episode_dir(self, episode_id):
        return os.path.join(self.root, EPISODES_DIR, episode_id)

    def metadata_path(self, episode_id):
        return os.path.join(self.episode_dir(episode_id), METADATA_FILE)

    def samples_path(self, episode_id):
        return os.path.join(self.episode_dir(episode_id), SAMPLES_FILE)

    @property
    def manifest_path(self):
        return os.path.join(self.root, MANIFEST_FILE)

    @property
    def index_path(self):
        return os.path.join(self.root, INDEX_FILE)

    def split_path(self, split):
        if split not in SPLITS:
            raise EpisodeDatasetError('illegal split name: %r' % (split,))
        return os.path.join(self.root, SPLITS_DIR, '%s.jsonl' % split)

    # ------------------------------------------------------------- reading
    def load_manifest(self):
        return _load_json(self.manifest_path)

    def load_index(self):
        return _load_jsonl(self.index_path)

    def load_metadata(self, episode_id):
        return _load_json(self.metadata_path(episode_id))

    def load_splits(self):
        result = {}
        for split in SPLITS:
            rows = _load_jsonl(self.split_path(split))
            result[split] = rows
        return result

    # ------------------------------------------------------------- writing
    def build(self, episodes, dataset_name, sample_schema, split_seed=0,
              train_ratio=0.8, val_ratio=0.1, created_at=None, generator=None,
              overwrite=False):
        """
        Atomically build a complete dataset from ``episodes``.

        ``episodes`` is a list of :class:`EpisodeInput`. Every source run is
        validated by the golden frozen validator before anything is written.
        Returns the final validation report (which is guaranteed valid).
        """
        if not _is_nonempty_str(dataset_name):
            raise EpisodeDatasetError('dataset_name is required')
        _validate_sample_schema(sample_schema)
        episodes = list(episodes)
        if not episodes:
            raise EpisodeDatasetError('at least one episode is required')
        ids = [e.episode_id for e in episodes]
        if len(ids) != len(set(ids)):
            raise EpisodeDatasetError('duplicate episode_id in build input')
        for e in episodes:
            if not isinstance(e, EpisodeInput):
                raise EpisodeDatasetError('episodes must be EpisodeInput instances')

        # Explicit vs planned splits: never mix the two silently.
        explicit = [e.split is not None for e in episodes]
        if any(explicit) and not all(explicit):
            raise EpisodeDatasetError(
                'either give a split for every episode or none')
        if all(explicit):
            split_plan = {e.episode_id: e.split for e in episodes}
            errors = validate_split_plan(split_plan)
            if errors:
                raise EpisodeDatasetError('illegal explicit split: %s' % errors)
        else:
            split_plan = plan_splits(ids, seed=split_seed,
                                     train_ratio=train_ratio,
                                     val_ratio=val_ratio)

        if os.path.isdir(self.root) and os.listdir(self.root) and not overwrite:
            raise EpisodeDatasetError(
                'dataset root already exists and is not empty: %s '
                '(pass overwrite=True to replace)' % self.root)
        if os.path.exists(self.root) and not os.path.isdir(self.root):
            raise EpisodeDatasetError('dataset root is not a directory: %s'
                                      % self.root)

        parent = os.path.dirname(self.root) or '.'
        os.makedirs(parent, exist_ok=True)
        staging = tempfile.mkdtemp(prefix='.phase2a_staging_', dir=parent)
        try:
            self._write_into(staging, episodes, split_plan,
                             dataset_name, sample_schema, split_seed,
                             train_ratio, val_ratio, created_at, generator)
            report = _validate_dataset_dir(staging)
            if not report['valid']:
                raise EpisodeDatasetError(
                    'self-validation of staged dataset failed: %s'
                    % json.dumps(report['errors'], sort_keys=True))
            self._swap_into_place(staging)
            return report
        except BaseException:
            if os.path.isdir(staging):
                shutil.rmtree(staging, ignore_errors=True)
            raise

    def _write_into(self, staging, episodes, split_plan, dataset_name,
                    sample_schema, split_seed, train_ratio, val_ratio,
                    created_at, generator):
        ep_root = os.path.join(staging, EPISODES_DIR)
        os.makedirs(ep_root)
        entries = []
        total_samples = 0
        for episode in sorted(episodes, key=lambda e: e.episode_id):
            run_dir = episode.run_dir
            if not os.path.isdir(run_dir):
                raise EpisodeDatasetError('source run dir missing: %s' % run_dir)
            validate_frozen_run(run_dir)
            frozen_manifest = _read_frozen_manifest(run_dir)
            _require_schema_match(sample_schema, frozen_manifest)

            metadata = copy.deepcopy(episode.metadata)
            existing = metadata.get('sample_manifest')
            if existing is not None and existing != frozen_manifest:
                raise EpisodeDatasetError(
                    'metadata.sample_manifest conflicts with source for %s'
                    % episode.episode_id)
            metadata['sample_manifest'] = frozen_manifest

            sample_count = _count_sample_lines(
                os.path.join(run_dir, SAMPLES_FILE))
            if 'sample_count' in metadata and metadata['sample_count'] != sample_count:
                raise EpisodeDatasetError(
                    'metadata.sample_count %r != actual %d for %s'
                    % (metadata['sample_count'], sample_count,
                       episode.episode_id))
            metadata['sample_count'] = sample_count

            errors = validate_metadata(metadata,
                                       expected_episode_id=episode.episode_id)
            if errors:
                raise MetadataValidationError(errors)

            dest_dir = os.path.join(ep_root, episode.episode_id)
            os.makedirs(dest_dir)
            src_samples = os.path.join(run_dir, SAMPLES_FILE)
            shutil.copyfile(src_samples, os.path.join(dest_dir, SAMPLES_FILE))
            if _sha256_file(src_samples) != _sha256_file(
                    os.path.join(dest_dir, SAMPLES_FILE)):
                raise EpisodeDatasetError(
                    'sample copy verification failed for %s' % episode.episode_id)
            metadata['samples_sha256'] = _sha256_file(
                os.path.join(dest_dir, SAMPLES_FILE))
            _atomic_write_text(os.path.join(dest_dir, METADATA_FILE),
                               json.dumps(metadata, ensure_ascii=False,
                                          indent=2, sort_keys=True,
                                          allow_nan=False) + '\n')
            total_samples += sample_count
            entries.append({
                'episode_id': episode.episode_id,
                'split': split_plan[episode.episode_id],
                'metadata': '%s/%s/%s' % (EPISODES_DIR, episode.episode_id,
                                          METADATA_FILE),
                'samples': '%s/%s/%s' % (EPISODES_DIR, episode.episode_id,
                                         SAMPLES_FILE),
                'sample_count': sample_count,
                'usable': metadata['usable'],
                'success': metadata['success'],
            })

        entries.sort(key=lambda row: row['episode_id'])
        _atomic_write_text(os.path.join(staging, INDEX_FILE),
                           ''.join(_strict_dumps(row) + '\n' for row in entries))

        split_root = os.path.join(staging, SPLITS_DIR)
        os.makedirs(split_root)
        for split in SPLITS:
            ids = sorted(row['episode_id'] for row in entries
                         if row['split'] == split)
            _atomic_write_text(
                os.path.join(split_root, '%s.jsonl' % split),
                ''.join(_strict_dumps({'episode_id': eid}) + '\n'
                        for eid in ids))

        manifest = {
            'dataset_name': dataset_name,
            'dataset_format': DATASET_FORMAT,
            'dataset_format_version': DATASET_FORMAT_VERSION,
            'metadata_schema_version': METADATA_SCHEMA_VERSION,
            'sample_schema': {
                'name': sample_schema['name'],
                'version': sample_schema['version'],
            },
            'robot': 'nao',
            'episode_count': len(entries),
            'sample_count': total_samples,
            'episodes_dir': EPISODES_DIR,
            'index_file': INDEX_FILE,
            'split_files': {split: '%s/%s.jsonl' % (SPLITS_DIR, split)
                            for split in SPLITS},
            'split_seed': split_seed,
            'split_ratios': {'train': train_ratio, 'val': val_ratio},
        }
        if created_at is not None:
            manifest['created_at'] = created_at
        if generator is not None:
            manifest['generator'] = generator
        _atomic_write_text(os.path.join(staging, MANIFEST_FILE),
                           json.dumps(manifest, ensure_ascii=False, indent=2,
                                      sort_keys=True, allow_nan=False) + '\n')

    def _swap_into_place(self, staging):
        parent = os.path.dirname(self.root) or '.'
        if os.path.isdir(self.root):
            backup = '%s.replaced.%d' % (self.root, os.getpid())
            os.replace(self.root, backup)
            try:
                os.replace(staging, self.root)
            except OSError:
                os.replace(backup, self.root)
                raise
            shutil.rmtree(backup, ignore_errors=True)
        else:
            os.replace(staging, self.root)
        _fsync_dir(parent)

    # ------------------------------------------------------------- validate
    def validate(self):
        return _validate_dataset_dir(self.root)


def _validate_sample_schema(sample_schema):
    if not isinstance(sample_schema, dict):
        raise EpisodeDatasetError('sample_schema must be an object')
    for key in ('name', 'version'):
        if not _is_nonempty_str(sample_schema.get(key)):
            raise EpisodeDatasetError('sample_schema.%s is required' % key)


def _require_schema_match(expected, frozen_manifest):
    if frozen_manifest.get('schema_name') != expected.get('name') or \
            frozen_manifest.get('schema_version') != expected.get('version'):
        raise EpisodeDatasetError(
            'source run sample schema %s/%s does not match dataset %s/%s'
            % (frozen_manifest.get('schema_name'),
               frozen_manifest.get('schema_version'),
               expected.get('name'), expected.get('version')))


# ---------------------------------------------------------------------------
# dataset validation (reader side)
# ---------------------------------------------------------------------------
def _validate_dataset_dir(root):
    report = {'dataset_dir': os.path.abspath(root), 'valid': True,
              'errors': [], 'episode_count': 0, 'sample_count': 0,
              'manifest_ok': False, 'index_ok': False, 'splits_ok': False}
    errors = report['errors']

    def fail(where, reason, **extra):
        entry = {'where': where, 'reason': reason}
        entry.update(extra)
        errors.append(entry)

    if not os.path.isdir(root):
        fail('dataset', 'not_a_directory')
        report['valid'] = False
        return report

    top = sorted(os.listdir(root))
    if top != sorted([MANIFEST_FILE, INDEX_FILE, EPISODES_DIR, SPLITS_DIR]):
        fail('dataset', 'unexpected_directory_contents', entries=top)
        report['valid'] = False
        return report

    # ---- manifest ----
    manifest = None
    try:
        manifest = _load_json(os.path.join(root, MANIFEST_FILE))
    except (OSError, ValueError) as exc:
        fail('manifest', 'manifest_unreadable', detail=str(exc))
    if isinstance(manifest, dict):
        report['manifest_ok'] = True
        for key, expected in (('dataset_format', DATASET_FORMAT),
                              ('dataset_format_version', DATASET_FORMAT_VERSION)):
            if manifest.get(key) != expected:
                report['manifest_ok'] = False
                fail('manifest', 'manifest_value_mismatch',
                     path=key, expected=expected, got=manifest.get(key))
        if not isinstance(manifest.get('sample_schema'), dict) or \
                not _is_nonempty_str(manifest.get('sample_schema', {}).get('name')) \
                or not _is_nonempty_str(
                    manifest.get('sample_schema', {}).get('version')):
            report['manifest_ok'] = False
            fail('manifest', 'sample_schema_invalid')
    elif manifest is not None:
        fail('manifest', 'manifest_not_object')

    # ---- index ----
    index = []
    try:
        index = _load_jsonl(os.path.join(root, INDEX_FILE))
        report['index_ok'] = True
    except (OSError, ValueError, EpisodeDatasetError) as exc:
        fail('index', 'index_unreadable', detail=str(exc))
    seen_ids = set()
    for lineno, row in enumerate(index, start=1):
        if not isinstance(row, dict):
            fail('index', 'row_not_object', line=lineno)
            report['index_ok'] = False
            continue
        if list(row.keys()) != INDEX_KEYS:
            report['index_ok'] = False
            fail('index', 'index_keys_mismatch', line=lineno,
                 keys=list(row.keys()),
                 missing=[k for k in INDEX_KEYS if k not in row],
                 extra=[k for k in row if k not in INDEX_KEYS])
            continue
        eid = row.get('episode_id')
        if not isinstance(eid, str) or not EPISODE_ID_RE.match(eid):
            report['index_ok'] = False
            fail('index', 'invalid_episode_id', line=lineno, detail=eid)
            continue
        if eid in seen_ids:
            report['index_ok'] = False
            fail('index', 'duplicate_episode', line=lineno, episode_id=eid)
            continue
        seen_ids.add(eid)
        if row.get('split') not in SPLITS:
            report['index_ok'] = False
            fail('index', 'illegal_split', line=lineno, split=row.get('split'))
        for key in ('metadata', 'samples'):
            try:
                resolved = resolve_within(root, row.get(key))
            except PathSafetyError as exc:
                report['index_ok'] = False
                fail('index', 'path_unsafe', line=lineno, field=key,
                     detail=str(exc))
                continue
            if not os.path.isfile(resolved):
                report['index_ok'] = False
                fail('index', 'referenced_file_missing', line=lineno,
                     field=key, path=row.get(key))
        for key in ('usable', 'success'):
            if not isinstance(row.get(key), bool):
                report['index_ok'] = False
                fail('index', 'summary_not_boolean', line=lineno, field=key)
        if not is_int(row.get('sample_count')) or row.get('sample_count') < 0:
            report['index_ok'] = False
            fail('index', 'sample_count_invalid', line=lineno)

    # ---- episodes vs index (metadata + samples authority) ----
    ep_dir = os.path.join(root, EPISODES_DIR)
    on_disk = set()
    if os.path.isdir(ep_dir):
        for name in os.listdir(ep_dir):
            if os.path.isdir(os.path.join(ep_dir, name)):
                on_disk.add(name)
            elif os.path.isfile(os.path.join(ep_dir, name)):
                fail('episodes', 'stray_file', name=name)
    if on_disk - seen_ids:
        fail('episodes', 'unindexed_episode', episodes=sorted(on_disk - seen_ids))
    if seen_ids - on_disk:
        fail('episodes', 'missing_episode_dir', episodes=sorted(seen_ids - on_disk))

    if report['manifest_ok']:
        if manifest.get('episode_count') != len(index):
            report['manifest_ok'] = False
            fail('manifest', 'episode_count_mismatch',
                 expected=manifest.get('episode_count'), got=len(index))

    total_samples = 0
    split_assignments = {}
    for row in index:
        # Every malformed row was already reported in the first pass; the
        # second pass must never crash on it. Non-dict rows and rows without a
        # sound episode_id are skipped here.
        if not isinstance(row, dict):
            continue
        eid = row.get('episode_id')
        if not isinstance(eid, str) or not EPISODE_ID_RE.match(eid):
            continue
        meta_ref = row.get('metadata')
        if not _is_nonempty_str(meta_ref):
            # Missing/None path is reported by index_keys_mismatch/path_unsafe.
            continue
        try:
            meta = _load_json(resolve_within(root, meta_ref))
        except (OSError, ValueError, PathSafetyError) as exc:
            fail('metadata', 'metadata_unreadable', episode_id=eid, detail=str(exc))
            continue
        if not isinstance(meta, dict):
            fail('metadata', 'metadata_not_object', episode_id=eid)
            continue
        meta_errors = validate_metadata(meta, expected_episode_id=eid)
        for err in meta_errors:
            fail('metadata', err.get('reason'), episode_id=eid,
                 field=err.get('field'), detail=err.get('detail'))
        if row.get('usable') != meta.get('usable') or \
                row.get('success') != meta.get('success'):
            fail('metadata', 'index_summary_mismatch', episode_id=eid,
                 detail={'index_usable': row.get('usable'),
                         'meta_usable': meta.get('usable'),
                         'index_success': row.get('success'),
                         'meta_success': meta.get('success')})

        samples_ref = row.get('samples')
        actual = None
        if _is_nonempty_str(samples_ref):
            try:
                actual = _count_sample_lines(resolve_within(root, samples_ref))
            except (OSError, EpisodeDatasetError, PathSafetyError) as exc:
                fail('samples', 'samples_unreadable', episode_id=eid,
                     detail=str(exc))
        if actual is not None:
            total_samples += actual
            if actual != row.get('sample_count'):
                fail('samples', 'sample_count_mismatch', episode_id=eid,
                     index=row.get('sample_count'), actual=actual)
            if 'sample_count' in meta and meta['sample_count'] != actual:
                fail('samples', 'metadata_sample_count_mismatch', episode_id=eid,
                     metadata=meta['sample_count'], actual=actual)
            # Optional content-integrity check: when the writer recorded a
            # streamed SHA-256, the reader recomputes it. Absent hash (legacy
            # datasets) is tolerated; a present-but-wrong hash is a failure.
            recorded = meta.get('samples_sha256')
            if isinstance(recorded, str) and _SHA256_RE.match(recorded):
                try:
                    actual_hash = _sha256_file(
                        resolve_within(root, samples_ref))
                except (OSError, PathSafetyError) as exc:
                    fail('samples', 'samples_unreadable', episode_id=eid,
                         detail=str(exc))
                else:
                    if actual_hash != recorded:
                        fail('samples', 'samples_hash_mismatch', episode_id=eid,
                             expected=recorded, actual=actual_hash)
        split = row.get('split')
        if split in SPLITS:
            split_assignments.setdefault(split, set()).add(eid)

    if report['manifest_ok'] and manifest.get('sample_count') != total_samples:
        report['manifest_ok'] = False
        fail('manifest', 'sample_count_mismatch',
             expected=manifest.get('sample_count'), got=total_samples)

    # ---- splits ----
    split_dir = os.path.join(root, SPLITS_DIR)
    present = sorted(os.listdir(split_dir)) if os.path.isdir(split_dir) else []
    expected_files = sorted('%s.jsonl' % s for s in SPLITS)
    if present != expected_files:
        fail('splits', 'unexpected_split_files', entries=present)
    else:
        report['splits_ok'] = True
        all_split_ids = {}
        declared = set()
        for split in SPLITS:
            path = os.path.join(split_dir, '%s.jsonl' % split)
            try:
                rows = _load_jsonl(path)
            except (OSError, ValueError, EpisodeDatasetError) as exc:
                report['splits_ok'] = False
                fail('splits', 'split_unreadable', split=split, detail=str(exc))
                continue
            ids = []
            for lineno, row in enumerate(rows, start=1):
                if not isinstance(row, dict) or \
                        list(row.keys()) != ['episode_id']:
                    report['splits_ok'] = False
                    fail('splits', 'split_row_invalid', split=split, line=lineno)
                    continue
                eid = row['episode_id']
                if not isinstance(eid, str) or eid not in seen_ids:
                    report['splits_ok'] = False
                    fail('splits', 'unknown_episode', split=split,
                         line=lineno, episode_id=eid)
                    continue
                ids.append(eid)
                if eid in declared:
                    report['splits_ok'] = False
                    fail('splits', 'episode_in_multiple_splits',
                         episode_id=eid, split=split)
                declared.add(eid)
                if eid in all_split_ids and all_split_ids[eid] != split:
                    report['splits_ok'] = False
                    fail('splits', 'split_disagreement', episode_id=eid,
                         first=all_split_ids[eid], second=split)
                all_split_ids[eid] = split
            if len(ids) != len(set(ids)):
                report['splits_ok'] = False
                fail('splits', 'duplicate_in_split', split=split)
        missing = seen_ids - declared
        if missing:
            report['splits_ok'] = False
            fail('splits', 'episode_missing_from_splits',
                 episodes=sorted(missing))
        for eid, split in all_split_ids.items():
            index_split = next(
                (r['split'] for r in index
                 if isinstance(r, dict) and r.get('episode_id') == eid), None)
            if index_split is not None and split != index_split:
                report['splits_ok'] = False
                fail('splits', 'split_disagrees_with_index', episode_id=eid,
                     split_file=split, index=index_split)

    report['episode_count'] = len(index)
    report['sample_count'] = total_samples
    report['valid'] = not errors
    return report


# ---------------------------------------------------------------------------
# legacy compatibility: read an original frozen run the old way
# ---------------------------------------------------------------------------
def read_frozen_run(run_dir):
    """Return (manifest, list-of-samples) for a frozen run, unchanged."""
    manifest = _read_frozen_manifest(run_dir)
    sample_path = os.path.join(run_dir, SAMPLES_FILE)
    samples = _load_jsonl(sample_path)
    return manifest, samples


def register_run(run_dir, episode_id, metadata):
    """
    Build an :class:`EpisodeInput` from a frozen collector run.

    The run is validated by the golden validator at build time; metadata keeps
    full authority. This is the integration point for the existing Phase 4D
    data-generation flow.
    """
    return EpisodeInput(episode_id, run_dir, metadata)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _cmd_build(args):
    with open(args.spec, 'r', encoding='utf-8') as fh:
        spec = json.load(fh, parse_constant=_reject_constant)
    episodes = []
    for item in spec['episodes']:
        metadata = item['metadata'] if 'metadata' in item else None
        if metadata is None and item.get('metadata_file'):
            metadata = _load_json(item['metadata_file'])
        if not isinstance(metadata, dict):
            raise EpisodeDatasetError(
                'episode %s has no metadata object' % item.get('episode_id'))
        episodes.append(EpisodeInput(item['episode_id'], item['run_dir'],
                                     metadata, split=item.get('split')))
    sample_schema = spec.get('sample_schema') or _read_frozen_manifest(
        episodes[0].run_dir)
    if 'schema_name' in sample_schema and 'name' not in sample_schema:
        sample_schema = {'name': sample_schema['schema_name'],
                         'version': sample_schema['schema_version']}
    dataset = EpisodeDataset(args.dataset)
    report = dataset.build(
        episodes,
        dataset_name=spec.get('dataset_name', 'nao_gait'),
        sample_schema=sample_schema,
        split_seed=spec.get('split_seed', 0),
        train_ratio=spec.get('train_ratio', 0.8),
        val_ratio=spec.get('val_ratio', 0.1),
        created_at=spec.get('created_at'),
        generator=spec.get('generator'),
        overwrite=args.overwrite)
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


def _cmd_validate(args):
    dataset = EpisodeDataset(args.dataset)
    report = dataset.validate()
    text = json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            fh.write(text)
    print(text)
    return 0 if report['valid'] else 1


def _cmd_index(args):
    dataset = EpisodeDataset(args.dataset)
    for row in dataset.load_index():
        print(_strict_dumps(row))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Phase 2A episode dataset tool')
    sub = parser.add_subparsers(dest='command', required=True)

    p_build = sub.add_parser('build', help='atomically build a dataset from a spec')
    p_build.add_argument('--dataset', required=True)
    p_build.add_argument('--spec', required=True)
    p_build.add_argument('--overwrite', action='store_true')
    p_build.set_defaults(func=_cmd_build)

    p_val = sub.add_parser('validate', help='validate a dataset directory')
    p_val.add_argument('--dataset', required=True)
    p_val.add_argument('--json-out', default=None)
    p_val.set_defaults(func=_cmd_validate)

    p_idx = sub.add_parser('index', help='print index.jsonl')
    p_idx.add_argument('--dataset', required=True)
    p_idx.set_defaults(func=_cmd_index)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except EpisodeDatasetError as exc:
        # Deliberate, domain-level failures are reported as a structured JSON
        # error (no traceback). Unexpected exceptions are NOT caught here.
        print(json.dumps({'ok': False, 'reason': type(exc).__name__,
                          'detail': str(exc)}, ensure_ascii=False),
              file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
