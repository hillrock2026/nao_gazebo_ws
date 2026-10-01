#!/usr/bin/env python3
"""Stage 7' — seed allocation + repository-wide freshness scan (offline only)."""

import hashlib
import json
import os
import re
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))

STAGE = "7-v4"
SEEDS = [909401, 909402, 909403, 909404, 909405]
FORBIDDEN_SEEDS = [10101, 30303, 40404, 50505,
                   606001, 606002, 606003, 606004, 606005,
                   707001, 707002, 707003, 707004, 707005,
                   808001, 809001,
                   909001, 909002, 909003, 909004, 909005,
                   909101, 909102, 909103, 909104, 909105,
                   909201, 909202, 909203, 909204, 909205,
                   909301, 909302, 909303, 909304, 909305]
SKIP_DIRS = {'.git', '__pycache__', 'node_modules', '.venv', 'build', 'log',
             '.cache', '.mypy_cache', '.pytest_cache'}
MAX_FILE_BYTES = 8_000_000
WINDOW = 30
SEED_KEY_RE = re.compile(r'(seed|physics_seed|gz_seed|replicate|repetition)[^0-9]{0,20}(\d{4,})',
                         re.IGNORECASE)


def sha256_text(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def atomic_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, path)


def repo_files():
    for base, dirs, files in os.walk(WS):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            full = os.path.join(base, name)
            try:
                if os.path.getsize(full) > MAX_FILE_BYTES:
                    continue
            except OSError:
                continue
            yield full


def scan_seed(seed, files):
    token = str(seed)
    literal = []
    semantic = []
    self_ref = []
    for full in files:
        try:
            txt = open(full, 'r', encoding='utf-8', errors='ignore').read()
        except OSError:
            continue
        if token not in txt:
            continue
        rel = os.path.relpath(full, WS)
        literal.append(rel)
        is_sem = False
        for m in re.finditer(re.escape(token), txt):
            w = txt[max(0, m.start() - WINDOW):m.end() + WINDOW].lower()
            if 'seed' in w or '物理' in w:
                is_sem = True
                break
        if not is_sem:
            for m in SEED_KEY_RE.finditer(txt):
                if m.group(2) == token:
                    is_sem = True
                    break
        if is_sem:
            (self_ref if os.path.abspath(full).startswith(os.path.abspath(EVID) + os.sep)
             else semantic).append(rel)
    return sorted(set(literal)), sorted(set(semantic)), sorted(set(self_ref))


def main():
    published_at = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    files = list(repo_files())
    scan, semantic_total, self_ref_total = {}, [], []
    for seed in SEEDS:
        literal, semantic, self_ref = scan_seed(seed, files)
        scan[str(seed)] = {'literal_hits': len(literal), 'semantic_hits': len(semantic),
                           'semantic_hit_files': semantic, 'self_reference_hits': self_ref}
        semantic_total.extend(semantic)
        self_ref_total.extend(self_ref)
    if [s for s in SEEDS if s in FORBIDDEN_SEEDS]:
        raise SystemExit('stage7r_seed_allocate: seed collides with forbidden set')
    if semantic_total:
        raise SystemExit('stage7r_seed_allocate: semantic freshness hits: %s'
                         % sorted(set(semantic_total)))
    payload = {'stage': STAGE, 'seed_count': len(SEEDS), 'seeds': SEEDS,
               'seed_source': 'operator-selected fresh holdout integers recorded before any Stage 7 run'}
    alloc_hash = sha256_text(json.dumps(payload, sort_keys=True))
    allocation = {
        'artifact': "Stage 7' B-only holdout seed allocation",
        'stage': STAGE, 'seed_count': len(SEEDS), 'seeds': SEEDS,
        'episodes_per_seed': 1,
        'seed_source': payload['seed_source'],
        'forbidden_or_historical_seeds': FORBIDDEN_SEEDS,
        'freshness_scan_scope': {
            'root': WS, 'skipped_dirs': sorted(SKIP_DIRS), 'max_file_bytes': MAX_FILE_BYTES,
            'semantic_rule': ("literal occurrence AND ('seed' within %d chars OR a seed-like "
                              "key immediately followed by the number)" % WINDOW),
            'n_files_scanned': len(files), 'per_seed': scan},
        'freshness_scan_hits': [],
        'self_reference_hit_files': sorted(set(self_ref_total)),
        'all_seeds_fresh': True,
        'seed_replacement_allowed': False, 'automatic_retry': False,
        'published_at': published_at,
        'seed_allocation_sha256': alloc_hash,
        'seed_allocation_sha256_note': ('SHA-256 over {stage, seed_count, seeds, '
                                        'seed_source}; allocation file also locked in SHA256SUMS.'),
    }
    run_order = {
        'artifact': "Stage 7' B-only holdout run order", 'stage': STAGE,
        'execution': 'strict_serial',
        'order': [{'run_order': i + 1, 'seed': s, 'episode': 'B',
                   'gait_phase_mode': 'explicit_support_swing'} for i, s in enumerate(SEEDS)],
        'seed_order_frozen': True, 'no_interleave': True, 'no_parallel': True,
        'published_at': published_at,
    }
    atomic_json(os.path.join(EVID, 'Stage7r_seed_allocation.json'), allocation)
    atomic_json(os.path.join(EVID, 'Stage7r_run_order.json'), run_order)
    print(json.dumps({'seeds': SEEDS, 'all_seeds_fresh': True,
                      'n_files_scanned': len(files),
                      'per_seed_semantic_hits': {k: v['semantic_hits'] for k, v in scan.items()},
                      'seed_allocation_sha256': alloc_hash}, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
