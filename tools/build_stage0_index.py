#!/usr/bin/env python3
"""Stage 0 corpus index builder.

Contract: docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md (REV1.1), §6.8.

Assembles ``corpus_index.json`` from the episode records and episode
directories produced by ``tools/run_stage0_dataset.py``.

Hard rules (RFC §5.4 / §6.8):
  * NEVER drop an episode. A failure episode is data and MUST be listed.
  * NEVER rank, score, select, or order by quality. The only ordering is
    deterministic by ``episode_id``.
  * Report EXECUTION status and BEHAVIOR status as two separate fields.
  * Read-only over episode artifacts.

Determinism (BF-3):
  * ``corpus_index.json`` contains NO volatile field. Writing it twice over the
    same episode set produces byte-identical output, so its SHA-256 is stable.
  * The index carries ``content_hash`` = SHA-256 over its own deterministic
    content (excluding ``content_hash``), making it self-verifying.
  * The only volatile value, ``generated_at``, is written to a SEPARATE sidecar
    ``corpus_index.meta.json`` — never into the hashed index content.
  * Both files are written atomically (temp file + fsync + ``os.replace``), so a
    reader never observes a partially written index.

Usage:
  python3 tools/build_stage0_index.py --root dataset/stage0
"""

import argparse
import hashlib
import json
import os
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

EPISODE_MARKERS = ('provenance.json', 'manifest.json', 'run_config.json')
INDEX_NAME = 'corpus_index.json'
META_NAME = 'corpus_index.meta.json'


def read_json(path):
    if not os.path.isfile(path):
        return None
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def canonical_bytes(obj):
    """Deterministic serialization (RFC hash convention, no trailing newline)."""
    return json.dumps(obj, indent=2, sort_keys=True, allow_nan=False).encode('utf-8')


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def discover_episode_dirs(root):
    """Any directory holding the canonical episode markers."""
    found = []
    for dirpath, dirnames, filenames in os.walk(root):
        if all(m in filenames for m in EPISODE_MARKERS):
            found.append(dirpath)
    return sorted(found)


def entry_from_record(rec):
    return {
        'episode_id': rec.get('episode_id'),
        'theta_id': rec.get('theta_id'),
        'path': rec.get('episode_dir'),
        'ring': rec.get('ring'),
        'ring_index': rec.get('ring_index'),
        'theta_list_hash': rec.get('theta_list_hash'),
        'theta_hash': rec.get('theta_hash'),
        'config_hash': rec.get('config_hash'),
        'resolved_theta_key_count': (rec.get('theta_contract') or {}).get('key_count'),
        'theta_contract_ok': (rec.get('theta_contract') or {}).get('ok'),
        'experiment_seed': rec.get('experiment_seed'),
        'gazebo_seed': rec.get('gazebo_seed'),
        'seed_verified': rec.get('seed_verified'),
        'status': {
            'execution_success': rec.get('execution_success'),
            'behavior_success': rec.get('behavior_success'),
            'episode_integrity': rec.get('episode_integrity'),
            'failure_class': rec.get('failure_class'),
            'termination': (rec.get('termination_status') or {}).get('termination'),
            'collection_status': (rec.get('termination_status') or {}).get('collection_status'),
        },
        'metrics_present': rec.get('metrics') is not None,
        'metrics_error': rec.get('metrics_error'),
        'git_commit': rec.get('git_commit'),
        'timestamp': rec.get('timestamp'),
        'record_source': 'record',
    }


def entry_from_episode_dir(ep_dir, ws):
    """Fallback: index an episode that has no sidecar record (never drop)."""
    prov = read_json(os.path.join(ep_dir, 'provenance.json')) or {}
    integ = read_json(os.path.join(ep_dir, 'episode_integrity.json')) or {}
    manifest = read_json(os.path.join(ep_dir, 'manifest.json')) or {}
    resolved = read_json(os.path.join(ep_dir, 'run_config.json')) or {}
    go = integ.get('gait_outcome') or {}
    return {
        'episode_id': manifest.get('episode_id') or os.path.basename(ep_dir),
        'theta_id': None,
        'path': os.path.relpath(ep_dir, ws),
        'ring': os.path.basename(os.path.dirname(ep_dir)),
        'ring_index': None,
        'theta_list_hash': None,
        'theta_hash': None,
        'config_hash': None,
        'resolved_theta_key_count': len(resolved) or None,
        'theta_contract_ok': None,
        'experiment_seed': (prov.get('seed') or {}).get('requested'),
        'gazebo_seed': (prov.get('seed') or {}).get('actual_gz_sim'),
        'seed_verified': (prov.get('seed') or {}).get('verified'),
        'status': {
            'execution_success': integ.get('episode_integrity') == 'PASS',
            'behavior_success': go.get('walking_pass'),
            'episode_integrity': integ.get('episode_integrity'),
            'failure_class': integ.get('failure_class'),
            'termination': integ.get('termination'),
            'collection_status': manifest.get('collection_status'),
        },
        'metrics_present': False,
        'metrics_error': 'no sidecar record; metrics not orchestrated',
        'git_commit': (prov.get('code_revision') or {}).get('git_commit'),
        'timestamp': manifest.get('start_time'),
        'record_source': 'episode_dir_fallback',
    }


def build_content(root, ws=WS):
    """Deterministic index content (no volatile fields)."""
    root = os.path.abspath(root)
    records_dir = os.path.join(root, 'records')
    entries = []
    seen_ids = set()

    if os.path.isdir(records_dir):
        for name in sorted(os.listdir(records_dir)):
            if not name.endswith('.record.json'):
                continue
            rec = read_json(os.path.join(records_dir, name))
            if not rec:
                continue
            e = entry_from_record(rec)
            entries.append(e)
            seen_ids.add(e['episode_id'])

    # Include any episode dir lacking a record — never drop.
    for ep_dir in discover_episode_dirs(root):
        eid = os.path.basename(ep_dir)
        if eid in seen_ids:
            continue
        entries.append(entry_from_episode_dir(ep_dir, ws))
        seen_ids.add(eid)

    # Deterministic, non-quality ordering.
    entries.sort(key=lambda e: (str(e.get('episode_id'))))

    n = len(entries)
    exec_pass = sum(1 for e in entries if e['status'].get('execution_success') is True)
    behavior_pass = sum(1 for e in entries if e['status'].get('behavior_success') is True)
    behavior_fail = sum(1 for e in entries if e['status'].get('behavior_success') is False)
    missing = sum(1 for e in entries if e.get('record_source') == 'episode_dir_fallback')
    hash_ok = sum(1 for e in entries if e.get('theta_contract_ok') is True)
    rings = sorted({str(e.get('ring')) for e in entries if e.get('ring')})

    theta_list_hashes = sorted({e['theta_list_hash'] for e in entries
                                if e.get('theta_list_hash')})

    return {
        'artifact': 'Stage 0 corpus index',
        'schema': 'stage0_corpus_index_1.0.0',
        'contract': {
            'rfc': 'docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md',
            'revision': 'REV1.1',
        },
        'root': os.path.relpath(root, ws) if root.startswith(ws) else root,
        'policy': {
            'ranking': 'NONE - entries are ordered by episode_id only',
            'selection': 'NONE - no best-gait selection',
            'dropping': 'NEVER - failure episodes are preserved',
            'statuses': 'execution_success (data coherent) and behavior_success '
                        '(gait walked) are independent',
        },
        'rings_present': rings,
        'theta_list_hashes': theta_list_hashes,
        'summary': {
            'episode_count': n,
            'execution_success_count': exec_pass,
            'execution_failure_count': n - exec_pass,
            'behavior_success_count': behavior_pass,
            'behavior_failure_count': behavior_fail,
            'behavior_unknown_count': n - behavior_pass - behavior_fail,
            'episodes_without_record': missing,
            'theta_contract_ok_count': hash_ok,
        },
        'episodes': entries,
    }


def write_atomic(path, obj):
    """Write JSON atomically: temp file in the same dir, fsync, os.replace."""
    directory = os.path.dirname(os.path.abspath(path)) or '.'
    tmp = os.path.join(directory, '.%s.tmp.%d' % (os.path.basename(path), os.getpid()))
    data = canonical_bytes(obj)
    with open(tmp, 'wb') as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)
    try:
        fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


def build_index(root, out_dir=None, ws=WS):
    """Return (index_path, meta_path, content, index_sha256)."""
    root = os.path.abspath(root)
    content = build_content(root, ws)
    content_hash = sha256_bytes(canonical_bytes(content))
    index = dict(content)
    index['content_hash'] = content_hash

    index_path = os.path.join(out_dir or root, INDEX_NAME)
    meta_path = os.path.join(out_dir or root, META_NAME)

    write_atomic(index_path, index)
    with open(index_path, 'rb') as fh:
        index_sha256 = sha256_bytes(fh.read())

    write_atomic(meta_path, {
        'artifact': 'Stage 0 corpus index metadata (volatile)',
        'schema': 'stage0_corpus_index_meta_1.0.0',
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'index_file': os.path.basename(index_path),
        'index_sha256': index_sha256,
        'content_hash': content_hash,
        'episode_count': content['summary']['episode_count'],
        'note': 'generated_at is volatile and is deliberately excluded from '
                'corpus_index.json so that the index hash is stable across runs.',
    })
    return index_path, meta_path, content, index_sha256


def main(argv=None):
    ap = argparse.ArgumentParser(description='Stage 0 corpus index builder')
    ap.add_argument('--root', default=os.path.join(WS, 'dataset', 'stage0'))
    ap.add_argument('--out', default=None,
                    help='output directory (default: <root>)')
    args = ap.parse_args(argv)

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        sys.stderr.write('build_stage0_index: root not found: %s\n' % root)
        return 2
    index_path, meta_path, content, index_sha256 = build_index(root, args.out)
    s = content['summary']
    print('wrote %s (content_hash=%s)' % (index_path, content_hash_short(content)))
    print('      %s (index_sha256=%s)' % (meta_path, index_sha256))
    print('      %d episodes; rings=%s; exec_pass=%d behavior_pass=%d '
          'behavior_fail=%d behavior_unknown=%d no_record=%d'
          % (s['episode_count'], content['rings_present'],
             s['execution_success_count'], s['behavior_success_count'],
             s['behavior_failure_count'], s['behavior_unknown_count'],
             s['episodes_without_record']))
    return 0


def content_hash_short(content):
    return sha256_bytes(canonical_bytes(content))[:16] + '...'


if __name__ == '__main__':
    sys.exit(main())
