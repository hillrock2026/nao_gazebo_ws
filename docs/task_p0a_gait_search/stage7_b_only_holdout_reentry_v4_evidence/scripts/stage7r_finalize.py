#!/usr/bin/env python3
"""Stage 7' finalizer: SHA256SUMS + output manifest (offline)."""

import hashlib
import json
import os
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
EXCLUDE = {'SHA256SUMS', 'Stage7r_output_manifest.json'}


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    entries = []
    for base, dirs, files in os.walk(EVID):
        dirs[:] = [d for d in dirs if d != '__pycache__']
        for name in sorted(files):
            rel = os.path.relpath(os.path.join(base, name), EVID)
            if rel in EXCLUDE:
                continue
            entries.append((rel, sha256_file(os.path.join(base, name))))
    entries.sort()
    with open(os.path.join(EVID, 'SHA256SUMS'), 'w', encoding='utf-8') as fh:
        for rel, d in entries:
            fh.write('%s  %s\n' % (d, rel))
    man = {'artifact': "Stage 7' output manifest",
           'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
           'algorithm': 'SHA-256', 'n_outputs': len(entries),
           'outputs': [{'path': r, 'sha256': h} for r, h in entries],
           'self_excluded': sorted(EXCLUDE)}
    tmp = os.path.join(EVID, 'Stage7r_output_manifest.json.tmp')
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(man, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, os.path.join(EVID, 'Stage7r_output_manifest.json'))
    print(json.dumps({'n_outputs': len(entries)}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
