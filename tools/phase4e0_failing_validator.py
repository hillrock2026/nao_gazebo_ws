#!/usr/bin/env python3
"""Deliberately failing validator used to inject a pre-commit failure.

Task 4 Phase 4E-0 evidence only: it always exits 1 so the collector's
pre-commit validation path is exercised and the dataset must NOT be published.
"""

import sys


def main():
    print('injected validator failure for dataset: %s'
          % (sys.argv[1] if len(sys.argv) > 1 else '<none>'))
    return 1


if __name__ == '__main__':
    sys.exit(main())
