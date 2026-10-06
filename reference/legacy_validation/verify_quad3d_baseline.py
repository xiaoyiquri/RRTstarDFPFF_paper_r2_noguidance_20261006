#!/usr/bin/env python3
"""Verify immutable Quad3D configurations, DF and shared PFF/SIMD source files."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / 'validation_logs/quad3d_frozen_baseline.json').read_text(encoding='utf-8'))
    failed = []
    for name, expected in manifest.items():
        path = root / name
        actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else 'missing'
        if actual != expected:
            failed.append(name)
    if failed:
        print('FAIL: frozen Quad3D baseline changed: ' + ', '.join(failed))
        return 1
    print(f'PASS: {len(manifest)} frozen Quad3D/PFF/SIMD files unchanged.')
    print('Run CTest quad3d_baseline for deterministic tree comparison; real FCL cases still require the VM.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
