#!/usr/bin/env python3
"""Verify the entire supplied package and extract immutable native runtime data."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deploy/policy'))
from manage import prepare_model, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, default=Path(__file__).resolve().parents[1] / 'artifacts/policy/models')
    options = parser.parse_args()
    target = prepare_model(options.source, options.output_root)
    print(json.dumps({'model_directory': str(target), 'package_sha256': sha(target / 'SHA256SUMS'), 'model_sha256': sha(target / 'model/policy_native.onnx'), 'numerical_preflight_required': True}))


if __name__ == '__main__':
    main()
