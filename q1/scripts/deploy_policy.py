#!/usr/bin/env python3
"""Explicitly deploy a policy-v2 runtime and independently versioned model."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deploy/policy'))
from manage import remote_main

if __name__ == '__main__':
    remote_main('deploy')
