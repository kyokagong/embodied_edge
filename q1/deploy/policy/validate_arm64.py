#!/usr/bin/env python3
"""Audit ELF architecture and transitive dependencies against the actual Q1 sysroot."""
import argparse
import json
from pathlib import Path
import re
import subprocess
from manage import atomic_json, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--sysroot', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    options = parser.parse_args()
    search = [options.package / 'lib', options.sysroot / 'opt/ros/humble/lib',
              options.sysroot / 'usr/lib/aarch64-linux-gnu', options.sysroot / 'lib/aarch64-linux-gnu']
    queue = list((options.package / 'bin').iterdir()) + list((options.package / 'lib').glob('*.so*'))
    seen, objects = set(), []
    while queue:
        path = queue.pop().resolve()
        if path in seen:
            continue
        seen.add(path)
        with path.open('rb') as stream:
            header = stream.read(20)
        if header[:6] != b'\x7fELF\x02\x01' or int.from_bytes(header[18:20], 'little') != 183:
            raise SystemExit('Wrong ELF architecture: ' + str(path))
        dynamic = subprocess.check_output(['readelf', '-d', str(path)], text=True)
        needed = re.findall(r'\(NEEDED\).*?\[(.*?)\]', dynamic)
        for name in needed:
            if '/' in name or name in ('.', '..'):
                raise SystemExit('Unsafe dependency name: ' + name)
            candidate = next((directory / name for directory in search if (directory / name).is_file()), None)
            if candidate is None:
                raise SystemExit('Unresolved ARM64 dependency: ' + name)
            queue.append(candidate)
        objects.append({'file': path.name, 'sha256': sha(path), 'needed': needed})
    report = {'status': 'passed', 'target': 'AArch64', 'checked_objects': len(objects), 'objects': sorted(objects, key=lambda entry: entry['file'])}
    atomic_json(options.report, report)
    print(json.dumps({key: value for key, value in report.items() if key != 'objects'}))


if __name__ == '__main__':
    main()
