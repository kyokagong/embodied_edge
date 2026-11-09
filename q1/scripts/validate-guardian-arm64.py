"""Resolve the ARM64 runtime closure against the copied, real Q1 sysroot."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--package', type=Path, required=True)
p.add_argument('--sysroot', type=Path, required=True)
p.add_argument('--report', type=Path, required=True)
o = p.parse_args()
search = [o.package / 'lib', o.sysroot / 'opt/ros/humble/lib',
          o.sysroot / 'usr/lib/aarch64-linux-gnu']
queue = list((o.package / 'bin').iterdir()) + list((o.package / 'lib').glob('*.so*'))
seen, objects = set(), []
while queue:
    path = queue.pop().resolve()
    if path in seen:
        continue
    seen.add(path)
    with path.open('rb') as f:
        header = f.read(20)
    if header[:6] != b'\x7fELF\x02\x01' or int.from_bytes(header[18:20], 'little') != 183:
        raise SystemExit('Wrong ELF architecture: ' + str(path))
    dynamic = subprocess.check_output(['readelf', '-d', str(path)], text=True)
    needed = re.findall(r'\(NEEDED\).*?\[(.*?)\]', dynamic)
    for name in needed:
        candidate = next((directory / name for directory in search if (directory / name).exists()), None)
        if candidate is None:
            raise SystemExit('Unresolved ARM64 dependency: ' + name)
        queue.append(candidate)
    with path.open('rb') as f:
        digest = hashlib.file_digest(f, 'sha256').hexdigest()
    objects.append({'file': path.name, 'sha256': digest, 'needed': needed})
report = {'status': 'passed', 'target': 'AArch64', 'checked_objects': len(objects), 'objects': objects}
o.report.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({k: v for k, v in report.items() if k != 'objects'}))
