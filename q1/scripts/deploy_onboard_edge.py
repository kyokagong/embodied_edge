"""Install the locally built Q1 actor/console and attach to the existing guard.

The existing guard is never restarted or cleared. The actor starts --shadow-only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import time
import urllib.request


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--package', type=Path, required=True)
    p.add_argument('--ssh-key', type=Path, required=True)
    p.add_argument('--operator-key', type=Path, required=True)
    p.add_argument('--host', required=True)
    p.add_argument('--user', default='run')
    p.add_argument('--guard-root', required=True)
    p.add_argument('--model-dir', required=True)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--replace-root', help='Replace only an inspected --shadow-only actor in this release directory')
    o = p.parse_args()
    package = o.package.resolve()
    for line in (package/'SHA256SUMS').read_text().splitlines():
        expected, name = line.split(maxsplit=1)
        file = (package/name).resolve()
        if not file.is_relative_to(package) or sha(file) != expected:
            raise ValueError('Invalid release file: '+name)
    header = (package/'bin/q1-edge').read_bytes()[:20]
    if header[:6] != b'\x7fELF\x02\x01' or int.from_bytes(header[18:20], 'little') != 183:
        raise ValueError('Q1 actor must be AArch64')
    tag = sha(package/'SHA256SUMS')[:12]
    remote = f'/home/{o.user}/q1-supermimic/onboard-{tag}'
    archive = package.parent/f'onboard-{tag}.tar.gz'
    with tarfile.open(archive, 'w:gz', compresslevel=1) as tar:
        tar.add(package, arcname='.')
    args = ['-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=3',
            '-o','ServerAliveCountMax=3','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes',
            '-i',str(o.ssh_key)]
    target = f'{o.user}@{o.host}'
    q = shlex.quote
    def ssh(command, **kw):
        return subprocess.run(['ssh',*args,target,command],text=True,check=True,**kw)
    ssh(f'mkdir -p {q(remote)}/private; chmod 700 {q(remote)}/private')
    for source, destination in [(archive, remote+'.tar.gz'), (o.operator_key, remote+'/private/operator-key')]:
        subprocess.run(['scp','-q',*args,str(source),target+':'+destination],check=True)
    ssh(f'tar -xzf {q(remote)}.tar.gz -C {q(remote)} && cd {q(remote)} && sha256sum -c SHA256SUMS >/dev/null && chmod 600 private/operator-key && ./run --help >/dev/null')
    # Retire only the old network observer that owns the single local IPC slot.
    # The independent RT/HAL process and its latched fault remain untouched.
    script = '''import pathlib,subprocess,json,socket,time
old=%r
if old:
 old=pathlib.Path(old);pid=int((old/'state/edge.pid').read_text());proc=pathlib.Path(f'/proc/{pid}/cmdline')
 if proc.exists() and proc.read_bytes():
  cmd=proc.read_bytes();assert str(old).encode() in cmd and b'q1-edge' in cmd and b'--shadow-only' in cmd
  subprocess.run(['sudo','-n','kill','-TERM',str(pid)],check=True)
  until=time.monotonic()+5
  while proc.exists() and proc.read_bytes() and time.monotonic()<until:time.sleep(.1)
  assert not proc.exists() or not proc.read_bytes(),'Old observation actor did not exit'
r=pathlib.Path(%r)
profile=json.loads((r/'config/hardware.json').read_text())
assert profile['supported_motion']==['air_hold'] and sum(profile['kp'])<=2
rt=int((r/'state/rt.pid').read_text());cmd=pathlib.Path(f'/proc/{rt}/cmdline').read_bytes()
assert str(r).encode() in cmd and b'--commissioning' in cmd
s=socket.socket();assert s.connect_ex(('127.0.0.1',8768))!=0,'Q1 console port already in use';s.close()
f=r/'state/net.pid'
if f.exists():
 pid=int(f.read_text());proc=pathlib.Path(f'/proc/{pid}/cmdline')
 if proc.exists() and proc.read_bytes():
  cmd=proc.read_bytes();assert str(r).encode() in cmd and b'q1-guard-net' in cmd and b'server' in cmd
  subprocess.run(['sudo','-n','kill','-TERM',str(pid)],check=True)
time.sleep(.2)
print('Kept existing RT/HAL guard PID',rt)
''' % (o.replace_root, o.guard_root)
    ssh('python3 -', input=script)
    ssh(f'cd {q(remote)} && ./start {q(o.model_dir)} {q(o.guard_root)}')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic()+90
    state = None
    last_error = ''
    while time.monotonic()<deadline:
        try:
            with opener.open(f'http://{o.host}:8768/api/state',timeout=3) as f:
                state=json.load(f)
            if state['deployment']['inference_location']=='Q1' and state['shadow_only'] and state['policy_steps']>0:
                break
        except (OSError,ValueError) as error:
            last_error=str(error)
        time.sleep(.5)
    else:
        raise RuntimeError('Q1 actor not ready; inspect state/edge.log: '+last_error)
    report={'host':o.host,'remote_directory':remote,'guard_root':o.guard_root,
            'model_dir':o.model_dir,'actor_sha256':sha(package/'bin/q1-edge'),
            'package_sha256':sha(package/'SHA256SUMS'),'local_build_verified':True,
            'shadow_only':state['shadow_only'],'deployment':state['deployment'],
            'guard_fault_preserved':state['guard_fault'],'model_actuator_active':state['model_actuator_active'],
            'policy_steps':state['policy_steps'],'direct_console':f'http://{o.host}:8768'}
    o.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))


if __name__ == '__main__':
    main()
