"""Verify a locally built ARM64 release, then copy it to a versioned Q1 directory."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tarfile


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--package',type=Path,required=True)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--network-key',type=Path,required=True)
    p.add_argument('--ssh-key',type=Path,required=True)
    p.add_argument('--host',required=True)
    p.add_argument('--user',default='run')
    p.add_argument('--mini-ip',required=True)
    p.add_argument('--mode',choices=['readonly','air'],default='readonly')
    p.add_argument('--report',type=Path,required=True)
    o=p.parse_args();package=o.package.resolve()
    for line in (package/'SHA256SUMS').read_text().splitlines():
        expected,name=line.split(maxsplit=1)
        f=(package/name.lstrip('*')).resolve()
        if not f.is_relative_to(package) or sha(f)!=expected:
            raise ValueError('Package digest mismatch: '+name)
    for binary in ['q1-rt','q1-guard-net','q1-air-check']:
        data=(package/'bin'/binary).read_bytes()
        # ELF64 little endian, EM_AARCH64=183. No target program is run here.
        if data[:6]!=b'\x7fELF\x02\x01' or int.from_bytes(data[18:20],'little')!=183:
            raise ValueError('Not an ARM64 executable: '+binary)
    release_id=sha(package/'SHA256SUMS')[:12]
    remote=f'/home/{o.user}/q1-supermimic/guardian-{release_id}'
    archive=package.parent/f'guardian-{release_id}.tar.gz'
    with tarfile.open(archive,'w:gz') as tar:
        tar.add(package,arcname='.')
    connection=['-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=3',
        '-o','ServerAliveCountMax=3','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes','-i',str(o.ssh_key)]
    destination=f'{o.user}@{o.host}'
    def ssh(command):
        return subprocess.run(['ssh',*connection,destination,command],check=True,text=True)
    def copy(source,target):
        subprocess.run(['scp','-q',*connection,str(source),destination+':'+target],check=True)
    q=shlex.quote
    ssh(f'mkdir -p {q(remote)}/config {q(remote)}/private; chmod 700 {q(remote)}/private')
    copy(archive,remote+'.tar.gz')
    copy(o.profile,remote+'/config/hardware.json')
    copy(o.network_key,remote+'/private/network-key')
    ssh(f'tar -xzf {q(remote)}.tar.gz -C {q(remote)}; cd {q(remote)}; sha256sum -c SHA256SUMS >/dev/null && chmod 600 private/network-key && ./run q1-rt --help >/dev/null && ./run q1-guard-net --help >/dev/null && ./start {q(o.mode)} {q(o.mini_ip)}')
    o.report.write_text(json.dumps({'remote_directory':remote,'host':o.host,'mode':o.mode,
        'build_location':str(package),'rt_sha256':sha(package/'bin/q1-rt'),
        'network_sha256':sha(package/'bin/q1-guard-net'),'profile_sha256':sha(o.profile),
        'uploaded_after_local_verification':True},indent=2)+'\n')
    print(json.dumps({'remote_directory':remote,'mode':o.mode}))


if __name__=='__main__':
    main()
