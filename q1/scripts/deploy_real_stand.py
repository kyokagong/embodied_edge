"""Install locally built real-stand binaries, retire only this task's old writers."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import tempfile
import subprocess
import tarfile
import time
import urllib.request


def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--package',type=Path,required=True)
    p.add_argument('--audit-recorded-actions',type=Path,help='Replay real recorded model outputs before replacing any running process')
    p.add_argument('--hardware-profile',type=Path,help='Use this JSON with existing ARM64 binaries; no compilation')
    p.add_argument('--require-released',action='store_true',help='Refuse replacement unless current controller remains disabled and released')
    p.add_argument('--ssh-key',type=Path,required=True)
    p.add_argument('--old-edge-root',required=True)
    p.add_argument('--old-guard-root',required=True)
    p.add_argument('--host',default='192.168.54.41')
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--reuse-rt-root',help='Keep this existing real stand RT process running; replace its stand UI/controller only')
    p.add_argument('--replace-real-rt-root',help='Replace this real RT guard after a verified HAL ABI correction; resume measured support hold')
    p.add_argument('--resume-released',action='store_true',help='Keep zero-stiffness damping during a real RT replacement')
    p.add_argument('--publish-hz',type=int,choices=[50,500],default=500,help='Actual HAL publication rate; 50Hz is an explicit diagnostic trial')
    p.add_argument('--old-stand-root',help='Previous real stand UI/controller directory')
    o=p.parse_args();package=o.package.resolve()
    if o.resume_released and not o.replace_real_rt_root:raise ValueError('--resume-released needs real RT replacement')
    for line in (package/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split(maxsplit=1);f=(package/name).resolve()
        if not f.is_relative_to(package) or sha(f)!=digest:raise ValueError('Bad package digest: '+name)
    staging=None
    archive_parent=package.parent
    if o.hardware_profile:
        json.loads(o.hardware_profile.read_text())
        staging=tempfile.TemporaryDirectory(prefix='q1-config-')
        source=package;package=Path(staging.name)/'package';shutil.copytree(source,package,symlinks=True)
        shutil.copyfile(o.hardware_profile,package/'config/hardware.json')
        entries=[]
        for line in (source/'SHA256SUMS').read_text().splitlines():
            _,name=line.split(maxsplit=1);entries.append(sha(package/name)+'  '+name)
        (package/'SHA256SUMS').write_text('\n'.join(entries)+'\n')
    tag=sha(package/'SHA256SUMS')[:12];remote=f'/home/run/q1-supermimic/stand-{tag}'
    archive=archive_parent/f'stand-{tag}.tar.gz'
    with tarfile.open(archive,'w:gz',compresslevel=1) as tar:tar.add(package,arcname='.')
    args=['-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=3','-o','ServerAliveCountMax=3','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes','-i',str(o.ssh_key)]
    host='run@'+o.host;q=shlex.quote
    def ssh(cmd,**kw):return subprocess.run(['ssh',*args,host,cmd],text=True,check=True,**kw)
    # This installed firmware consumes a fixed positional HAL ABI. Refuse a
    # changed driver/config before replacing any process or sending commands.
    mapping=json.loads((package/'notices/stand-hardware-map.json').read_text())
    abi=mapping['hal_command_abi']
    expected_config=json.loads((package/'config/hardware.json').read_text())['stand_trial']['hardware_config_sha256']
    hashes=ssh('sha256sum /robot/software/device_manager/lib/libdevice_manager_pkg.so /robot/software/device_manager/bin/cfg/tmp/temp_cfg_file_for_MotorModule.yaml',capture_output=True).stdout.splitlines()
    if [line.split()[0] for line in hashes] != [abi['driver_sha256'],expected_config]:raise ValueError('Q1 driver/config changed: verify HAL ABI before deployment')
    if o.reuse_rt_root:
        active_profile=ssh('sha256sum '+q(o.reuse_rt_root+'/config/hardware.json'),capture_output=True).stdout.split()[0]
        if active_profile!=sha(package/'config/hardware.json'):raise ValueError('Reused guard profile differs; replace real RT explicitly')
    ssh(f'mkdir -p {q(remote)}/private; chmod 700 {q(remote)}/private')
    for source,target in [(archive,remote+'.tar.gz'),(Path(__file__).with_name('check_real_stand_onboard.py'),remote+'/check_real_stand.py')]:
        subprocess.run(['scp','-q',*args,str(source),host+':'+target],check=True)
    ssh(f'tar -xzf {q(remote)}.tar.gz -C {q(remote)} && cd {q(remote)} && sha256sum -c SHA256SUMS >/dev/null && ./run q1-rt --validate-only --stand-trial --hardware-profile config/hardware.json && ./run q1-stand --help >/dev/null')
    if o.audit_recorded_actions:
        target=remote+'/private/predeploy-real-action-audit.json'
        subprocess.run(['scp','-q',*args,str(o.audit_recorded_actions),host+':'+target],check=True)
        ssh('cd '+q(remote)+' && chmod 600 private/predeploy-real-action-audit.json && ./run q1-stand --model-dir /home/run/q1-supermimic/models/iter22100-onnx --hardware-profile config/hardware.json --socket state/audit-unused.sock --audit-recorded-actions private/predeploy-real-action-audit.json')
    if o.require_released:
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open('http://'+o.host+':8768/api/state',timeout=3) as f:current=json.load(f)
        if current['enabled'] or not current['bridge']['released']:raise ValueError('Please release robot before applying configuration; existing processes kept')
    if o.replace_real_rt_root:
        if o.reuse_rt_root or not o.old_stand_root:raise ValueError('RT replacement needs --old-stand-root and cannot reuse RT')
        script='''import pathlib,subprocess,time
old=pathlib.Path(%r);guard=pathlib.Path(%r);new=pathlib.Path(%r)
for root,role,marker in [(old,'stand',b'q1-stand'),(guard,'rt',b'--stand-trial')]:
 pid=int((root/'state'/f'{role}.pid').read_text());proc=pathlib.Path(f'/proc/{pid}/cmdline')
 assert proc.exists() and str(root).encode() in proc.read_bytes() and marker in proc.read_bytes()
 subprocess.run(['sudo','-n','kill','-TERM',str(pid)],check=True)
 until=time.monotonic()+5
 while proc.exists() and proc.read_bytes() and time.monotonic()<until:time.sleep(.05)
 assert not proc.exists() or not proc.read_bytes()
(new/'state').mkdir(exist_ok=True)
args=[str(new/'run'),'q1-rt','--hardware-profile',str(new/'config/hardware.json'),'--socket',str(new/'state/bridge.sock'),'--enable-hardware','--stand-trial',%r,'--rt-priority','20','--publish-hz',str(%r)]
with (new/'state/rt.log').open('w') as log:
 subprocess.Popen(['sudo','-n','bash','-c','echo $$ > "$1"; shift; exec "$@"','stand',str(new/'state/rt.pid'),*args],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
time.sleep(1)
subprocess.run(['sudo','-n','taskset','-pc','7',(new/'state/rt.pid').read_text().strip()],check=True)
args=['taskset','-c','4,5',str(new/'run'),'q1-stand','--model-dir','/home/run/q1-supermimic/models/iter22100-onnx','--hardware-profile',str(new/'config/hardware.json'),'--socket',str(new/'state/bridge.sock'),'--private-trace-file',str(new/'private/model-trace.json'),'--port','8768']
with (new/'state/stand.log').open('w') as log:
 subprocess.Popen(['sudo','-n','bash','-c','echo $$ > "$1"; shift; exec "$@"','stand',str(new/'state/stand.pid'),*args],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
print('Replaced only this task real RT/controller, resuming measured support with corrected HAL slots')
'''%(o.old_stand_root,o.replace_real_rt_root,remote,'--resume-damping' if o.resume_released else '--resume-supported-hold',o.publish_hz)
        ssh('python3 -',input=script)
    elif o.reuse_rt_root:
        if not o.old_stand_root:raise ValueError('--old-stand-root required')
        script='''import pathlib,subprocess,time
old=pathlib.Path(%r);guard=pathlib.Path(%r)
rt=int((guard/'state/rt.pid').read_text());cmd=pathlib.Path(f'/proc/{rt}/cmdline').read_bytes();assert str(guard).encode() in cmd and b'--stand-trial' in cmd
pid=int((old/'state/stand.pid').read_text());proc=pathlib.Path(f'/proc/{pid}/cmdline')
if proc.exists() and proc.read_bytes():
 assert str(old).encode() in proc.read_bytes() and b'q1-stand' in proc.read_bytes()
 subprocess.run(['sudo','-n','kill','-TERM',str(pid)],check=True)
 until=time.monotonic()+5
 while proc.exists() and proc.read_bytes() and time.monotonic()<until:time.sleep(.1)
 assert not proc.exists() or not proc.read_bytes()
print('Kept existing real RT guard PID',rt)
'''%(o.old_stand_root,o.reuse_rt_root)
        ssh('python3 -',input=script)
        command=['taskset','-c','4,5',remote+'/run','q1-stand','--model-dir','/home/run/q1-supermimic/models/iter22100-onnx','--hardware-profile',o.reuse_rt_root+'/config/hardware.json','--socket',o.reuse_rt_root+'/state/bridge.sock','--private-trace-file',remote+'/private/model-trace.json','--port','8768']
        script='''import pathlib,subprocess
r=pathlib.Path(%r);(r/'state').mkdir(exist_ok=True)
args=%r
with (r/'state/stand.log').open('w') as log:
 subprocess.Popen(['sudo','-n','bash','-c','echo $$ > "$1"; shift; exec "$@"','stand',str(r/'state/stand.pid'),*args],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
'''%(remote,command)
        ssh('python3 -',input=script)
    else:
        script='''import pathlib,subprocess,time,json
edge=pathlib.Path(%r);guard=pathlib.Path(%r)
for root,role,marker in [(edge,'edge',b'--shadow-only'),(guard,'rt',b'--commissioning')]:
 f=root/'state'/f'{role}.pid'
 if f.exists():
  pid=int(f.read_text());proc=pathlib.Path(f'/proc/{pid}/cmdline')
  if proc.exists() and proc.read_bytes():
   cmd=proc.read_bytes();assert str(root).encode() in cmd and marker in cmd
   subprocess.run(['sudo','-n','kill','-TERM',str(pid)],check=True)
   until=time.monotonic()+5
   while proc.exists() and proc.read_bytes() and time.monotonic()<until:time.sleep(.1)
   assert not proc.exists() or not proc.read_bytes()
print('Retired only old shadow actor and old single-elbow pilot; manufacturer processes unchanged')
'''%(o.old_edge_root,o.old_guard_root)
        ssh('python3 -',input=script)
        ssh(f'cd {q(remote)} && ./start /home/run/q1-supermimic/models/iter22100-onnx')
    op=urllib.request.build_opener(urllib.request.ProxyHandler({}));until=time.monotonic()+100
    while time.monotonic()<until:
        try:
            with op.open('http://'+o.host+':8768/api/state',timeout=3) as f:s=json.load(f)
            if s['controller']=='supermimic_stand' and s['input_count']>0 and s['bridge']['controlled_joints']==22:break
        except (OSError,ValueError):pass
        time.sleep(.5)
    else:raise RuntimeError('Stand service not ready; inspect state logs')
    guard_root=o.reuse_rt_root or remote
    actual_rt_sha=ssh('sha256sum '+q(guard_root+'/bin/q1-rt'),capture_output=True).stdout.split()[0]
    o.report.write_text(json.dumps({'remote_directory':remote,'guard_root':guard_root,'controller':'supermimic_stand','rt_sha256':actual_rt_sha,'packaged_rt_sha256':sha(package/'bin/q1-rt'),'stand_sha256':sha(package/'bin/q1-stand'),'profile_sha256':sha(package/'config/hardware.json'),'explicit_ui_stand_required':True,'initial_state':s},ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'remote_directory':remote,'phase':s['phase'],'hal_frames_before_arm':s['bridge']['tx_count']},ensure_ascii=False))


if __name__=='__main__':main()
