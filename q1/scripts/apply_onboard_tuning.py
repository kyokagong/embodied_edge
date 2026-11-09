"""Apply JSON tuning with the installed ARM64 package; never invokes a compiler."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request


def main():
    root=Path(__file__).resolve().parents[1]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=root/'config/supermimic-stand-trial.json')
    p.add_argument('--package',type=Path,default=root/'artifacts/stand/arm64')
    p.add_argument('--deployment',type=Path,default=root/'rust/reports/real-stand-deployment.json')
    p.add_argument('--ssh-key',type=Path,default=Path('/home/kyokagong/code/QY01BAL2610000105/id_ed25519_QY01BAL2610000105.lf'))
    p.add_argument('--host',default='192.168.54.41')
    o=p.parse_args();config=json.loads(o.config.read_text());before=json.loads(o.deployment.read_text())
    op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with op.open('http://'+o.host+':8768/api/state',timeout=3) as f:state=json.load(f)
    if state['enabled'] or not state['bridge']['released']:raise SystemExit('请先在UI解除接管，再应用参数；现有进程未改变')
    def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
    # Configuration application must not silently upgrade program code.
    if sha(o.package/'bin/q1-rt')!=before['rt_sha256'] or sha(o.package/'bin/q1-stand')!=before['stand_sha256']:
        raise SystemExit('本地二进制与已部署版本不同。先单独完成代码部署，再仅应用配置。')
    if before['profile_sha256']==sha(o.config):
        print(json.dumps({'changed':False,'reason':'configuration already loaded','compiled':False}));return
    backup=o.deployment.with_name('before-config-'+before['profile_sha256'][:12]+'.json')
    backup.write_text(json.dumps(before,ensure_ascii=False,indent=2)+'\n')
    args=[sys.executable,str(root/'scripts/deploy_real_stand.py'),'--package',str(o.package),'--hardware-profile',str(o.config.resolve()),'--ssh-key',str(o.ssh_key),'--host',o.host,'--old-edge-root',before['remote_directory'],'--old-guard-root',before['guard_root'],'--replace-real-rt-root',before['guard_root'],'--old-stand-root',before['remote_directory'],'--resume-released','--require-released','--publish-hz',str(state['bridge']['publish_hz']),'--report',str(o.deployment)]
    subprocess.run(args,check=True)
    after=json.loads(o.deployment.read_text())
    assert after['rt_sha256']==before['rt_sha256'] and after['stand_sha256']==before['stand_sha256']
    assert after['profile_sha256']==sha(o.config)
    print(json.dumps({'changed':True,'compiled':False,'program_sha256_unchanged':True,'profile_sha256':after['profile_sha256'],'effort_limits_nm':config['stand_trial']['effort_limit'],'explicit_ui_start_required':True},ensure_ascii=False))


if __name__=='__main__':main()
