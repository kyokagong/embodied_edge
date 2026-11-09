"""Deploy a locally verified numerical benchmark; never starts a control node."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tarfile


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--package', type=Path, required=True)
    p.add_argument('--model-dir', type=Path, required=True)
    p.add_argument('--ssh-key', type=Path, required=True)
    p.add_argument('--host', required=True)
    p.add_argument('--user', default='run')
    p.add_argument('--report-dir', type=Path, required=True)
    o = p.parse_args()
    package, model = o.package.resolve(), o.model_dir.resolve()
    for line in (package/'SHA256SUMS').read_text().splitlines():
        digest, name = line.split(maxsplit=1)
        path = (package/name).resolve()
        if not path.is_relative_to(package) or sha(path) != digest:
            raise ValueError('Package digest differs: '+name)
    release = json.loads((model/'release.json').read_text())
    if release['iteration'] != 22100:
        raise ValueError('Unexpected model iteration')
    for name, digest in release['files'].items():
        path = (model/name).resolve()
        if not path.is_relative_to(model) or sha(path) != digest:
            raise ValueError('Model digest differs: '+name)
    tag = sha(package/'SHA256SUMS')[:12]
    remote_base = f'/home/{o.user}/q1-supermimic'
    remote_tool = f'{remote_base}/tools/model-check-{tag}'
    remote_model = f'{remote_base}/models/iter22100-onnx'
    archive = package.parent/f'model-check-{tag}.tar.gz'
    with tarfile.open(archive, 'w:gz', compresslevel=1) as tar:
        tar.add(package, arcname=f'tools/model-check-{tag}')
        tar.add(model, arcname='models/iter22100-onnx')
    connection = ['-o','BatchMode=yes','-o','ConnectTimeout=8',
                  '-o','ServerAliveInterval=3','-o','ServerAliveCountMax=3',
                  '-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes','-i',str(o.ssh_key)]
    destination = f'{o.user}@{o.host}'
    q = shlex.quote
    def ssh(command):
        return subprocess.check_output(['ssh',*connection,destination,command], text=True)
    ssh(f'mkdir -p {q(remote_base)}')
    subprocess.run(['scp','-q',*connection,str(archive),f'{destination}:{remote_base}/model-check-{tag}.tar.gz'], check=True)
    ssh(f'tar -xzf {q(remote_base)}/model-check-{tag}.tar.gz -C {q(remote_base)} && cd {q(remote_tool)} && sha256sum -c SHA256SUMS >/dev/null && ./run --help >/dev/null')
    o.report_dir.mkdir(parents=True, exist_ok=True)
    for threads in [2, 1]:
        report = f'{remote_tool}/threads-{threads}.json'
        ssh(f'taskset -c 4,5 nice -n 10 {q(remote_tool)}/run --model-dir {q(remote_model)} --report {q(report)} --threads {threads} --samples 500 > {q(report)}.log 2>&1')
        data = json.loads(ssh(f'cat {q(report)}'))
        (o.report_dir/f'model-check-onboard-threads-{threads}.json').write_text(json.dumps(data,indent=2)+'\n')
        print(json.dumps({'threads':threads,'paced':data['paced'],'golden':data['golden']}, ensure_ascii=False), flush=True)
    (o.report_dir/'model-check-deployment.json').write_text(json.dumps({
        'host':o.host,'remote_tool':remote_tool,'remote_model':remote_model,
        'package_sha256':sha(package/'SHA256SUMS'),'model_release_sha256':sha(model/'release.json'),
        'no_ros_node_or_hal_command':True,'build_location':str(package)}, indent=2)+'\n')


if __name__ == '__main__':
    main()
