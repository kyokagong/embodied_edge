"""Export the fixed release and compare every genuine golden sample. No ROS."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import time


def digest(p):
    with open(p, 'rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def main():
    import numpy as np
    import torch
    import onnx
    import onnxruntime as ort
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    src, out = args.source.resolve(), args.output.resolve()
    if out == src or src in out.parents:
        raise ValueError('Output must be separate from the immutable source package')
    snapshot = json.loads((src/'model/snapshot.json').read_text())
    assert digest(src/'model/policy.jit') == snapshot['jit_sha256']
    assert snapshot['iteration'] == 22100
    torch.set_num_threads(2)
    model = torch.jit.load(str(src/'model/policy.jit'), map_location='cpu').eval()
    data = np.load(src/'validation/golden_io.npz', allow_pickle=False)
    obs, expected = data['obs'], data['actions']
    assert obs.shape == (68, 1376) and expected.shape == (68, 22)
    out.mkdir(parents=True, exist_ok=True)
    path = out/'policy.onnx'
    torch.onnx.export(model, torch.from_numpy(obs[:1].copy()), str(path),
                      input_names=['obs'], output_names=['actions'],
                      opset_version=17, dynamo=False)
    onnx.checker.check_model(onnx.load(path))
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
    assert session.get_inputs()[0].shape == [1, 1376]
    assert session.get_outputs()[0].shape == [1, 22]
    actual = np.concatenate([session.run(['actions'], {'obs': x[None]})[0] for x in obs])
    np.testing.assert_allclose(actual, expected, rtol=3e-4, atol=3e-5)
    contract = json.loads((src/'deployment_contract.json').read_text())
    target = np.asarray(contract['action']['home'])+.25*np.clip(actual, -20, 20)
    np.testing.assert_allclose(target, data['q_targets'], rtol=3e-4, atol=3e-5)
    durations = []
    for i in range(1200):
        start = time.perf_counter_ns()
        session.run(['actions'], {'obs': obs[i % len(obs)][None]})
        if i >= 100:
            durations.append((time.perf_counter_ns()-start)/1e6)
    for f in ['deployment_contract.json', 'dance_catalog.json', 'DATA_PROVENANCE.md']:
        shutil.copy2(src/f, out/f)
    shutil.copytree(src/'references_50hz', out/'references_50hz', dirs_exist_ok=True)
    shutil.copy2(src/'model/snapshot.json', out/'source_snapshot.json')
    # Keep genuine inputs for Rust ORT parity, without requiring Torch online.
    (out/'golden.json').write_text(json.dumps({'obs': obs.tolist(), 'actions': expected.tolist(),
                                             'q_targets': data['q_targets'].tolist()}))
    files = {str(f.relative_to(out)): digest(f) for f in sorted(out.rglob('*'))
             if f.is_file() and f.name not in ['release.json', 'export-report.json']}
    report = {'iteration':22100, 'samples':68, 'max_action_abs_error':float(abs(actual-expected).max()),
              'latency_ms':dict(zip(['p50','p95','p99','max'],map(float,np.percentile(durations,[50,95,99,100])))),
              'torch':torch.__version__, 'onnx':onnx.__version__, 'onnxruntime':ort.__version__,
              'hardware_command_sent':False, 'motion_qualified':False}
    (out/'export-report.json').write_text(json.dumps(report, indent=2)+'\n')
    (out/'release.json').write_text(json.dumps({'schema':1,'iteration':22100,'onnx_sha256':digest(path),
       'source_jit_sha256':snapshot['jit_sha256'], 'files':files, 'opset':17,
       'input':[1,1376], 'output':[1,22], 'motion_qualified':False},indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
