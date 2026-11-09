"""Real filesystem/process tests; no robot feedback or hardware acceptance is implied."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'deploy/policy/manage.py'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def seal(root):
    paths = sorted(p for p in root.rglob('*') if p.is_file() and p.name != 'SHA256SUMS')
    (root / 'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.relative_to(root).as_posix()}\n' for p in paths))


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(MODULE.exists(), 'policy package/deployment module is missing')
        spec = importlib.util.spec_from_file_location('policy_manage', MODULE)
        self.m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.m)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def source(self):
        src = self.root / 'source'
        files = {
            'model/policy_native.onnx': b'graph',
            'model/checkpoint.pt': b'private-training-state',
            'deployment_contract.json': json.dumps({'models': {'policy_native.onnx': {'sha256': hashlib.sha256(b'graph').hexdigest()}}}).encode(),
            'references/manifest.json': json.dumps({'clips': {'standing': {'file': 'standing.npz', 'npz_sha256': hashlib.sha256(b'reference').hexdigest()}}}).encode(),
            'references/standing.npz': b'reference',
            'validation/golden.npz': b'golden',
            'validation/cases.json': b'[]',
            'LICENSE': b'license',
            'THIRD_PARTY_NOTICES.md': b'notices',
            'provenance/prone_1000_summary.json': b'{}',
        }
        for name, data in files.items():
            p = src / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(data)
        seal(src)
        return src

    def test_extracts_verified_runtime_subset_without_training_checkpoint(self):
        src = self.source()
        target = self.m.prepare_model(src, self.root / 'models')
        self.assertEqual((target / 'model/policy_native.onnx').read_bytes(), b'graph')
        self.assertFalse((target / 'model/checkpoint.pt').exists())
        self.assertTrue((target / 'provenance/prone_1000_summary.json').is_file())
        self.assertEqual((target / 'notices/source-SHA256SUMS').read_bytes(), (src / 'SHA256SUMS').read_bytes())
        self.assertEqual(target.name, digest(target / 'SHA256SUMS'))
        self.assertEqual(self.m.prepare_model(src, self.root / 'models'), target)
        self.m.verify_package(target)

    def test_full_source_digest_failure_prevents_any_model_extraction(self):
        src = self.source()
        (src / 'model/checkpoint.pt').write_bytes(b'corrupted unused checkpoint')
        with self.assertRaisesRegex(ValueError, 'digest'):
            self.m.prepare_model(src, self.root / 'models')
        self.assertFalse((self.root / 'models').exists())

    def test_refuses_traversal_duplicate_unlisted_and_symlink_files(self):
        for kind in ['traversal', 'duplicate', 'unlisted', 'symlink']:
            with self.subTest(kind=kind):
                src = self.source()
                sums = src / 'SHA256SUMS'
                if kind == 'traversal':
                    sums.write_text('0' * 64 + '  ../outside\n')
                elif kind == 'duplicate':
                    sums.write_text(sums.read_text() * 2)
                elif kind == 'unlisted':
                    (src / 'extra').write_text('not checked')
                else:
                    (src / 'model/policy_native.onnx').unlink()
                    (src / 'model/policy_native.onnx').symlink_to(src / 'model/checkpoint.pt')
                with self.assertRaises(ValueError):
                    self.m.verify_package(src)
                shutil.rmtree(src)

    def test_checksums_work_on_target_python_without_file_digest(self):
        from unittest.mock import patch
        file = self.root / 'abc'
        file.write_bytes(b'abc')
        with patch.object(hashlib, 'file_digest', None):
            self.assertEqual(self.m.sha(file), 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad')

    def test_declared_reference_must_match_manifest_and_be_checked(self):
        src = self.source()
        (src / 'references/standing.npz').write_bytes(b'wrong reference')
        seal(src)
        with self.assertRaisesRegex(ValueError, 'reference'):
            self.m.prepare_model(src, self.root / 'models')


class FrontendTests(unittest.TestCase):
    setUp = PackageTests.setUp
    source = PackageTests.source

    def test_offline_deployment_plan_does_not_execute_ssh(self):
        script = ROOT / 'scripts/deploy_policy.py'
        self.assertTrue(script.exists(), 'offline deployment planner is missing')
        model = self.m.prepare_model(self.source(), self.root / 'models')
        # Runtime is an integrity-checked package here; no target execution is
        # possible during --dry-run, including validation or SSH discovery.
        result = subprocess.run(['python3', str(script), '--dry-run', '--runtime', str(model),
            '--model', str(model), '--previous-deployment', str(self.root / 'old.json'),
            '--host', 'q1.invalid', '--report', str(self.root / 'report.json')], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        plan = json.loads(result.stdout)
        self.assertEqual(plan['operation'], 'deploy')
        self.assertTrue(plan['target_preflight_required'])
        self.assertIn('/home/run/q1-policy/runtimes/', plan['runtime_root'])
        self.assertFalse((self.root / 'report.json').exists())

    def test_embedded_remote_program_has_no_startup_side_effect(self):
        self.assertTrue(hasattr(self.m, 'remote_program'), 'isolated remote program builder is missing')
        incoming = self.root / 'incoming'
        incoming.mkdir()
        request = {'incoming': str(incoming), 'archives': {}, 'port': 8768, 'operation': 'unknown'}
        result = subprocess.run(['python3', '-'], input=self.m.remote_program(request), text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)['ok'])
        self.assertFalse(incoming.exists())

    def test_build_help_needs_no_ros_environment_or_network(self):
        script = ROOT / 'scripts/build-policy-arm64.sh'
        self.assertTrue(script.exists(), 'offline ARM64 build entrypoint is missing')
        result = subprocess.run(['bash', str(script), '--help'], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--package-only', result.stdout)

    def test_arm64_dependency_audit_refuses_host_elf(self):
        script = ROOT / 'deploy/policy/validate_arm64.py'
        self.assertTrue(script.exists(), 'portable policy ARM64 verifier is missing')
        package = self.root / 'host-runtime'
        (package / 'bin').mkdir(parents=True)
        shutil.copyfile('/bin/true', package / 'bin/q1-policy')
        report = self.root / 'closure.json'
        result = subprocess.run(['python3', str(script), '--package', str(package),
            '--sysroot', str(self.root), '--report', str(report)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Wrong ELF architecture', result.stderr)
        self.assertFalse(report.exists())

    def test_unpack_rejects_archive_escape_and_links(self):
        import io, tarfile
        self.assertTrue(hasattr(self.m, 'extract_archive'), 'safe archive extraction is missing')
        for kind in ['../escape', '/absolute', 'symlink']:
            with self.subTest(kind=kind):
                archive = self.root / 'bad.tar.gz'
                with tarfile.open(archive, 'w:gz') as stream:
                    entry = tarfile.TarInfo(kind)
                    if kind == 'symlink':
                        entry.type = tarfile.SYMTYPE
                        entry.linkname = '/tmp'
                        stream.addfile(entry)
                    else:
                        entry.size = 1
                        stream.addfile(entry, io.BytesIO(b'x'))
                with self.assertRaises(ValueError):
                    self.m.extract_archive(archive, self.root / ('out-' + kind.replace('/', '-')))


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        PackageTests.setUp(self)
        self.assertTrue(hasattr(self.m, 'Backend'), 'task-scoped deployment backend is missing')
        import socket
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            self.port = sock.getsockname()[1]
        self.driver = self.root / 'driver.so'
        self.driver.write_bytes(b'pinned driver')
        self.hardware = self.root / 'firmware.yaml'
        self.hardware.write_bytes(b'pinned hardware')
        import os
        allowed = sorted(os.sched_getaffinity(0))
        self.affinity = {'rt': [allowed[0]], 'policy': allowed[-2:]}
        fixture_scope = str(self.root)
        class FixtureBackend(self.m.Backend):
            def process_commands(inner):
                # The process inventory is the external boundary. All lifecycle,
                # /proc identity, filesystem, signals and HTTP checks remain real.
                for pid, command in super().process_commands():
                    if any(token.startswith(fixture_scope + '/') for token in command):
                        yield pid, command
        self.backend = FixtureBackend(self.root / 'installed', port=self.port,
                                      hardware_paths=(self.driver, self.hardware), health_timeout=1.5, affinity=self.affinity)
        self.processes = []
        self.addCleanup(self.cleanup_processes)

    def test_cli_preserves_selected_existing_base_and_updates_follow_report(self):
        import sys
        runtime=self.runtime();model=self.model();base='/home/run/q1-supermimic'
        args=[sys.executable,str(ROOT/'scripts/deploy_policy.py'),'--runtime',str(runtime),'--model',str(model),'--base',base,'--cold-start','--host','192.168.54.41','--report',str(self.root/'report.json'),'--dry-run']
        result=subprocess.run(args,text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr)
        plan=json.loads(result.stdout);self.assertTrue(plan['runtime_root'].startswith(base+'/runtimes/'));self.assertTrue(plan['model_root'].startswith(base+'/models/'))
        previous=self.root/'previous.json';previous.write_text(json.dumps({'base':base,'host':'192.168.54.41'}))
        args=[sys.executable,str(ROOT/'scripts/update_policy_model.py'),'--model',str(model),'--previous-deployment',str(previous),'--host','192.168.54.41','--report',str(self.root/'new.json'),'--dry-run']
        result=subprocess.run(args,text=True,capture_output=True);self.assertEqual(result.returncode,0,result.stderr)
        self.assertTrue(json.loads(result.stdout)['model_root'].startswith(base+'/models/'))

    def cleanup_processes(self):
        import os, signal, time
        for pid in set(self.processes) | set(self.backend.children):
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for child in self.backend.children.values():
            child.wait(timeout=3)

    def runtime(self):
        root = self.root / 'runtime'
        files = {
            'config/hardware.json': json.dumps({'stand_trial': {'hardware_config_sha256': digest(self.hardware)}}),
            'notices/stand-hardware-map.json': json.dumps({'hal_command_abi': {'driver_sha256': digest(self.driver)}}),
            'run': '#!/bin/sh\nroot="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"\nprogram="$1"; shift\nexec "$root/bin/$program" "$@"\n',
            'bin/q1-rt': '#!/usr/bin/env python3\nimport sys,time,threading\nif "--validate-only" not in sys.argv:\n threading.Thread(target=lambda: time.sleep(120),daemon=True).start()\n while True: time.sleep(.1)\n',
            'bin/q1-policy': POLICY_FIXTURE,
        }
        for name, text in files.items():
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
            if name == 'run' or name.startswith('bin/'):
                path.chmod(0o755)
        seal(root)
        return root

    def model(self, content=b'graph'):
        import uuid
        path = self.root / ('model-' + uuid.uuid4().hex)
        (path / 'model').mkdir(parents=True)
        (path / 'model/policy_native.onnx').write_bytes(content)
        (path / 'deployment_contract.json').write_text('{}')
        seal(path)
        return path

    def deploy(self):
        report = self.m.deploy(self.runtime(), self.model(), None, self.backend)
        self.processes.extend([report['rt_pid'], report['policy_pid']])
        return report

    def test_initial_deploy_stages_both_packages_and_starts_disabled(self):
        report = self.deploy()
        self.assertIn('/runtimes/', report['runtime_root'])
        self.assertIn('/models/', report['model_root'])
        self.assertEqual(Path(report['active_model']).resolve(), Path(report['model_root']))
        self.assertFalse(report['initial_state']['enabled'])
        self.assertTrue(report['initial_state']['bridge']['released'])
        self.assertEqual(report['rt_sha256'], digest(Path(report['runtime_root']) / 'bin/q1-rt'))

    def test_rt_middleware_threads_keep_parent_cpus_while_only_control_thread_is_pinned(self):
        import os
        parent=set(os.sched_getaffinity(0));report=self.deploy();pid=report['rt_pid']
        self.assertEqual(set(os.sched_getaffinity(pid)),set(self.affinity['rt']))
        workers=[int(p.name) for p in Path('/proc',str(pid),'task').iterdir() if int(p.name)!=pid]
        self.assertTrue(workers,'fixture must create a real middleware-like thread')
        for tid in workers:self.assertEqual(set(os.sched_getaffinity(tid)),parent)

    def test_processes_receive_configured_affinity_and_rt_priority(self):
        import os
        # Test CPUs come from this host's allowed cpuset; target defaults are
        # RT CPU 7 and policy CPUs 4,5 on the eight-core Q1.
        self.backend.affinity = self.affinity
        report = self.deploy()
        self.assertEqual(os.sched_getaffinity(report['rt_pid']), set(self.affinity['rt']))
        self.assertEqual(os.sched_getaffinity(report['policy_pid']), set(self.affinity['policy']))
        command = report['rt_process']['cmdline']
        self.assertEqual(command[command.index('--rt-priority') + 1], '20')

    def test_model_update_keeps_rt_and_switches_only_model(self):
        old = self.deploy()
        new = self.m.update_model(self.model(b'new-graph'), old, self.backend)
        self.processes.append(new['policy_pid'])
        self.assertEqual(new['rt_pid'], old['rt_pid'])
        self.assertEqual(new['rt_process'], old['rt_process'])
        self.assertEqual(new['runtime_root'], old['runtime_root'])
        self.assertEqual(new['profile_sha256'], old['profile_sha256'])
        self.assertNotEqual(new['model_sha256'], old['model_sha256'])
        self.assertTrue(Path(old['model_root']).is_dir())
        self.assertEqual(Path(new['active_model']).resolve(), Path(new['model_root']))

    def test_new_model_preflight_failure_leaves_old_service_and_active_path(self):
        old = self.deploy()
        with self.assertRaisesRegex(ValueError, 'preflight'):
            self.m.update_model(self.model(b'preflight-fail'), old, self.backend)
        self.assertEqual(Path(old['active_model']).resolve(), Path(old['model_root']))
        self.assertEqual(self.backend.process(old['policy_pid']), old['policy_process'])
        self.assertEqual(self.backend.process(old['rt_pid']), old['rt_process'])

    def test_failed_new_policy_health_rolls_back_model_and_policy_with_same_rt(self):
        old = self.deploy()
        with self.assertRaises(self.m.UpdateFailure) as caught:
            self.m.update_model(self.model(b'health-fail'), old, self.backend)
        restored = caught.exception.report
        self.processes.append(restored['policy_pid'])
        self.assertEqual(restored['model_root'], old['model_root'])
        self.assertEqual(restored['rt_process'], old['rt_process'])
        self.assertEqual(Path(old['active_model']).resolve(), Path(old['model_root']))
        self.assertFalse(restored['initial_state']['enabled'])

    def test_new_policy_exit_during_health_still_restores_old_model(self):
        old = self.deploy()
        with self.assertRaises(self.m.UpdateFailure) as caught:
            self.m.update_model(self.model(b'health-exit'), old, self.backend)
        restored = caught.exception.report
        self.processes.append(restored['policy_pid'])
        self.assertEqual(restored['model_root'], old['model_root'])
        self.assertEqual(restored['rt_process'], old['rt_process'])
        self.assertFalse(restored['initial_state']['enabled'])

    def test_legacy_migration_stops_only_reported_task_processes(self):
        import time
        source = self.runtime()
        legacy = self.root / 'legacy'
        shutil.copytree(source, legacy)
        shutil.copyfile(legacy / 'bin/q1-policy', legacy / 'bin/q1-stand')
        (legacy / 'bin/q1-stand').chmod(0o755)
        (legacy / 'state').mkdir()
        model = self.model()
        children = []
        for program, args in [('q1-rt', ['--stand-trial']), ('q1-stand', ['--model-dir', str(model), '--hardware-profile', str(legacy / 'config/hardware.json'), '--port', str(self.port)])]:
            child = subprocess.Popen([str(legacy / 'run'), program, *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.backend.children[child.pid] = child
            self.processes.append(child.pid)
            children.append(child)
        (legacy / 'state/rt.pid').write_text(str(children[0].pid))
        (legacy / 'state/stand.pid').write_text(str(children[1].pid))
        outsider = subprocess.Popen(['sleep', '30'])
        self.backend.children[outsider.pid] = outsider
        self.processes.append(outsider.pid)
        until = time.monotonic() + 3
        while True:
            try:
                self.backend.released()
                break
            except OSError:
                if time.monotonic() > until: raise
                time.sleep(.02)
        previous = {'remote_directory': str(legacy), 'guard_root': str(legacy),
                    'stand_sha256': digest(legacy / 'bin/q1-stand'), 'rt_sha256': digest(legacy / 'bin/q1-rt'),
                    'profile_sha256': digest(legacy / 'config/hardware.json')}
        current = self.m.deploy(source, model, previous, self.backend)
        self.processes.extend([current['rt_pid'], current['policy_pid']])
        self.assertIsNone(outsider.poll())
        self.assertNotEqual(current['rt_pid'], children[0].pid)
        self.assertFalse(current['initial_state']['enabled'])

    def test_faults_blockers_or_stalled_hal_cannot_pass_update_health(self):
        old = self.deploy()
        for marker in [b'health-fault', b'health-blocker', b'health-stalled-hal']:
            with self.subTest(marker=marker):
                with self.assertRaises(self.m.UpdateFailure) as caught:
                    self.m.update_model(self.model(marker), old, self.backend)
                old = caught.exception.report
                self.processes.append(old['policy_pid'])
                self.assertEqual(Path(old['active_model']).resolve(), Path(old['model_root']))
                self.assertFalse(old['initial_state']['enabled'])

    def test_explicit_cold_start_refuses_unknown_live_writer(self):
        old = self.deploy()
        cold = type(self.backend)(self.root / 'cold', port=self.port + 1,
            hardware_paths=(self.driver, self.hardware), health_timeout=1.5, affinity=self.affinity)
        self.assertTrue(hasattr(cold, 'assert_no_writers'), 'safe cold start admission is missing')
        with self.assertRaisesRegex(ValueError, 'writer'):
            cold.assert_no_writers()
        self.assertEqual(self.backend.process(old['rt_pid']), old['rt_process'])

    def test_explicit_cold_start_accepts_stale_report_without_signalling(self):
        self.assertTrue(hasattr(self.backend, 'assert_no_writers'), 'safe cold start admission is missing')
        state = self.backend.base / 'state'
        state.mkdir(parents=True)
        (state / 'deployment.json').write_text('{"obsolete":true}')
        report = self.m.deploy(self.runtime(), self.model(), None, self.backend, cold_start=True)
        self.processes.extend([report['rt_pid'], report['policy_pid']])
        self.assertFalse(report['initial_state']['enabled'])

    def test_pid_reuse_or_hardware_change_cannot_stop_live_policy(self):
        old = self.deploy()
        wrong = dict(old)
        wrong['policy_process'] = dict(old['policy_process'], start_ticks='0')
        with self.assertRaisesRegex(ValueError, 'identity'):
            self.m.update_model(self.model(b'new-graph'), wrong, self.backend)
        self.driver.write_bytes(b'changed firmware driver')
        with self.assertRaisesRegex(ValueError, 'driver/config'):
            self.m.update_model(self.model(b'newer-graph'), old, self.backend)
        self.assertEqual(self.backend.process(old['policy_pid']), old['policy_process'])

    def test_enabled_or_unreleased_service_blocks_update_before_stop(self):
        old = self.deploy()
        (Path(old['model_root']).parent / 'enable-for-test').write_text('1')
        # The fixture server reads this external marker to simulate operator state.
        with self.assertRaisesRegex(ValueError, 'disabled and released'):
            self.m.update_model(self.model(b'new-graph'), old, self.backend)
        self.assertEqual(self.backend.process(old['policy_pid']), old['policy_process'])
        (Path(old['model_root']).parent / 'enable-for-test').unlink()


POLICY_FIXTURE = r'''#!/usr/bin/env python3
import hashlib,json,pathlib,sys,time
from http.server import BaseHTTPRequestHandler,HTTPServer
args=sys.argv[1:]
def arg(key):return args[args.index(key)+1]
model=pathlib.Path(arg('--model-dir'))
profile=pathlib.Path(arg('--hardware-profile'))
content=(model/'model/policy_native.onnx').read_bytes()
if '--check-only' in args:
 print(json.dumps({'ok': content != b'preflight-fail'}));sys.exit(0 if content != b'preflight-fail' else 1)
began=time.monotonic()
if content==b'health-exit':
 import threading,os
 threading.Timer(.25,lambda:os._exit(7)).start()
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  count=int((time.monotonic()-began)*100)+1
  enabled=(model.parent/'enable-for-test').exists()
  state={'controller':'q1_policy','enabled':enabled,'input_count':count,'model_count':0 if content in (b'health-fail',b'health-exit') else count,'model_sha256':hashlib.sha256(content).hexdigest(),'profile_sha256':hashlib.sha256(profile.read_bytes()).hexdigest(),'policy_protocol':2,'bridge':{'released':not enabled,'tx_count':0 if content==b'health-stalled-hal' else count,'controlled_joints':22,'fault':'driver fault' if content==b'health-fault' else None,'blockers':['bad feedback'] if content==b'health-blocker' else []}}
  self.send_response(200);self.end_headers();self.wfile.write(json.dumps(state).encode())
 def log_message(self,*args):pass
HTTPServer(('127.0.0.1',int(arg('--port'))),Handler).serve_forever()
'''


if __name__ == '__main__':
    unittest.main()
