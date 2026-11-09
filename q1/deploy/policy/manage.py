#!/usr/bin/env python3
"""Checksum packages and task-scoped policy lifecycle. Python stdlib only.

This module is sent verbatim over SSH by the frontends. Packaging never imports
ROS or starts a process. A deployment starts disabled and needs a new UI action.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import subprocess
import tarfile
import tempfile
import time
import urllib.request


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def relative(name):
    path = PurePosixPath(name)
    if not name or path.is_absolute() or '..' in path.parts or '.' in path.parts or path.as_posix() != name or '\\' in name:
        raise ValueError('unsafe package path: ' + name)
    return path


def verify_package(root):
    root = Path(root).resolve()
    sums = root / 'SHA256SUMS'
    if sums.is_symlink() or not sums.is_file():
        raise ValueError('missing checksum manifest')
    entries = {}
    for line in sums.read_text().splitlines():
        match = re.fullmatch(r'([0-9a-f]{64})  (.+)', line)
        if not match:
            raise ValueError('malformed digest line')
        digest, name = match.groups()
        relative(name)
        if name == 'SHA256SUMS' or name in entries:
            raise ValueError('duplicate or self-referential digest: ' + name)
        path = root / name
        if any(p.is_symlink() for p in [path, *path.parents] if p != root.parent):
            raise ValueError('symlink package path: ' + name)
        if not path.is_file() or sha(path) != digest:
            raise ValueError('bad package digest: ' + name)
        entries[name] = digest
    actual = set()
    for path in root.rglob('*'):
        if path.is_symlink():
            raise ValueError('symlink in package: ' + str(path))
        if path.is_file() and path != sums:
            actual.add(path.relative_to(root).as_posix())
        elif not path.is_dir() and path != sums:
            raise ValueError('nonregular package entry: ' + str(path))
    if not entries or actual != set(entries):
        raise ValueError('unlisted or missing package file')
    return entries


def seal(root):
    root = Path(root)
    names = sorted(p.relative_to(root).as_posix() for p in root.rglob('*') if p.is_file() and p != root / 'SHA256SUMS')
    (root / 'SHA256SUMS').write_text(''.join(sha(root / name) + '  ' + name + '\n' for name in names))
    verify_package(root)
    return sha(root / 'SHA256SUMS')


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp-' + str(os.getpid()))
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    os.replace(temporary, path)


def install_immutable(source, root):
    """Fully verify before copying, then atomically install by manifest digest."""
    source, root = Path(source).resolve(), Path(root)
    verify_package(source)
    identity = sha(source / 'SHA256SUMS')
    target = root / identity
    root.mkdir(parents=True, exist_ok=True)
    if target.exists():
        verify_package(target)
        if sha(target / 'SHA256SUMS') != identity:
            raise ValueError('immutable package identity changed')
        return target
    with tempfile.TemporaryDirectory(prefix='.stage-', dir=root) as temporary:
        staged = Path(temporary) / 'package'
        shutil.copytree(source, staged)
        verify_package(staged)
        if sha(staged / 'SHA256SUMS') != identity:
            raise ValueError('package changed during staging')
        os.rename(staged, target)
    return target


def prepare_model(source, output_root):
    source = Path(source).resolve()
    entries = verify_package(source)  # Include unused training files in this check.
    contract = json.loads((source / 'deployment_contract.json').read_text())
    native = 'model/policy_native.onnx'
    if entries.get(native) != contract['models']['policy_native.onnx']['sha256']:
        raise ValueError('native model does not match contract')
    manifest = json.loads((source / 'references/manifest.json').read_text())
    names = {native, 'deployment_contract.json', 'references/manifest.json', 'validation/golden.npz', 'validation/cases.json', 'LICENSE', 'THIRD_PARTY_NOTICES.md'}
    if not manifest.get('clips'):
        raise ValueError('reference manifest has no clips')
    for clip in manifest['clips'].values():
        file = relative(clip['file'])
        if len(file.parts) != 1 or file.suffix != '.npz':
            raise ValueError('unsafe reference file')
        name = 'references/' + str(file)
        if entries.get(name) != clip['npz_sha256']:
            raise ValueError('reference digest mismatch: ' + name)
        names.add(name)
    names.update(name for name in entries if name.startswith('provenance/'))
    names.update(name for name in ['CITATION.cff', 'README_部署指南.md'] if name in entries)
    for name in names:
        if name not in entries:
            raise ValueError('required runtime model file is unchecked: ' + name)
    with tempfile.TemporaryDirectory(prefix='q1-policy-model-') as temporary:
        staged = Path(temporary)
        for name in sorted(names):
            target = staged / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source / name, target)
        (staged / 'notices').mkdir(exist_ok=True)
        shutil.copyfile(source / 'SHA256SUMS', staged / 'notices/source-SHA256SUMS')
        atomic_json(staged / 'model_package.json', {'schema': 'q1-policy-model-package-v1', 'source_manifest_sha256': sha(source / 'SHA256SUMS'), 'model_sha256': entries[native], 'contract_sha256': entries['deployment_contract.json'], 'numerical_preflight_required': True})
        seal(staged)
        return install_immutable(staged, output_root)


def linux_syscall(number, *arguments):
    import ctypes
    import platform
    if platform.machine() not in ('x86_64', 'aarch64'):
        raise RuntimeError('pidfd syscall fallback supports x86_64/aarch64 only')
    libc = ctypes.CDLL(None, use_errno=True)
    libc.syscall.restype = ctypes.c_long
    value = libc.syscall(number, *arguments)
    if value < 0:
        number = ctypes.get_errno()
        raise OSError(number, os.strerror(number))
    return value


def open_pidfd(pid):
    if hasattr(os, 'pidfd_open'):
        return os.pidfd_open(pid)
    return linux_syscall(434, pid, 0)


def terminate_pidfd(descriptor):
    if hasattr(signal, 'pidfd_send_signal'):
        signal.pidfd_send_signal(descriptor, signal.SIGTERM)
    else:
        import ctypes
        linux_syscall(424, descriptor, signal.SIGTERM, ctypes.c_void_p(), 0)


class UpdateFailure(RuntimeError):
    def __init__(self, message, report):
        super().__init__(message)
        self.report = report


class Backend:
    """Local target operations; SSH frontends run this exact code on the target."""
    def __init__(self, base=Path('/home/run/q1-policy'), port=8768,
                 hardware_paths=(Path('/robot/software/device_manager/lib/libdevice_manager_pkg.so'), Path('/robot/software/device_manager/bin/cfg/tmp/temp_cfg_file_for_MotorModule.yaml')),
                 health_timeout=30, affinity=None):
        self.base = Path(base).resolve()
        self.port = int(port)
        self.hardware_paths = tuple(Path(p) for p in hardware_paths)
        self.health_timeout = health_timeout
        self.children = {}
        self.affinity = affinity if affinity is not None else {'rt': [7], 'policy': [4, 5]}

    def lock(self):
        import contextlib
        import fcntl
        @contextlib.contextmanager
        def locked():
            self.base.mkdir(parents=True, exist_ok=True)
            with (self.base / '.deployment.lock').open('a') as stream:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                yield
        return locked()

    def process(self, pid):
        pid = int(pid)
        if pid <= 1:
            raise ValueError('invalid process identity')
        root = Path('/proc') / str(pid)
        try:
            fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                raise ProcessLookupError(pid)
            command = (root / 'cmdline').read_bytes().split(b'\0')
            if not command or command == [b'']:
                raise ProcessLookupError(pid)
            return {'pid': pid, 'start_ticks': fields[19], 'cmdline': [os.fsdecode(x) for x in command if x], 'exe': os.readlink(root / 'exe')}
        except FileNotFoundError as error:
            raise ProcessLookupError(pid) from error

    def assert_process(self, report, role):
        expected = report[role + '_process']
        if self.process(report[role + '_pid']) != expected:
            raise ValueError(role + ' process identity changed; refusing signal')
        binary = Path(report[role + '_root']) / 'bin' / report[role + '_program']
        if str(binary) not in expected['cmdline'] or sha(binary) != report[role + '_sha256']:
            raise ValueError(role + ' process identity/binary mismatch')
        if role == 'rt' and '--stand-trial' not in expected['cmdline']:
            raise ValueError('RT process identity is not this stand task')
        return expected

    def stop(self, report, role, missing_ok=False):
        try:
            expected = self.assert_process(report, role)
        except ProcessLookupError:
            child = self.children.pop(report[role + '_pid'], None)
            if child:
                child.wait(timeout=1)
            if missing_ok:
                return
            raise
        pid = expected['pid']
        # pidfd prevents a PID reused between inspection and signalling receiving
        # the signal. Linux targets without pidfd must be upgraded, not weakened.
        descriptor = open_pidfd(pid)
        try:
            self.assert_process(report, role)
            terminate_pidfd(descriptor)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                try:
                    if self.process(pid) != expected:
                        break
                except ProcessLookupError:
                    break
                time.sleep(.05)
            else:
                raise RuntimeError(role + ' did not exit after TERM; no forced kill used')
        finally:
            os.close(descriptor)
        child = self.children.pop(pid, None)
        if child:
            child.wait(timeout=1)

    def hardware(self, runtime):
        mapping = json.loads((runtime / 'notices/stand-hardware-map.json').read_text())
        profile = json.loads((runtime / 'config/hardware.json').read_text())
        expected = [mapping['hal_command_abi']['driver_sha256'], profile['stand_trial']['hardware_config_sha256']]
        if [sha(path) for path in self.hardware_paths] != expected:
            raise ValueError('Q1 driver/config changed; verify HAL ABI before deployment')
        return {'driver_sha256': expected[0], 'hardware_config_sha256': expected[1]}

    def preflight(self, runtime, model):
        verify_package(runtime)
        verify_package(model)
        profile = runtime / 'config/hardware.json'
        subprocess.run([str(runtime / 'run'), 'q1-rt', '--validate-only', '--stand-trial', '--policy-v2', '--hardware-profile', str(profile)], check=True, capture_output=True, text=True, timeout=60)
        result = subprocess.run([str(runtime / 'run'), 'q1-policy', '--check-only', '--model-dir', str(model), '--hardware-profile', str(profile)], capture_output=True, text=True, timeout=120)
        try:
            checked = json.loads(result.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            checked = {}
        if result.returncode or checked.get('ok') is not True:
            raise ValueError('model preflight failed: ' + (result.stderr.strip() or result.stdout.strip())[-4000:])
        return checked

    def assert_no_writers(self):
        import socket
        try:
            connection = socket.create_connection(('127.0.0.1', self.port), timeout=.3)
        except ConnectionRefusedError:
            pass
        else:
            connection.close()
            raise ValueError('cold start refused: API port already has a writer/service')
        for pid, command in self.process_commands():
            if '--check-only' in command or '--validate-only' in command:
                continue
            if any(Path(token).name in ('q1-policy', 'q1-rt', 'q1-stand') for token in command):
                raise ValueError('cold start refused: existing task writer PID ' + str(pid))

    def process_commands(self):
        """Read the target process inventory; isolated tests scope this boundary."""
        for directory in Path('/proc').glob('[0-9]*'):
            try:
                command = [os.fsdecode(token) for token in (directory / 'cmdline').read_bytes().split(b'\0') if token]
            except (ProcessLookupError, FileNotFoundError):
                continue
            except PermissionError as error:
                raise ValueError('cannot inspect existing processes; use explicit privileged deployment') from error
            yield int(directory.name), command

    def state(self):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open('http://127.0.0.1:' + str(self.port) + '/api/state', timeout=2) as stream:
            return json.load(stream)

    def released(self):
        state = self.state()
        if state.get('enabled') is not False or state.get('bridge', {}).get('released') is not True:
            raise ValueError('current policy must remain disabled and released; processes kept')
        return state

    def switch(self, model):
        link = self.base / 'active-model'
        if link.exists() and not link.is_symlink():
            raise ValueError('active-model must be a symlink')
        temporary = self.base / ('.active-model-' + str(os.getpid()))
        try:
            temporary.symlink_to(model)
            os.replace(temporary, link)
        finally:
            temporary.unlink(missing_ok=True)

    def spawn(self, runtime, model, role):
        state = self.base / 'state'
        state.mkdir(parents=True, exist_ok=True)
        program = 'q1-' + role
        arguments = [str(runtime / 'run'), program, '--hardware-profile', str(runtime / 'config/hardware.json'), '--socket', str(state / 'bridge.sock')]
        if role == 'rt':
            arguments += ['--enable-hardware', '--stand-trial', '--policy-v2', '--resume-damping', '--publish-hz', '50', '--rt-priority', '20']
        else:
            arguments += ['--model-dir', str(model), '--port', str(self.port)]
        # RT middleware inherits the normal CPU set. Pin only its control/main
        # thread after the ROS readers have initialized (health admission below).
        if role == 'policy':
            arguments = ['taskset', '-c', ','.join(str(cpu) for cpu in self.affinity[role]), *arguments]
        with (state / (role + '.log')).open('a') as log:
            child = subprocess.Popen(arguments, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        self.children[child.pid] = child
        (state / (role + '.pid')).write_text(str(child.pid) + '\n')
        try:
            expected_binary = str(runtime / 'bin' / program)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                try:
                    current = self.process(child.pid)
                except ProcessLookupError:
                    if child.poll() is not None:
                        raise RuntimeError(program + ' exited during startup with status ' + str(child.returncode))
                    time.sleep(.01)
                    continue
                if expected_binary in current['cmdline']:
                    return current
                time.sleep(.01)
            raise RuntimeError(program + ' did not exec expected binary')
        except Exception:
            if child.poll() is None:
                child.terminate()
            child.wait(timeout=5)
            self.children.pop(child.pid, None)
            raise

    def health(self, report):
        deadline = time.monotonic() + self.health_timeout
        previous = None
        last_error = 'no response'
        while time.monotonic() < deadline:
            try:
                self.assert_process(report, 'rt')
                self.assert_process(report, 'policy')
                state = self.released()
                # A live bridge plus the policy API proves middleware startup is
                # complete; pinning main now does not restrict existing DDS/RX threads.
                rt_pid = report['rt_pid']
                expected_cpus = set(self.affinity['rt'])
                if os.sched_getaffinity(rt_pid) != expected_cpus:
                    os.sched_setaffinity(rt_pid, expected_cpus)
                if state.get('controller') != 'q1_policy' or state.get('model_sha256') != report['model_sha256'] or state.get('profile_sha256') != report['profile_sha256'] or state.get('policy_protocol') != 2 or state.get('bridge', {}).get('controlled_joints') != 22:
                    raise ValueError('health identity/protocol mismatch')
                bridge = state['bridge']
                if state.get('fault') or state.get('guard_fault') or bridge.get('fault') or state.get('blockers') or bridge.get('blockers'):
                    raise ValueError('controller/RT fault or blockers prevent healthy deployment')
                counters = (int(state['input_count']), int(state['model_count']), int(state['bridge']['tx_count']))
                if previous and counters[0] > previous[0] and counters[1] > previous[1] and counters[2] > previous[2]:
                    return state
                previous = counters
                last_error = 'health counters not advancing'
            except (OSError, ValueError, KeyError) as error:
                last_error = str(error)
            time.sleep(.05)
        raise RuntimeError('policy health failed: ' + last_error)

    def report(self, runtime, model, rt, policy, checked):
        return {'schema': 'q1-policy-deployment-v1', 'base': str(self.base), 'port': self.port,
                'runtime_root': str(runtime), 'rt_root': str(runtime), 'policy_root': str(runtime),
                'rt_program': 'q1-rt', 'policy_program': 'q1-policy',
                'runtime_sha256': sha(runtime / 'SHA256SUMS'), 'model_package_sha256': sha(model / 'SHA256SUMS'),
                'model_root': str(model), 'active_model': str(self.base / 'active-model'),
                'model_sha256': sha(model / 'model/policy_native.onnx'), 'contract_sha256': sha(model / 'deployment_contract.json'),
                'rt_sha256': sha(runtime / 'bin/q1-rt'), 'policy_sha256': sha(runtime / 'bin/q1-policy'),
                'profile_path': str(runtime / 'config/hardware.json'), 'profile_sha256': sha(runtime / 'config/hardware.json'),
                'socket': str(self.base / 'state/bridge.sock'), 'rt_pid': rt['pid'], 'policy_pid': policy['pid'],
                'rt_process': rt, 'policy_process': policy, 'preflight': checked,
                'hardware': self.hardware(runtime), 'scheduling': {'rt_priority': 20, 'affinity': self.affinity, 'rt_affinity_scope': 'main/control thread only; middleware retains parent CPUs'}, 'explicit_ui_action_required': True}

    def save(self, report):
        atomic_json(self.base / 'state/deployment.json', report)


def previous_report(report, backend):
    """Accept this workflow's report or the explicit legacy stand deployment."""
    report = dict(report)
    if report.get('schema') == 'q1-policy-deployment-v1':
        if Path(report['base']).resolve() != backend.base or report['port'] != backend.port:
            raise ValueError('previous deployment base/port mismatch')
        return report
    if not {'remote_directory', 'guard_root', 'stand_sha256', 'rt_sha256', 'profile_sha256'} <= report.keys():
        raise ValueError('unsupported previous deployment report')
    for role, root, program, pidfile, expected in [
            ('policy', report['remote_directory'], 'q1-stand', 'stand.pid', report['stand_sha256']),
            ('rt', report['guard_root'], 'q1-rt', 'rt.pid', report['rt_sha256'])]:
        directory = Path(root)
        pid = int((directory / 'state' / pidfile).read_text())
        report.update({role + '_root': root, role + '_program': program, role + '_sha256': expected,
                       role + '_pid': pid, role + '_process': backend.process(pid)})
        backend.assert_process(report, role)
    if sha(Path(report['guard_root']) / 'config/hardware.json') != report['profile_sha256']:
        raise ValueError('previous profile identity changed')
    return report


def deploy(runtime_source, model_source, previous, backend, cold_start=False):
    with backend.lock():
        runtime = install_immutable(runtime_source, backend.base / 'runtimes')
        model = install_immutable(model_source, backend.base / 'models')
        backend.hardware(runtime)
        checked = backend.preflight(runtime, model)
        old = previous_report(previous, backend) if previous else None
        if not old and not cold_start and (backend.base / 'state/deployment.json').exists():
            raise ValueError('--previous-deployment required for an existing task')
        if not old:
            backend.assert_no_writers()
        if old:
            backend.assert_process(old, 'rt')
            backend.assert_process(old, 'policy')
            backend.released()
            backend.stop(old, 'policy')
            backend.stop(old, 'rt')
        backend.switch(model)
        rt = backend.spawn(runtime, model, 'rt')
        policy = None
        report = None
        try:
            policy = backend.spawn(runtime, model, 'policy')
            report = {'policy_pid': policy['pid'], 'policy_process': policy, 'policy_root': str(runtime), 'policy_program': 'q1-policy', 'policy_sha256': sha(runtime / 'bin/q1-policy')}
            report = backend.report(runtime, model, rt, policy, checked)
            report['initial_state'] = backend.health(report)
            backend.save(report)
            return report
        except Exception as error:
            if report:
                backend.stop(report, 'policy', missing_ok=True)
            # Keep the new RT in damping on a failed initial deployment. Its PID
            # is recorded so recovery never has to guess which process owns HAL.
            failure = report or {'schema': 'q1-policy-start-failure-v1', 'rt_pid': rt['pid'], 'rt_process': rt, 'runtime_root': str(runtime), 'model_root': str(model)}
            failure['error'] = str(error)
            backend.save(failure)
            raise UpdateFailure('deployment health failed; RT retained in damping', failure) from error


def update_model(model_source, previous, backend):
    with backend.lock():
        old = previous_report(previous, backend)
        if old.get('schema') != 'q1-policy-deployment-v1':
            raise ValueError('model-only update requires the policy-v2 runtime; deploy it first')
        runtime = Path(old['runtime_root'])
        if sha(runtime / 'SHA256SUMS') != old['runtime_sha256'] or sha(runtime / 'config/hardware.json') != old['profile_sha256']:
            raise ValueError('retained runtime/profile identity changed')
        backend.assert_process(old, 'rt')
        backend.assert_process(old, 'policy')
        backend.hardware(runtime)
        model = install_immutable(model_source, backend.base / 'models')
        checked = backend.preflight(runtime, model)  # Existing policy is still running.
        old_model = Path(old['model_root'])
        verify_package(old_model)
        if sha(old_model / 'SHA256SUMS') != old['model_package_sha256'] or Path(old['active_model']).resolve() != old_model.resolve():
            raise ValueError('active model identity changed')
        backend.released()
        backend.assert_process(old, 'rt')
        backend.stop(old, 'policy')
        new = None
        try:
            backend.switch(model)
            policy = backend.spawn(runtime, model, 'policy')
            new = dict(old, policy_pid=policy['pid'], policy_process=policy)
            new = backend.report(runtime, model, old['rt_process'], policy, checked)
            new['initial_state'] = backend.health(new)
            backend.assert_process(old, 'rt')
            backend.save(new)
            return new
        except Exception as error:
            if new:
                backend.stop(new, 'policy', missing_ok=True)
            backend.switch(old_model)
            restored = dict(old)
            restored['policy_process'] = backend.spawn(runtime, old_model, 'policy')
            restored['policy_pid'] = restored['policy_process']['pid']
            restored['initial_state'] = backend.health(restored)
            restored['rollback'] = {'failed_model': str(model), 'error': str(error)}
            backend.assert_process(old, 'rt')
            backend.save(restored)
            raise UpdateFailure('model update failed; old policy/model restored disabled', restored) from error


def extract_archive(archive, target):
    target = Path(target)
    target.mkdir(parents=True, exist_ok=False)
    seen = set()
    with tarfile.open(archive, 'r:gz') as stream:
        for entry in stream:
            name = str(relative(entry.name))
            if not entry.isfile() or name in seen:
                raise ValueError('archive link/nonregular/duplicate member')
            seen.add(name)
            path = target / name
            path.parent.mkdir(parents=True, exist_ok=True)
            with stream.extractfile(entry) as source, path.open('xb') as output:
                shutil.copyfileobj(source, output)
            path.chmod(0o755 if entry.mode & 0o111 else 0o644)
    verify_package(target)
    return target


def archive_package(package, archive):
    entries = verify_package(package)
    with tarfile.open(archive, 'w:gz', compresslevel=1) as stream:
        for name in ['SHA256SUMS', *sorted(entries)]:
            stream.add(Path(package) / name, arcname=name, recursive=False)
    return sha(archive)


def dispatch(request):
    incoming = Path(request['incoming'])
    try:
        sources = {}
        for role, item in request['archives'].items():
            archive = incoming / (role + '.tar.gz')
            if sha(archive) != item['archive_sha256']:
                raise ValueError('uploaded archive digest mismatch')
            sources[role] = extract_archive(archive, incoming / role)
            if sha(sources[role] / 'SHA256SUMS') != item['package_sha256']:
                raise ValueError('uploaded package identity mismatch')
        backend = Backend(base=Path(request.get('base', '/home/run/q1-policy')), port=request['port'])
        if request['operation'] == 'deploy':
            report = deploy(sources['runtime'], sources['model'], request['previous'], backend, cold_start=request.get('cold_start', False))
        elif request['operation'] == 'update':
            report = update_model(sources['model'], request['previous'], backend)
        else:
            raise ValueError('unknown deployment operation')
        return {'ok': True, 'report': report}
    except UpdateFailure as error:
        return {'ok': False, 'error': str(error), 'report': error.report}
    except Exception as error:
        return {'ok': False, 'error': str(error)}
    finally:
        shutil.rmtree(incoming)


def remote_program(request):
    namespace = "policy_deploy_helper"
    source = Path(__file__).read_text()
    return "scope={'__name__':" + repr(namespace) + "}\nexec(compile(" + repr(source) + ", '<q1-policy-deploy>', 'exec'), scope)\nresult=scope['dispatch'](scope['json'].loads(" + repr(json.dumps(request)) + "))\nprint(scope['json'].dumps(result))\n"


def remote_main(operation):
    """Explicit SSH entrypoint. --dry-run is entirely local and read-only."""
    import shlex
    import uuid
    parser = argparse.ArgumentParser(description='Stage checked immutable policy packages and start disabled.')
    if operation == 'deploy':
        parser.add_argument('--runtime', '--runtime-package', type=Path, required=True)
    parser.add_argument('--model', '--model-package', type=Path, required=True)
    admission = parser.add_mutually_exclusive_group(required=True) if operation == 'deploy' else parser
    admission.add_argument('--previous-deployment', type=Path, required=operation != 'deploy', help='This task\'s exact prior deployment report; only its verified PIDs may be stopped')
    if operation == 'deploy':
        admission.add_argument('--cold-start', action='store_true', help='Start after boot only when no task writer/API/transaction is active; never signal unknown PIDs')
    parser.add_argument('--base', type=Path, help='Reuse this deployment directory; updates default to the recorded base')
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', default='run')
    parser.add_argument('--ssh-key', type=Path)
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--sudo', action='store_true', help='Run only the task-scoped remote transaction with sudo -n, required for prior root-owned writers')
    parser.add_argument('--dry-run', action='store_true', help='Print paths/hashes only, without SSH, preflight, or process changes')
    options = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', options.host) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', options.user):
        parser.error('invalid SSH host/user')
    if not 1 <= options.port <= 65535:
        parser.error('invalid port')
    previous = json.loads(options.previous_deployment.read_text()) if options.previous_deployment and (options.previous_deployment.exists() or not options.dry_run) else None
    base = options.base or (Path(previous['base']) if previous and previous.get('base') else Path('/home/run/q1-policy'))
    if not base.is_absolute() or base == Path('/'):
        parser.error('--base must be an absolute project deployment directory')
    if previous and previous.get('base') and Path(previous['base']) != base:
        parser.error('--base differs from the recorded deployment')
    packages = {'model': options.model.resolve()}
    if operation == 'deploy':
        packages['runtime'] = options.runtime.resolve()
    identities = {}
    for role, path in packages.items():
        verify_package(path)
        identities[role] = sha(path / 'SHA256SUMS')
    plan = {'operation': operation, 'host': options.host, 'target_preflight_required': True,
            'base': str(base), 'model_root': str(base / 'models' / identities['model']),
            'explicit_ui_action_required': True, 'target_privilege': 'root' if options.sudo else 'current-user'}
    if operation == 'deploy':
        plan['runtime_root'] = str(base / 'runtimes' / identities['runtime'])
    if options.dry_run:
        print(json.dumps(plan))
        return
    if not options.ssh_key or not options.ssh_key.is_file():
        parser.error('--ssh-key must name an existing private key')
    if previous and previous.get('host') not in (None, options.host):
        raise ValueError('previous deployment belongs to another host')
    flags = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=3', '-o', 'IdentitiesOnly=yes', '-o', 'StrictHostKeyChecking=yes', '-i', str(options.ssh_key)]
    host = options.user + '@' + options.host
    incoming = str(base / '.incoming' / uuid.uuid4().hex)
    request = {'operation': operation, 'base': str(base), 'incoming': incoming, 'port': options.port, 'previous': previous, 'cold_start': getattr(options, 'cold_start', False), 'archives': {}}
    # No shell interpolation of JSON, paths, or credentials. Remote Python source
    # is stdin to the fixed python3 command; archive paths are generated here.
    subprocess.run(['ssh', *flags, host, 'python3 -'], input='from pathlib import Path\np=Path(' + repr(incoming) + ')\np.mkdir(parents=True, mode=0o700)\n', text=True, check=True)
    with tempfile.TemporaryDirectory(prefix='q1-policy-transfer-') as temporary:
        for role, package in packages.items():
            archive = Path(temporary) / (role + '.tar.gz')
            request['archives'][role] = {'archive_sha256': archive_package(package, archive), 'package_sha256': identities[role]}
            subprocess.run(['scp', '-q', *flags, str(archive), host + ':' + shlex.quote(incoming + '/' + role + '.tar.gz')], check=True)
        script = remote_program(request)
        command = 'sudo -n python3 -' if options.sudo else 'python3 -'
        result = subprocess.run(['ssh', *flags, host, command], input=script, text=True, capture_output=True, check=True)
    response = json.loads(result.stdout.strip().splitlines()[-1])
    report = response.get('report', {'operation': operation, 'error': response.get('error')})
    report['host'] = options.host
    atomic_json(options.report, report)
    print(json.dumps(response))
    if not response['ok']:
        raise SystemExit(1)


def start_main():
    parser = argparse.ArgumentParser(description='Start an already installed policy-v2 runtime in damping and policy disabled.')
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--base', type=Path, default=Path('/home/run/q1-policy'))
    options = parser.parse_args()
    backend = Backend(options.base)
    report = deploy(options.runtime, options.model, None, backend, cold_start=True)
    print(json.dumps(report))


if __name__ == '__main__':
    start_main()
