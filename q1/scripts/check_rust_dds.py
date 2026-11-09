"""Exercise Rust HTTP and actual Humble DDS against a separate fake MC.

Run only inside a --network none container with ROS_DOMAIN_ID=117 and
ROS_LOCALHOST_ONLY=1. No actual robot is involved. Reports contain measured
receiver-side timing, not real-hardware stopping guarantees.
"""
import argparse
import http.client
import hashlib
import json
import os
import platform
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time


TEST_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def require_isolation():
    interfaces = sorted(path.name for path in Path('/sys/class/net').iterdir())
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '117' or interfaces != ['lo']:
        raise RuntimeError('Refusing fake MC: require --network none, ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=117; interfaces=' + repr(interfaces))


def wait(predicate, description, timeout=8., heartbeat=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if heartbeat:
            heartbeat()
        if predicate():
            return
        time.sleep(.025)
    raise AssertionError(description)


def nonzero(packet):
    return any(abs(value) > 1e-9 for value in (packet.forward_velocity, packet.lateral_velocity, packet.angular_velocity))


def timing(samples):
    intervals = sorted((right - left) * 1000 for left, right in zip(samples, samples[1:]))
    return dict(samples=len(samples), p99_ms=intervals[min(len(intervals) - 1, int(len(intervals) * .99))] if intervals else None,
                maximum_ms=max(intervals) if intervals else None)


def make_peer(standard):
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    import aimdk_msgs.msg as msg
    import aimdk_msgs.srv as srv
    from q1_edge.backends import UPPER_HOME

    class Peer(Node):
        def __init__(self):
            super().__init__('fake_q1_rust_acceptance')
            self.group = ReentrantCallbackGroup()
            self.joint_pub = self.create_publisher(JointState if standard else msg.JointStateArray,
                '/upper_body_joint_states' if standard else '/aima/hal/joint/state',
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(msg.McLocomotionVelocity, '/aima/mc/locomotion/velocity', self.on_velocity, 10,
                                     callback_group=self.group)
            self.service_handles = []
            for name in ('GetMcAction', 'GetCurrentInputSource', 'SetMcInputSource', 'SetMcAction'):
                self.service_handles.append(self.create_service(getattr(srv, name), '/aimdk_5Fmsgs/srv/' + name,
                    lambda request, response, n=name: self.rpc(n, request, response), callback_group=self.group))
            self.create_timer(.01, self.publish_joints, callback_group=self.group)
            self.reset()

        def reset(self):
            self.feedback = True
            self.bad_feedback = False
            self.action = 'BIPED_WALK_RUN'
            self.owner = ''
            self.source = ''
            self.registered = False
            self.takeover = False
            self.allow_owner = True
            self.owner_delay = 0.
            self.query_delay = 0.
            self.first_motion = None
            self.owner_ack_at = None
            self.velocity = []
            self.calls = []

        def rpc(self, name, request, response):
            self.calls.append((time.monotonic(), name, request))
            if name == 'GetMcAction':
                response.header.code = 0
                response.info.action_desc = self.action
                response.info.status.value = msg.McActionStatus.RUNNING
                response.info.switch_ready = True
                return response
            response.response.header.code = 0
            response.response.state.value = msg.CommonState.UNKNOWN
            response.response.state.reason = 0
            if name == 'GetCurrentInputSource':
                if self.query_delay:
                    time.sleep(self.query_delay)
                if self.allow_owner and self.first_motion and time.monotonic() - self.first_motion >= self.owner_delay and not self.takeover:
                    self.owner = self.source
                if self.owner == self.source and self.source and self.owner_ack_at is None:
                    self.owner_ack_at = time.monotonic()
                response.input_source.name = self.owner
            elif name == 'SetMcInputSource':
                if request.action.value == 1001:
                    assert request.input_source.priority == 30
                    assert 250 <= request.input_source.timeout <= 1000
                    self.source = request.input_source.name
                    self.registered = True
                    self.first_motion = None
                    self.owner_ack_at = None
                elif request.action.value == 1003:
                    assert request.input_source.name == self.source, 'must only delete own registered source'
                    self.registered = False
                    # Real firmware can retain the expired/deleted name.
                else:
                    raise AssertionError('unexpected source action')
            else:
                raise AssertionError('hand-controller must not change native MC action')
            return response

        def publish_joints(self):
            if not self.feedback:
                return
            packet = JointState() if standard else msg.JointStateArray()
            packet.header.stamp = self.get_clock().now().to_msg()
            for index, (name, value) in enumerate(UPPER_HOME.items()):
                if self.bad_feedback and index == 0:
                    continue
                if standard:
                    packet.name.append(name)
                    packet.position.append(value)
                else:
                    joint = msg.JointState()
                    joint.name, joint.position = name, value
                    packet.joints.append(joint)
            self.joint_pub.publish(packet)

        def on_velocity(self, packet):
            assert packet.pitch_velocity == 0 and packet.level == 0
            if self.velocity:
                previous = self.velocity[-1][1]
                delta = (packet.header.sequence - previous.header.sequence) % (2 ** 32)
                assert 0 < delta < 2 ** 31, 'velocity sequence must advance'
                previous_ns = previous.header.stamp.sec * 10 ** 9 + previous.header.stamp.nanosec
                stamp_ns = packet.header.stamp.sec * 10 ** 9 + packet.header.stamp.nanosec
                assert stamp_ns > previous_ns, 'velocity timestamps must advance'
            self.velocity.append((time.monotonic(), packet))
            if nonzero(packet) and self.registered and self.first_motion is None:
                self.first_motion = time.monotonic()

    return Peer()


class Session:
    def __init__(self, binary, directory, port=None, service_timeout=None):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', port or 0))
            self.port = listener.getsockname()[1]
        self.sequence = 0
        self.token = None
        self.last_input_at = None
        self.last_moving = None
        self.log_path = Path(directory) / ('rust-' + str(self.port) + '.log')
        self.log = self.log_path.open('w')
        command = [sys.executable, '-m', 'q1_edge'] if binary is None else [str(binary)]
        command += ['--backend', 'ros2', '--enable-hardware', '--host', '127.0.0.1', '--port', str(self.port)]
        if service_timeout is not None:
            config_path = Path(directory) / ('config-' + str(self.port) + '.json')
            config_path.write_text(json.dumps({'ros2': {'service_timeout_s': service_timeout}}))
            command += ['--config', str(config_path)]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=self.log)
        wait(self.started, 'HTTP service failed to start')
        html = self.request('GET', '/', raw=True)
        self.token = re.search(r'name="api-token"\s+content="([^"]+)"', html).group(1)
        self.cpu_before = self.cpu_ticks()

    def request(self, method, path, data=None, raw=False, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2.)
        supplied = {'Content-Type': 'application/json'}
        if self.token:
            supplied['X-Q1-Token'] = self.token
        supplied.update(headers or {})
        try:
            connection.request(method, path, None if data is None else json.dumps(data), headers=supplied)
            response = connection.getresponse()
            body = response.read().decode()
            if raw:
                assert response.status == 200, (response.status, body)
                return body
            parsed = json.loads(body)
            if response.status >= 400:
                raise AssertionError((response.status, parsed))
            return parsed
        finally:
            connection.close()

    def started(self):
        assert self.process.poll() is None, self.log_path.read_text()
        try:
            self.request('GET', '/api/state')
            return True
        except (ConnectionError, OSError):
            return False

    def state(self):
        return self.request('GET', '/api/state')

    def pad(self, axes=(0., 0., 0., 0.), buttons=(), connected=True):
        self.sequence += 1
        packet = dict(client_id='dds-pad', source='browser_gamepad', sequence=self.sequence,
                      axes=list(axes), buttons=[i in buttons for i in range(17)], connected=connected)
        result = self.request('POST', '/api/input', packet, headers={'X-Q1-Compact': '1'})
        assert result == {'ok': True, 'ack': {'sequence': self.sequence}}, result
        self.last_input_at = time.monotonic()
        if any(axes):
            self.last_moving = self.last_input_at
        return result

    def action(self, name):
        return self.request('POST', '/api/action', dict(client_id='dds-pad', action=name))

    def prepare(self, peer):
        wait(lambda: self.state()['robot']['connected'], 'fresh complete feedback before enable')
        assert not peer.registered and not peer.velocity, 'startup must be read-only'
        self.pad()
        self.action('calibrate')
        self.action('enable')
        wait(lambda: peer.registered, 'explicit source registration', heartbeat=self.pad)
        for _ in range(4):
            self.pad()
            time.sleep(.03)
        assert not self.state()['robot']['source_active'], 'zero velocity must not acquire owner'
        self.action('walk')

    def acquire(self, peer):
        wait(lambda: self.state()['robot']['source_active'] and any(packet.forward_velocity > .25 for _, packet in peer.velocity),
             'nonzero intent acquires source then normal velocity', heartbeat=lambda: self.pad((0., -1., 0., 0.)))
        first = next(packet for _, packet in peer.velocity if nonzero(packet))
        assert 0 < first.forward_velocity <= .2, 'bootstrap speed cap'
        assert peer.owner_ack_at is not None
        pre_ack = [packet for at, packet in peer.velocity if at < peer.owner_ack_at and nonzero(packet)]
        assert pre_ack and all(max(abs(packet.forward_velocity), abs(packet.lateral_velocity), abs(packet.angular_velocity)) <= .2 for packet in pre_ack), 'all pre-owner-ACK velocities must be capped'
        assert first.source == peer.source and re.fullmatch(r'edge_q1_[a-zA-Z0-9_]+', first.source)

    def hold(self):
        deadline = time.monotonic() + 1.12
        while time.monotonic() < deadline:
            self.pad(buttons=(5,))
            time.sleep(.035)

    def cpu_ticks(self):
        try:
            fields = Path('/proc/' + str(self.process.pid) + '/stat').read_text().split()
            return int(fields[13]) + int(fields[14])
        except FileNotFoundError:
            return None

    def resources(self):
        status = Path('/proc/' + str(self.process.pid) + '/status').read_text()
        rss = re.search(r'^VmRSS:\s+(\d+) kB', status, re.M)
        return dict(rss_kib=int(rss.group(1)) if rss else None,
                    cpu_seconds=(self.cpu_ticks() - self.cpu_before) / os.sysconf('SC_CLK_TCK'))

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
                raise AssertionError('SIGTERM shutdown exceeded 3 seconds')
        self.log.close()
        assert self.process.returncode == 0, self.log_path.read_text()


def check_case(binary, peer, directory, fault=None):
    peer.reset()
    session = Session(binary, directory, service_timeout=.15 if fault == 'service_deadline' else None)
    report = dict(name=fault or 'normal_controls')
    stalled_http = None
    try:
        if fault == 'cached_foreign_owner':
            peer.owner = 'edge_q1_old_deleted'
        if fault == 'late_owner':
            peer.owner_delay = .55
        if fault == 'no_owner':
            peer.allow_owner = False
        session.prepare(peer)
        if fault in ('late_owner', 'no_owner'):
            started = time.monotonic()
            wait(lambda: session.state()['mode'] == 'fault', 'acquisition deadline must fault',
                 heartbeat=lambda: session.pad((0., -1., 0., 0.)))
            report['acquisition_fault_ms'] = (time.monotonic() - started) * 1000
            wait(lambda: not peer.registered, 'failed acquisition releases own source')
            assert any(nonzero(packet) for _, packet in peer.velocity)
            assert not nonzero(peer.velocity[-1][1])
            assert all(abs(packet.forward_velocity) <= .2 for _, packet in peer.velocity)
            for _ in range(14):
                session.pad((0., -1., 0., 0.))
                time.sleep(.04)
            assert not session.state()['enabled'], 'late owner/input must not auto-rearm'
            assert not peer.registered
            motion_at = next(at for at, packet in peer.velocity if nonzero(packet))
            zero_at = next(at for at, packet in peer.velocity if at > motion_at and not nonzero(packet))
            report['first_nonzero_to_zero_ms'] = (zero_at - motion_at) * 1000
            assert report['first_nonzero_to_zero_ms'] <= 400, report
            return report
        session.acquire(peer)
        if fault:
            before = time.monotonic()
            if fault == 'feedback_stale':
                peer.feedback = False
            elif fault == 'incomplete_feedback':
                peer.bad_feedback = True
            elif fault == 'takeover':
                peer.takeover = True
                peer.owner = 'rc'
            elif fault == 'owner_service_hang':
                peer.query_delay = .65
            elif fault == 'service_deadline':
                peer.query_delay = .6
            elif fault == 'slow_http':
                stalled_http = socket.create_connection(('127.0.0.1', session.port), timeout=2.)
                request = ('POST /api/input HTTP/1.1\r\nHost: 127.0.0.1:' + str(session.port) +
                           '\r\nContent-Type: application/json\r\nX-Q1-Token: ' + session.token +
                           '\r\nContent-Length: 100\r\n\r\n{')
                stalled_http.sendall(request.encode())
            elif fault == 'input_stale':
                pass
            elif fault == 'cached_foreign_owner':
                session.pad(connected=False)
            else:
                raise AssertionError(fault)
            heartbeat = None if fault in ('input_stale', 'slow_http') else (session.pad if fault == 'cached_foreign_owner' else lambda: session.pad((0., -1., 0., 0.)))
            wait(lambda: not session.state()['enabled'], 'fault stops controller', heartbeat=heartbeat)
            wait(lambda: bool(peer.velocity) and not nonzero(peer.velocity[-1][1]), 'fault reaches receiver as zero')
            first_zero = next(at for at, packet in peer.velocity if at >= before and not nonzero(packet))
            report['fault_to_zero_ms'] = (first_zero - before) * 1000
            if fault in ('input_stale', 'slow_http'):
                report['last_input_to_zero_ms'] = (first_zero - session.last_input_at) * 1000
                assert report['last_input_to_zero_ms'] <= 400, report
            assert report['fault_to_zero_ms'] <= 500, report
            if fault == 'service_deadline':
                # Until the first delayed response, retain one timed-out Rust
                # future so r2r's native request bookkeeping cannot accumulate.
                # rclpy removes its pending request on timeout and may safely retry.
                until = before + .45
                while time.monotonic() < until:
                    session.pad((0., -1., 0., 0.))
                    time.sleep(.025)
                queries = sum(at >= before and name == 'GetCurrentInputSource' for at, name, _ in peer.calls)
                report['owner_queries_before_any_delayed_response'] = queries
                assert queries >= 1, 'fault injection must exercise a real query'
                if binary is not None:
                    assert queries == 1, 'timed-out native request must remain bounded until its late reply is drained'
            for _ in range(10):
                session.pad((0., -1., 0., 0.))
                time.sleep(.03)
            assert not session.state()['enabled'], 'recovery frames may not auto-rearm'
            return report

        received = len(peer.velocity)
        start = time.monotonic()
        while time.monotonic() - start < 1.5:
            session.pad((0., -1., 0., 0.))
            time.sleep(.035)
        samples = [at for at, _ in peer.velocity[received:]]
        report['heartbeat'] = timing(samples)
        assert len(samples) >= 45, report
        assert report['heartbeat']['maximum_ms'] < 150, report
        for axes, field, sign in (((0., 0., -.7, 0.), 'angular_velocity', 1),
                                  ((0., 0., .7, 0.), 'angular_velocity', -1),
                                  ((-1., 0., 0., 0.), 'lateral_velocity', 1)):
            before = len(peer.velocity)
            wait(lambda: any(getattr(packet, field) * sign > .1 for _, packet in peer.velocity[before:]),
                 'joystick direction ' + field, heartbeat=lambda a=axes: session.pad(a))
        session.pad()
        wait(lambda: not nonzero(peer.velocity[-1][1]), 'recenter zeros')
        session.pad()
        session.hold()
        assert not session.state()['enabled'], 'RB long-hold disables'
        session.pad()
        wait(lambda: not peer.registered, 'RB release own source', heartbeat=session.pad)
        session.hold()
        assert session.state()['enabled'], 'independent second hold enables'
        session.pad()
        session.action('walk')
        wait(lambda: session.state()['robot']['source_active'], 'new epoch acquired',
             heartbeat=lambda: session.pad((0., -.8, 0., 0.)))
        session.pad(connected=False)
        wait(lambda: not peer.registered, 'disconnect releases source')
        assert not nonzero(peer.velocity[-1][1])
        assert not session.state()['enabled'] and not session.state()['calibrated']
        report['resources'] = session.resources()
        report['checks'] = ['readonly_startup', 'unknown_state_source_rpc', 'bootstrap_cap',
                            'latched_rb_release_motion', 'right_yaw_both_signs', 'lateral',
                            'recenter_zero', 'rb_toggle', 'disconnect_zero_release']
        return report
    except Exception as error:
        try:
            state = session.state()
        except Exception:
            state = None
        raise AssertionError(dict(case=fault or 'normal_controls', error=str(error),
                                  state=state, log=session.log_path.read_text()[-6000:])) from error
    finally:
        if stalled_http is not None:
            stalled_http.close()
        session.close()


def check_restart(binary, peer, directory):
    peer.reset()
    first = Session(binary, directory)
    try:
        first.prepare(peer)
        first.acquire(peer)
        original = peer.source
        port = first.port
        first.action('stop')
        wait(lambda: not peer.registered, 'stop releases source before restart')
        assert not nonzero(peer.velocity[-1][1])
    finally:
        first.close()
    # Preserve the stale native owner while clearing observations for the new run.
    peer.velocity = []
    peer.calls = []
    second = Session(binary, directory, port=port)
    try:
        wait(lambda: second.state()['robot']['connected'], 'restart feedback')
        assert not second.state()['enabled'] and not second.state()['calibrated']
        assert not peer.registered and not peer.velocity, 'restart must remain read-only'
        second.prepare(peer)
        assert peer.source != original, 'restart must register a new epoch'
        second.acquire(peer)
        second.action('stop')
        wait(lambda: not peer.registered, 'restarted process stops and releases')
        return dict(name='graceful_restart_cached_owner', passed=True)
    finally:
        second.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path, nargs='?')
    parser.add_argument('--python-reference', action='store_true')
    parser.add_argument('--standard-joints', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    require_isolation()
    if args.python_reference:
        assert args.binary is None, '--python-reference runs the existing Python service without a binary argument'
        binary = None
    else:
        assert args.binary is not None, 'Rust executable is required'
        binary = args.binary.resolve()
        assert binary.is_file(), binary
    binary_digest = None if binary is None else hashlib.sha256(binary.read_bytes()).hexdigest()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    peer = make_peer(args.standard_joints)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(peer)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        with tempfile.TemporaryDirectory(prefix='q1-rust-dds-') as directory:
            reports = []
            for case in (None, 'cached_foreign_owner', 'no_owner', 'late_owner', 'feedback_stale',
                         'incomplete_feedback', 'input_stale', 'slow_http', 'takeover', 'owner_service_hang', 'service_deadline'):
                reports.append(check_case(binary, peer, directory, case))
            peer.query_delay = 0.
            time.sleep(.7)  # Finish the injected delayed response before reusing the peer.
            reports.append(check_restart(binary, peer, directory))
            if binary is not None:
                assert hashlib.sha256(binary.read_bytes()).hexdigest() == binary_digest, 'executable changed during acceptance; rerun the stable release artifact'
            report = dict(passed=True, implementation='python' if args.python_reference else 'rust', joint_format='standard' if args.standard_joints else 'sdk',
                          transport='real Humble DDS and HTTP; independent Python fake MC', cases=reports,
                          host_architecture=platform.machine(),
                          test_sha256=TEST_SHA256,
                          binary_sha256=binary_digest,
                          ros_distro=os.environ.get('ROS_DISTRO'), rmw=os.environ.get('RMW_IMPLEMENTATION'),
                          limitations=['timing from isolated host only', 'physical stop and process-kill source timeout require real firmware'])
            if args.report:
                args.report.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report))
    finally:
        executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.)
        peer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
