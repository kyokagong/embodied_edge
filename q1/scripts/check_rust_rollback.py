"""Prove Rust -> Python -> Rust rollback using real HTTP/DDS in isolation.

The same TCP port and fake MC are retained while each controller is stopped,
its registered source release is confirmed, and the next runtime starts disabled.
Requires the same --network none guard as check_rust_dds.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time

from scripts.check_rust_dds import Session, make_peer, nonzero, require_isolation, wait


def stopped(session, peer):
    session.action('stop')
    wait(lambda: not peer.registered, 'old runtime must release its own source before rollback')
    assert peer.velocity and not nonzero(peer.velocity[-1][1]), 'last old-runtime command must be zero'
    assert not session.state()['enabled'] and not session.state()['calibrated']


def readonly(session, peer):
    wait(lambda: session.state()['robot']['connected'], 'new runtime receives complete feedback')
    state = session.state()
    assert state['backend'] == 'ros2'
    assert not state['enabled'] and not state['calibrated'], 'new runtime must require new calibration and enable'
    time.sleep(.15)
    assert not peer.registered and not peer.velocity, 'new runtime must not register or send velocity at startup'
    return dict(enabled=state['enabled'], calibrated=state['calibrated'], mode=state['mode'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    require_isolation()
    binary = args.binary.resolve()
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    peer = make_peer(standard=True)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(peer)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    stages = []
    try:
        with tempfile.TemporaryDirectory(prefix='q1-rollback-') as directory:
            rust = Session(binary, directory)
            try:
                port = rust.port
                stages.append(dict(runtime='rust', startup=readonly(rust, peer)))
                rust.prepare(peer)
                rust.acquire(peer)
                rust_source = peer.source
                stopped(rust, peer)
            finally:
                rust.close()
            # Preserve the stale attribution the actual firmware can retain.
            peer.velocity = []
            peer.calls = []
            python = Session(None, directory, port=port)
            try:
                stages.append(dict(runtime='python', startup=readonly(python, peer)))
                python.prepare(peer)
                assert peer.source != rust_source, 'Python registration must use its own fresh epoch'
                python.acquire(peer)
                python_source = peer.source
                stopped(python, peer)
            finally:
                python.close()
            peer.velocity = []
            peer.calls = []
            rust_again = Session(binary, directory, port=port)
            try:
                stages.append(dict(runtime='rust', startup=readonly(rust_again, peer)))
                assert peer.owner == python_source, 'test must retain the previous owner attribution'
            finally:
                rust_again.close()
            assert hashlib.sha256(binary.read_bytes()).hexdigest() == digest, 'binary changed during rollback acceptance'
            report = dict(passed=True, runtimes=['rust', 'python', 'rust'], port_reused=True,
                          stages=stages, rust_source=rust_source, python_source=python_source,
                          binary_sha256=digest, test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          transport='real Humble DDS and HTTP; isolated fake MC',
                          limitations=['physical source release and robot stop still require real firmware validation'])
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
