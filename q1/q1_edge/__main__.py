"""Run the local console. Mock is the default; ROS requires an explicit flag."""
import argparse
import importlib.util
import json
import signal
import sys
import threading
import time

from .config import load_config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('mock','ros2'), default='mock')
    parser.add_argument('--enable-hardware', action='store_true', help='Allow explicit UI enabling of the native MC backend')
    parser.add_argument('--config', help='JSON overrides; unknown keys are rejected')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--allow-host', action='append', default=[], help='Additional allowed HTTP Host for an explicitly bound LAN interface')
    parser.add_argument('--pico', choices=('none','xrobotoolkit'), default='none')
    parser.add_argument('--trace', help='Append command/status JSONL at10Hz for next-day diagnostics')
    parser.add_argument('--diagnose', action='store_true', help='Read-only dependency/config report; no ROS nodes or actuators started')
    args = parser.parse_args(argv)
    if args.backend == 'ros2' and not args.enable_hardware and not args.diagnose:
        parser.error('ROS2 commands require --backend ros2 --enable-hardware; default mock never contacts a robot')
    if not 0 <= args.port <= 65535:
        parser.error('port must be in [0,65535]')
    try:
        config = load_config(args.config)
    except (ValueError,OSError) as error:
        parser.error(str(error))
    if args.diagnose:
        def present(name):
            try:return importlib.util.find_spec(name) is not None
            except (ImportError,ValueError):return False
        print(json.dumps(dict(python=sys.version.split()[0],backend=args.backend,
            dependencies={n:present(n) for n in ('numpy','rclpy','aimdk_msgs','xrobotoolkit_sdk')},
            upper_hardware_limits_confirmed=config['upper']['hardware_limits_confirmed'],
            sdk_commit='7310314eee058323a2c15ab1f4f947df4112573e',
            note='Dependency presence is not robot connection or hardware verification'),ensure_ascii=False,indent=2))
        return 0
    backend = source = server = None
    quit_event = threading.Event()
    trace = None
    try:
        from .backends import MockBackend, Ros2Backend
        from .core import Controller
        from .pico import PicoRetargeter, XRoboSource
        from .server import create_server
        retargeter = PicoRetargeter(limits=config['upper']['limits'])
        backend = MockBackend() if args.backend == 'mock' else Ros2Backend(config)
        source = XRoboSource() if args.pico == 'xrobotoolkit' else None
        controller = Controller(backend, config, retargeter)
        server = create_server(controller, args.host, args.port, args.allow_host)
        if args.trace:
            trace = open(args.trace,'a',buffering=1)

        def control_loop():
            deadline = time.monotonic()
            next_log = deadline
            while not quit_event.is_set():
                try:
                    if source:
                        frame = source.poll()
                        if frame is not None:
                            controller.pico_frame(frame, source='xrobotoolkit')
                    controller.tick()
                    now = time.monotonic()
                    if trace and now >= next_log:
                        state = controller.state()
                        record = {k:state[k] for k in ('backend','enabled','mode','status','command','input')}
                        record.update(monotonic=now,robot_action=state['robot'].get('action'),pico_age_ms=state['pico']['age_ms'])
                        trace.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
                        next_log = now+.1
                except Exception as error:
                    with controller.lock:
                        controller._trip('输入／运行器错误：'+str(error))
                deadline += .01
                now = time.monotonic()
                if deadline < now-.05:
                    deadline = now
                quit_event.wait(max(0.,deadline-now))

        def stop_signal(*_):
            quit_event.set()

        for signum in (signal.SIGINT,signal.SIGTERM):
            signal.signal(signum,stop_signal)
        server.timeout = .2
        worker = threading.Thread(target=control_loop, name='q1-controller', daemon=True)
        worker.start()
        actual_port = server.server_address[1]
        print(f'Q1 Edge console ({args.backend}): http://{args.host}:{actual_port}/',flush=True)
        print('MOCK: no robot commands.' if args.backend=='mock' else 'ROS2 connected read-only until explicit calibrated enable; native MC stays running.',flush=True)
        while not quit_event.is_set():
            server.handle_request()
        worker.join(timeout=2)
        controller.action('stop')
        return 0
    except (ImportError,RuntimeError,ValueError,OSError) as error:
        print('Q1 console startup failed: '+str(error),file=sys.stderr)
        return 2
    finally:
        quit_event.set()
        if source:
            source.close()
        if backend:
            backend.close()
        if server:
            server.server_close()
        if trace:
            trace.close()


if __name__ == '__main__':
    raise SystemExit(main())
