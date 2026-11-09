"""Replay identical timed hand-controller events through Python and Rust.

This test uses only MockBackend and never opens ROS or a robot connection.
Run from q1: python3 -m scripts.check_rust_parity /path/to/q1-edge
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

from q1_edge.backends import MockBackend
from q1_edge.config import load_config
from q1_edge.core import Controller


class Trace:
    def __init__(self, source='browser_gamepad'):
        self.events = []
        self.time = 100.0
        self.sequence = 0
        self.source = source

    def pad(self, axes=(0., 0., 0., 0.), buttons=(), client='a', connected=True, **changes):
        self.sequence += 1
        packet = dict(client_id=client, source=self.source, sequence=self.sequence,
                      axes=list(axes), buttons=[i in buttons for i in range(17)], connected=connected)
        packet.update(changes)
        self.events.append(dict(time=self.time, input=packet))
        return self

    def action(self, name, client='a'):
        self.events.append(dict(time=self.time, action=name, client_id=client))
        return self

    def tick(self, delta=0.):
        self.time += delta
        self.events.append(dict(time=self.time, tick=True))
        return self

    def advance(self, delta):
        self.time += delta
        return self

    def ready(self):
        return self.pad().action('calibrate')

    def walk(self):
        return self.ready().action('enable').action('walk')

    def hold(self, duration=1.1):
        start = self.time
        self.pad(buttons=(5,))
        for index in range(1, round(duration / .1) + 1):
            self.time = start + index * .1
            self.pad(buttons=(5,)).tick()
        return self


def cases():
    result = []
    def add(name, trace, config=None):
        result.append((name, trace.events, config or {}))

    t = Trace().ready().hold(2.).pad().action('walk')
    for axes in ((0., -1., 0., 0.), (1., 0., 0., 0.), (0., 0., 1., 0.),
                 (0., 0., -1., 0.), (0., 0., 0., 0.)):
        t.pad(axes=axes).tick()
    t.hold().pad().hold().pad().action('walk').pad(axes=(0., -.7, 0., 0.)).tick()
    add('physical_rb_latch_toggle_and_body_yaw', t)

    t = Trace().ready().hold(.9).pad().hold()
    t.pad().hold().pad(buttons=(5, 9)).action('enable').tick()
    t.pad().hold().tick()
    add('short_hold_and_release_before_reenable', t)

    t = Trace().walk().pad(axes=(0., -.8, 0., 0.)).tick()
    t.action('stop').pad(buttons=(8, 9)).tick().pad().action('calibrate').action('enable')
    t.pad(buttons=(1, 9)).tick()
    add('stop_fences_late_enable_and_same_frame_stop_wins', t)

    for delta in (.25, .251):
        t = Trace().walk().pad(axes=(0., -.5, 0., 0.)).tick(delta)
        t.pad(axes=(0., -.5, 0., 0.), buttons=(5,)).tick()
        add('input_tick_boundary_' + str(delta), t)
        t = Trace().walk().pad(axes=(0., -.5, 0., 0.)).advance(delta)
        t.pad(axes=(0., -.5, 0., 0.), buttons=(8, 9)).tick()
        add('late_input_before_tick_' + str(delta), t)

    t = Trace().walk()
    for index in range(100):
        t.advance((.05, .12, .18, .2)[index % 4])
        t.pad(axes=(0., -.4, -.5 if index % 2 else .5, 0.)).tick()
    t.advance(.251).pad(buttons=(8, 9)).tick().pad().hold()
    add('bounded_jitter_then_stale_recovery', t)

    t = Trace().walk().pad(axes=(0., -1., 0., 0.)).tick()
    t.pad(client='b').tick().advance(.3).pad(client='b').tick().action('enable', 'b')
    t.action('clear_fault', 'b').pad(client='b').action('calibrate', 'b').action('enable', 'b')
    add('ownership_change_invalidates_calibration', t)

    t = Trace().walk().pad(axes=(0., -1., 0., 0.)).tick()
    previous = t.sequence
    t.advance(.15).pad(sequence=previous).advance(.15).tick()
    t.pad().action('clear_fault').action('calibrate').action('enable')
    add('replay_cannot_refresh_deadline', t)

    t = Trace().pad(buttons=(9,)).action('enable').pad().action('calibrate').action('enable')
    t.pad(connected=False).tick().pad(buttons=(5, 9)).tick()
    add('held_start_disconnect_reconnect', t)

    t = Trace().walk()
    for axes in ((0., -.179999, 0., 0.), (0., -.18, 0., 0.), (0., -.180001, 0., 0.),
                 (0., -.5, 0., 0.), (0., -1., 0., 0.), (-1., 0., 0., 0.),
                 (0., 0., -.180001, 0.), (0., 0., .18, 0.)):
        t.pad(axes=axes).tick()
    add('velocity_deadzone_minimum_limits', t)

    t = Trace().ready().pad(axes=(.1, -.1, .1, 0.)).action('calibrate').action('enable').action('walk')
    t.pad(axes=(.1, -.1, .1, 0.)).tick().pad(axes=(.1, -.7, -.4, 0.)).tick()
    add('calibration_offsets', t)

    for source, config in (('simulator', {}), ('browser_gamepad', {'gamepad': {'rb_toggle_enabled': False}})):
        t = Trace(source).walk().pad(axes=(0., -1., 0., 0.)).tick()
        t.pad(axes=(0., -1., 0., 0.), buttons=(5,)).tick().pad().tick()
        add('held_deadman_' + source, t, config)

    t = Trace().walk()
    for invalid in ({'axes': [True, 0, 0, 0]}, {'axes': [0, 0, 0]}, {'axes': [2, 0, 0, 0]},
                    {'buttons': [1]}, {'connected': 1}, {'sequence': True}, {'sequence': -1},
                    {'client_id': ''}, {'source': 'invalid'}):
        t.pad()
        t.events[-1]['input'].update(invalid)
        t.tick()
    t.tick(.251)
    add('invalid_packets_do_not_refresh_input', t)
    return result


def projection(result):
    state = result['state']
    return dict(ok=result['ok'], enabled=state['enabled'], calibrated=state['calibrated'],
                mode=state['mode'], source=state['input']['source'],
                client_id=state['input']['client_id'], deadman=state['input']['deadman'],
                motion_latched=state['input']['motion_latched'], velocity=state['command']['velocity'],
                actual_velocity=state['robot'].get('velocity'))


def same(left, right):
    if isinstance(left, dict) and isinstance(right, dict):
        return left.keys() == right.keys() and all(same(left[key], right[key]) for key in left)
    if isinstance(left, (int, float)) and not isinstance(left, bool) and isinstance(right, (int, float)):
        return math.isclose(left, right, rel_tol=1e-10, abs_tol=1e-10)
    return type(left) is type(right) and left == right


def python_replay(events, config_path):
    clock = [0.]
    core = Controller(MockBackend(), load_config(config_path), clock=lambda: clock[0])
    output = []
    for event in events:
        clock[0] = event['time']
        ok = True
        try:
            if 'input' in event:
                core.input(copy.deepcopy(event['input']))
            elif 'action' in event:
                core.action(event['action'], client_id=event.get('client_id'))
            else:
                core.tick()
        except (ValueError, RuntimeError, TypeError) as error:
            ok = False
        output.append(dict(ok=ok, state=core.state()))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    binary = args.binary.resolve()
    assert binary.is_file(), 'Rust replay executable is missing: ' + str(binary)
    reports = []
    for name, events, config in cases():
        with tempfile.TemporaryDirectory(prefix='q1-parity-') as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(config))
            expected = python_replay(events, str(path))
            process = subprocess.run([str(binary), '--replay', '--config', str(path)],
                                     input=''.join(json.dumps(row) + '\n' for row in events),
                                     text=True, capture_output=True, timeout=30, check=True)
            actual = [json.loads(line) for line in process.stdout.splitlines() if line.strip()]
            assert len(actual) == len(expected), (name, 'response count', len(expected), len(actual), process.stderr)
            for index, (left, right) in enumerate(zip(expected, actual)):
                assert same(projection(left), projection(right)), json.dumps(dict(
                    case=name, index=index, event=events[index], python=projection(left), rust=projection(right)), ensure_ascii=False)
            reports.append(dict(name=name, events=len(events), passed=True))
    report = dict(passed=True, backend='mock', cases=reports, event_count=sum(item['events'] for item in reports),
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                  reference_sha256={name: hashlib.sha256((Path(__file__).resolve().parents[1] / name).read_bytes()).hexdigest()
                                    for name in ('q1_edge/core.py', 'q1_edge/backends.py', 'q1_edge/config.py', 'config/default.json')})
    if args.report:
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
