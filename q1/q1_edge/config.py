"""Small explicit configuration shared by the controller and SDK bridge."""
import copy
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def load_config(path=None):
    value = json.loads((ROOT / 'config/default.json').read_text())
    if path is not None:
        supplied = json.loads(Path(path).read_text())
        def merge(dst, src):
            if not isinstance(src, dict):
                raise ValueError('Configuration must be a JSON object')
            for key, item in src.items():
                if key not in dst:
                    raise ValueError('Unknown configuration key: ' + key)
                if isinstance(dst[key], dict):
                    merge(dst[key], item)
                else:
                    dst[key] = item
        merge(value, supplied)
    for key in ('input_timeout_s', 'pico_timeout_s', 'neutral_threshold', 'deadzone'):
        if type(value[key]) not in (int, float) or not math.isfinite(value[key]) or value[key] <= 0:
            raise ValueError('Invalid ' + key)
    if value['neutral_threshold'] > value['deadzone'] or value['deadzone'] >= 1:
        raise ValueError('Require neutral_threshold <= deadzone < 1')
    for name, limit in value['upper']['limits'].items():
        if len(limit) != 2 or not all(type(x) in (int, float) and math.isfinite(x) for x in limit) or limit[0] >= limit[1]:
            raise ValueError('Invalid joint limits: ' + name)
    for key, maximum in value['velocity'].items():
        if type(maximum) not in (int, float) or not math.isfinite(maximum) or maximum < 0:
            raise ValueError('Invalid velocity setting: ' + key)
    if any(value['velocity'][key] < value['velocity']['min_nonzero'] for key in ('max_forward','max_lateral','max_angular')):
        raise ValueError('Velocity maxima must cover the configured nonzero minimum')
    if type(value['upper']['hardware_limits_confirmed']) is not bool:
        raise ValueError('hardware_limits_confirmed must be boolean')
    if type(value['gamepad']['rb_toggle_enabled']) is not bool:
        raise ValueError('rb_toggle_enabled must be boolean')
    duration=value['gamepad']['rb_hold_s']
    if type(duration) not in (int,float) or not math.isfinite(duration) or not 1 <= duration <= 3:
        raise ValueError('rb_hold_s must be between 1 and 3 seconds')
    for index in value['buttons'].values():
        if type(index) is not int or not 0 <= index < 17:
            raise ValueError('Button indices must be integers in [0,16]')
    if len(set(value['buttons'].values())) != len(value['buttons']):
        raise ValueError('Each control requires a distinct button')
    return copy.deepcopy(value)
