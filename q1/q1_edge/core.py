"""Input arbitration and explicit native-MC control; no ROS dependency."""
from collections import deque
import copy
import math
import threading
import time


def _numbers(values, count, label):
    if not isinstance(values, (list, tuple)) or len(values) != count or any(
        type(x) not in (int, float) or not math.isfinite(x) for x in values
    ):
        raise ValueError(label + ' must contain ' + str(count) + ' finite numbers')
    return [float(x) for x in values]


def validate_pico(frame):
    if not isinstance(frame, dict) or frame.get('valid') is not True:
        raise ValueError('Pico tracking is not valid')
    stamp = frame.get('timestamp_ns')
    if type(stamp) is not int or not 0 < stamp < 2**63:
        raise ValueError('Pico timestamp must be a positive int64')
    result = {'timestamp_ns': stamp, 'valid': True}
    for key in ('head', 'left', 'right'):
        pose = _numbers(frame.get(key), 7, key)
        if abs(sum(x*x for x in pose[3:]) - 1) > .002:
            raise ValueError(key + ' quaternion must have unit length')
        result[key] = pose
    for key in ('left_grip', 'right_grip'):
        grip = frame.get(key, 0.)
        if type(grip) not in (int, float) or not math.isfinite(grip) or not 0 <= grip <= 1:
            raise ValueError('Grip must be in [0,1]')
        result[key] = float(grip)
    return result


class Controller:
    def __init__(self, backend, config, retargeter=None, clock=time.monotonic):
        self.backend, self.config, self.retargeter, self.clock = backend, config, retargeter, clock
        self.lock = threading.RLock()
        self.enabled = self.calibrated = False
        self.mode, self.status = 'idle', '等待输入，先校准，再使能'
        self._owner = None
        self._source = None
        self._seq = -1
        self._received = None
        self._axes = [0.]*4
        self._offsets = [0.]*4
        self._buttons = [False]*17
        self._buttons_seen = False
        self._pico = None
        self._pico_time = self._pico_processed_time = None
        self._pico_source = None
        self._pico_stamp = self._processed_stamp = 0
        self._pico_gripping = self._pico_calibrated = False
        self._source_was_active = False
        self._velocity = dict(forward=0., lateral=0., angular=0.)
        self._upper = {}
        self.events = deque(maxlen=60)
        self._log('启动：默认禁用，尚未校准')

    @property
    def kind(self):
        return getattr(self.backend, 'kind', 'mock')

    def _log(self, event):
        self.events.append({'time': time.strftime('%H:%M:%S'), 'event': str(event)})

    def _input_fresh(self, now):
        return self._received is not None and 0 <= now-self._received <= self.config['input_timeout_s']

    def _pico_fresh(self, now):
        return self._pico_time is not None and 0 <= now-self._pico_time <= self.config['pico_timeout_s']

    def _deadman(self):
        return bool(self._buttons[self.config['buttons']['deadman']])

    def _zero(self):
        self._velocity = dict(forward=0., lateral=0., angular=0.)
        self.backend.set_velocity(0., 0., 0.)

    def _trip(self, message, invalidate=True):
        if self.enabled or self.mode != 'fault':
            self._log(message)
        self.enabled = False
        if invalidate:
            self.calibrated = self._pico_calibrated = False
        self.mode, self.status = 'fault', message
        self._pico_gripping = False
        self._velocity = dict(forward=0., lateral=0., angular=0.)
        try:
            self._zero()
            self.backend.stop()
        except Exception as error:
            self._log('停止通信失败，需独立物理急停／原厂遥控器接管：' + str(error))

    def input(self, payload, now=None):
        with self.lock:
            now = self.clock() if now is None else now
            if not isinstance(payload, dict):
                raise ValueError('Input must be an object')
            source = payload.get('source')
            if source not in ('simulator', 'browser_gamepad', 'linux_gamepad'):
                raise ValueError('Unknown input source')
            if self.kind == 'ros2' and source == 'simulator':
                raise ValueError('模拟输入不能控制真实 ROS2 后端')
            client = payload.get('client_id')
            seq = payload.get('sequence')
            if not isinstance(client, str) or not 1 <= len(client) <= 80 or type(seq) is not int or seq < 0:
                raise ValueError('Input requires client_id and nonnegative integer sequence')
            axes = _numbers(payload.get('axes'), 4, 'axes')
            if any(abs(v) > 1 for v in axes):
                raise ValueError('Axes must be in [-1,1]')
            buttons = payload.get('buttons')
            if not isinstance(buttons, list) or len(buttons) > 17 or any(type(b) is not bool for b in buttons):
                raise ValueError('Buttons must be a boolean array, at most17')
            buttons = buttons + [False]*(17-len(buttons))
            connected = payload.get('connected')
            if type(connected) is not bool:
                raise ValueError('connected must be boolean')
            same = client == self._owner and source == self._source
            # An absent pad is not a controller; it must not revoke standalone Pico calibration.
            if not connected and not same:
                return self.state()
            if not same and self._owner and self._input_fresh(now):
                raise ValueError('另一控制页面正在持有输入，请先停止或等待输入过期')
            if same and seq <= self._seq:
                raise ValueError('重复或倒序输入不会刷新控制时限')
            if not same:
                if self.enabled:
                    self._trip('控制输入切换，需要重新校准和使能')
                self.calibrated = self._pico_calibrated = False
                self._buttons_seen = False
                self._seq = -1
                self._owner, self._source = client, source
            self._seq = seq
            previous = self._buttons
            self._axes, self._buttons = axes, buttons
            if not connected:
                was_connected = self._received is not None
                self._received = None
                self._buttons_seen = False
                self._buttons = [False]*17
                if was_connected:
                    self.calibrated = self._pico_calibrated = False
                if self.enabled and was_connected:
                    self._trip('手柄断开，已停止；重连后需重新校准和使能')
                return self.state()
            if self._received is not None and not self._input_fresh(now) and self.enabled:
                self._trip('输入曾过期，需重新校准和使能')
            self._received = now
            if self._buttons_seen:
                stop = self.config['buttons']['stop']
                actions = ('stop',) if buttons[stop] else ('calibrate','enable','stand','walk','teleop')
                for action in actions:
                    index = self.config['buttons'][action]
                    if buttons[index] and not previous[index]:
                        try:
                            self.action(action, client_id=client)
                        except ValueError as error:
                            self.status = str(error)
                            self._log(str(error))
            self._buttons_seen = True
            if not self._deadman():
                self._zero()
            return self.state()

    def pico_frame(self, frame, source='simulator', now=None):
        with self.lock:
            now = self.clock() if now is None else now
            if source not in ('simulator', 'xrobotoolkit'):
                raise ValueError('Unknown Pico source')
            if self.kind == 'ros2' and source == 'simulator':
                raise ValueError('模拟 Pico 数据不能控制真实 ROS2 后端')
            try:
                clean = validate_pico(frame)
            except ValueError:
                if self.enabled and self.mode == 'teleop':
                    self._trip('Pico 数据无效，需要重新校准')
                raise
            if self._pico_source == source and clean['timestamp_ns'] <= self._pico_stamp:
                raise ValueError('Pico 缓存或倒序帧不会刷新跟踪时限')
            if self._pico_source != source:
                if self.enabled:
                    self._trip('Pico 输入源切换，需要重新校准')
                self._pico_calibrated = False
            if self._pico_time is not None and not self._pico_fresh(now):
                self._pico_calibrated = False
                if self.enabled and self.mode == 'teleop':
                    self._trip('Pico 输入曾中断，需要重新校准')
            self._pico_source = source
            self._pico, self._pico_stamp, self._pico_time = clean, clean['timestamp_ns'], now
            return self.state()

    def _measured_upper(self):
        snap = self.backend.snapshot()
        age = snap.get('joints_age_s')
        if not snap.get('connected') or age is None or age > self.config['ros2']['feedback_timeout_s']:
            raise ValueError('关节状态缺失或过期')
        measured = snap.get('joints', {})
        names = self.config['upper']['limits']
        if any(n not in measured or type(measured[n]) not in (int,float) or not math.isfinite(measured[n]) for n in names):
            raise ValueError('必须收到全部10个上肢和腰部关节的有效状态')
        return {n: float(measured[n]) for n in names}

    def action(self, name, client_id=None):
        with self.lock:
            now = self.clock()
            if name not in ('enable','disable','calibrate','stand','walk','teleop','stop','clear_fault'):
                raise ValueError('Unknown action')
            if name not in ('stop','disable') and client_id and self._owner and client_id != self._owner and self._input_fresh(now):
                raise ValueError('操作页面未持有控制输入')
            if name in ('stop','disable'):
                self.enabled = False
                self.mode, self.status = 'idle', '已停止输出，保留原厂平衡控制'
                self._pico_gripping = False
                self._zero()
                self.backend.stop()
                self._log('软件停止；不替代物理急停')
            elif name == 'clear_fault':
                if self.enabled:
                    raise ValueError('请先停止')
                self.mode, self.status = 'idle', '故障已清除，需要重新校准和使能'
                self.calibrated = self._pico_calibrated = False
                self._log(self.status)
            elif name == 'calibrate':
                if self.mode not in ('idle','stand'):
                    raise ValueError('先停止运动，再校准')
                fresh_pad = self._input_fresh(now)
                fresh_pico = self._pico_fresh(now)
                if not fresh_pad and not fresh_pico:
                    raise ValueError('没有新鲜的手柄或 Pico 输入')
                if fresh_pad and (self._deadman() or any(abs(v) > self.config['neutral_threshold'] for v in self._axes)):
                    raise ValueError('校准前请松开使能保持键，并让摇杆回到中位')
                if fresh_pico and (self._pico['left_grip'] >= .7 or self._pico['right_grip'] >= .7):
                    raise ValueError('Pico 校准前请松开双手握持键')
                if fresh_pico and self.retargeter:
                    self.retargeter.calibrate(self._pico, self._measured_upper())
                    self._pico_calibrated = True
                    self._processed_stamp = 0
                if fresh_pad:
                    self._offsets = list(self._axes)
                self.calibrated = True
                self.status = '已校准摇杆中位／Pico 参考坐标；未改动电机零位'
                self._log(self.status)
            elif name == 'enable':
                if self.mode == 'fault':
                    raise ValueError('先清除故障并重新校准')
                if not self.calibrated or not (self._input_fresh(now) or self._pico_fresh(now)):
                    raise ValueError('必须先连接输入并完成校准')
                if self._input_fresh(now) and (self._deadman() or any(abs(a-b) > self.config['neutral_threshold'] for a,b in zip(self._axes,self._offsets))):
                    raise ValueError('使能前请松开保持键并让摇杆回中')
                snap = self.backend.snapshot()
                age = snap.get('joints_age_s')
                if not snap.get('connected') or age is None or age > self.config['ros2']['feedback_timeout_s']:
                    raise ValueError('机器人未连接或反馈过期')
                self.backend.enable()
                self.backend.request_mode('stand')
                self.enabled, self.mode = True, 'stand'
                self._source_was_active = False
                self.status = '已请求使能，等待原厂运控就绪'
                self._log(self.status)
            else:
                if not self.enabled:
                    raise ValueError('请先使能')
                if name == 'teleop':
                    if self.kind == 'ros2' and not self.config['upper']['hardware_limits_confirmed']:
                        raise ValueError('真实上肢控制需要先确认并配置本体限位／零位参数')
                    if not self._pico_fresh(now) or not self._pico_calibrated or self.retargeter is None:
                        raise ValueError('先接入并校准 Pico')
                    self._upper = self._measured_upper()
                self._zero()
                self._pico_gripping = False
                self.backend.request_mode(name)
                self.mode, self.status = name, '模式切换中：' + name
                self._log(self.status)
            return self.state()

    def _axis_velocity(self, value, maximum):
        dead = self.config['deadzone']
        if abs(value) <= dead:
            return 0.
        amount = min(1., (abs(value)-dead)/(1-dead))
        minimum = self.config['velocity']['min_nonzero']
        return math.copysign(minimum + amount*(maximum-minimum), value)

    def tick(self, now=None):
        with self.lock:
            now = self.clock() if now is None else now
            if not self.enabled:
                return
            try:
                fresh = self._pico_fresh(now) if self.mode == 'teleop' else self._input_fresh(now) or (self.mode == 'stand' and self._pico_fresh(now))
                if not fresh:
                    self._trip('控制输入过期，已停止；需重新校准和使能')
                    return
                snap = self.backend.snapshot()
                age = snap.get('joints_age_s')
                if not snap.get('connected') or snap.get('fault') or age is None or age > self.config['ros2']['feedback_timeout_s']:
                    self._trip('机器人反馈异常：' + str(snap.get('fault') or '连接中断'))
                    return
                if self._source_was_active and not snap.get('source_active'):
                    self._trip('控制源被抢占，已停止，需重新使能')
                    return
                expected = 'BIPED_CUSTOM_UPPER' if self.mode == 'teleop' else 'BIPED_WALK_RUN'
                ready = snap.get('action') == expected and snap.get('action_running')
                self._source_was_active |= bool(snap.get('source_active'))
                if not ready:
                    self._zero()
                    self.status = '等待运控模式确认：' + expected
                    return
                self.status = {'walk':'行走模式：按住保持键才发送速度','stand':'平衡站立：零速度','teleop':'Pico 遥操：双手握持才跟随'}[self.mode]
                if self.mode == 'walk' and self._input_fresh(now) and self._deadman() and snap.get('source_active'):
                    a = [max(-1.,min(1.,v-o)) for v,o in zip(self._axes,self._offsets)]
                    v = self.config['velocity']
                    self._velocity = dict(forward=self._axis_velocity(-a[1],v['max_forward']),lateral=self._axis_velocity(-a[0],v['max_lateral']),angular=self._axis_velocity(-a[2],v['max_angular']))
                    self.backend.set_velocity(**self._velocity)
                else:
                    self._zero()
                if self.mode == 'teleop':
                    if not snap.get('source_active'):
                        self.status = '等待已注册输入源获得控制权'
                        self._pico_gripping = False
                        return
                    gripping = self._pico['left_grip'] >= .7 and self._pico['right_grip'] >= .7
                    if gripping and self._processed_stamp != self._pico_stamp:
                        if not self._pico_gripping:
                            self._upper = self._measured_upper()
                            self.retargeter.calibrate(self._pico, self._upper)
                        dt = .01 if self._pico_processed_time is None else min(.05,max(.001,now-self._pico_processed_time))
                        candidate = self.retargeter.retarget(self._pico, dt=dt)
                        limits = self.config['upper']['limits']
                        if set(candidate) != set(limits):
                            raise ValueError('IK 未返回完整10个关节')
                        for joint, value in candidate.items():
                            lo, hi = limits[joint]
                            if not math.isfinite(value) or not lo <= value <= hi:
                                raise ValueError('IK 超出关节范围：' + joint)
                            if abs(value-self._upper[joint]) > self.config['upper']['max_joint_step_rad']:
                                raise ValueError('IK 目标变化过大：' + joint)
                        self._upper = candidate
                        self._processed_stamp = self._pico_stamp
                        self._pico_processed_time = now
                    self._pico_gripping = gripping
                    self.backend.set_upper(dict(self._upper))
            except Exception as error:
                self._trip('控制停止：' + str(error))

    def state(self):
        with self.lock:
            now = self.clock()
            pico_status = self.retargeter.status if self.retargeter else {}
            return dict(backend=self.kind,enabled=self.enabled,calibrated=self.calibrated,mode=self.mode,status=self.status,
                robot=self.backend.snapshot(),
                input=dict(source=self._source,age_ms=None if self._received is None else round(max(0.,now-self._received)*1000,1),deadman=self._deadman(),client_id=self._owner),
                pico=dict(connected=self._pico_fresh(now),calibrated=self._pico_calibrated,age_ms=None if self._pico_time is None else round(max(0.,now-self._pico_time)*1000,1),source=self._pico_source,frame=copy.deepcopy(self._pico),ik_error_m=pico_status.get('error_m'),diagnostic=pico_status),
                command=dict(velocity=dict(self._velocity),upper=dict(self._upper)),events=list(self.events),
                limits=dict(hardware_upper_confirmed=self.config['upper']['hardware_limits_confirmed'],velocity=self.config['velocity']),
                supermimic=dict(available=False,reason='28,000模型仅完成仿真评估；后续需IMU、标定与互斥低层控制器'))
