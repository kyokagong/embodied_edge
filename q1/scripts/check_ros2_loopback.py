"""Exercise actual Humble messages/DDS locally, with a fake MC peer and no robot.

Run only in the documented --network none container; never on the robot LAN.
"""
import argparse
import json
import threading
import time

import rclpy
from sensor_msgs.msg import JointState as StandardJointState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import aimdk_msgs.msg as msg
import aimdk_msgs.srv as srv

from q1_edge.backends import Ros2Backend, UPPER_HOME
from q1_edge.config import load_config
from q1_edge.core import Controller


class Peer(Node):
    def __init__(self, standard=False):
        self.standard = standard
        super().__init__('fake_q1_loopback')
        self.action = 'BIPED_WALK_RUN'
        self.owner = ''
        self.registered = False
        self.feedback = True
        self.pose = dict(UPPER_HOME)
        self.calls, self.velocity, self.upper = [], [], []
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joints = self.create_publisher(
            StandardJointState if standard else msg.JointStateArray,
            '/upper_body_joint_states' if standard else '/aima/hal/joint/state', best)
        self.create_subscription(msg.McLocomotionVelocity, '/aima/mc/locomotion/velocity', self.on_velocity, 10)
        self.create_subscription(msg.McCustomJointCommand, '/aima/mc/custom/joint/command', self.on_upper, best)
        for name in ('GetMcAction','SetMcAction','GetCurrentInputSource','SetMcInputSource'):
            self.create_service(getattr(srv,name), '/aimdk_5Fmsgs/srv/'+name,
                                lambda request,response,n=name: self.rpc(n,request,response))
        self.create_timer(.01, self.publish_joints)

    def rpc(self, name, request, response):
        self.calls.append(name)
        if name == 'GetMcAction':
            response.header.code = 0
            response.info.action_desc = self.action
            response.info.status.value = msg.McActionStatus.RUNNING
            response.info.switch_ready = True
        else:
            response.response.header.code = 0
            response.response.state.value = (msg.CommonState.UNKNOWN
                if self.standard and name == "SetMcInputSource" else msg.CommonState.SUCCESS)
            if name == 'SetMcAction':
                self.action = request.command.action_desc
            elif name == 'SetMcInputSource':
                self.registered = request.action.value == 1001
                if not self.registered and not self.standard:
                    self.owner = ''
            else:
                if self.standard:
                    response.response.state.value = msg.CommonState.UNKNOWN
                response.input_source.name = self.owner
        return response

    def publish_joints(self):
        if not self.feedback:
            return
        packet = StandardJointState() if self.standard else msg.JointStateArray()
        packet.header.stamp = self.get_clock().now().to_msg()
        for name, value in self.pose.items():
            if self.standard:
                packet.name.append(name)
                packet.position.append(value)
            else:
                joint = msg.JointState()
                joint.name, joint.position = name, value
                packet.joints.append(joint)
        self.joints.publish(packet)

    def on_velocity(self, packet):
        self.velocity.append((time.monotonic(), packet))
        if self.registered and (not self.standard or any((packet.forward_velocity, packet.lateral_velocity, packet.angular_velocity))):
            self.owner = packet.source

    def on_upper(self, packet):
        self.upper.append(packet)
        self.pose = {joint.name:joint.position for joint in packet.joints}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--standard-joints', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    peer = Peer(standard=args.standard_joints)
    executor = SingleThreadedExecutor()
    executor.add_node(peer)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    robot = None
    try:
        robot = Ros2Backend(load_config())
        def wait(predicate, description, timeout=8, keepalive=None):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                if keepalive:
                    keepalive()
                if predicate():
                    return
                time.sleep(.01)
            raise AssertionError((description, robot.snapshot()))
        wait(lambda:robot.snapshot()['connected'], 'fresh real DDS feedback')
        assert not peer.registered and not peer.velocity and not peer.upper, 'startup must be read-only'
        controller = Controller(robot, load_config())
        sequence = 0
        def gamepad(buttons=(), axes=(0., 0., 0., 0.), connected=True):
            nonlocal sequence
            sequence += 1
            controller.input(dict(client_id='loopback-pad', source='browser_gamepad',
                sequence=sequence, axes=list(axes),
                buttons=[i in buttons for i in range(17)], connected=connected))
            controller.tick()
        gamepad()
        gamepad((8,)); gamepad()  # Back: calibrate; release before next edge.
        assert controller.state()['calibrated']
        gamepad((9,)); gamepad()  # Start: explicit enable.
        wait(lambda:robot._registered, 'gamepad source registered', keepalive=gamepad)
        if args.standard_joints:
            assert not robot.snapshot()['source_active'], 'zero must not acquire the real firmware source'
        gamepad((2,)); gamepad()  # X: walk.
        moving = lambda: gamepad((5,), (0., -.5, 0., 0.))
        wait(lambda:robot.snapshot()['source_active'] and any(p.forward_velocity > .2 for _,p in peer.velocity),
             'gamepad frame to real DDS velocity', keepalive=moving)
        assert controller.state()['enabled'], controller.state()
        gamepad()  # Recenter zeros; released RB keeps physical enable latched.
        wait(lambda:peer.velocity[-1][1].forward_velocity == 0., 'joystick recenter zeros velocity', keepalive=gamepad)
        before = len(peer.velocity)
        moving = lambda: gamepad((), (0., -.5, 0., 0.))
        wait(lambda:any(p.forward_velocity > .2 for _,p in peer.velocity[before:]),
             'latched enable moves with RB released', keepalive=moving)
        for axis, sign in ((-.6,1),(.6,-1)):
            before=len(peer.velocity)
            wait(lambda:any(p.angular_velocity*sign > .1 for _,p in peer.velocity[before:]),
                 'right stick body yaw both signs',keepalive=lambda:gamepad((),(0.,0.,axis,0.)))
        gamepad()
        def hold_rb():
            started=time.monotonic()
            while time.monotonic()-started < 1.15:
                gamepad((5,));time.sleep(.02)
        hold_rb()
        assert not controller.state()['enabled'], 'second long hold must disable'
        assert not controller.state()['input']['motion_latched']
        gamepad();hold_rb();gamepad()
        assert controller.state()['enabled'], 'fresh separate long hold must re-enable'
        gamepad((2,));gamepad()
        wait(lambda:robot.snapshot()['source_active'], 'fresh source acquired after toggle',keepalive=moving)
        gamepad(connected=False)
        wait(lambda:not peer.registered and not robot._registered
             and 'SetMcInputSource' not in robot._pending, 'pad disconnect stops and releases source')
        assert not controller.state()['enabled'] and not controller.state()['calibrated']
        assert peer.velocity[-1][1].forward_velocity == 0.
        def acquire_backend():
            robot.enable()
            wait(lambda:robot.snapshot()['source_active'] or robot.snapshot()['source_can_acquire'],
                 'source ready for explicit motion')
            robot.request_mode('walk')
            wait(lambda:robot.snapshot()['source_active'], 'nonzero source arbitration',
                 keepalive=lambda:robot.set_velocity(.22,0.,0.))
            robot.set_velocity(0.,0.,0.)
        acquire_backend()
        robot.request_mode('walk')
        robot.set_velocity(.22,0.,0.)
        wait(lambda:any(packet.forward_velocity == .22 for _,packet in peer.velocity), 'walking velocity')
        start = time.monotonic()
        while time.monotonic()-start < .35:
            robot.set_velocity(.22,0.,0.)
            time.sleep(.02)
        assert sum(t >= start for t,_ in peer.velocity) >= 10, 'velocity heartbeat absent'
        robot.set_velocity(0.,0.,0.)
        robot.request_mode('teleop')
        wait(lambda:robot.snapshot()['action']=='BIPED_CUSTOM_UPPER', 'confirmed upper mode')
        pose = dict(UPPER_HOME)
        pose['left_elbow_joint'] += .01
        robot.set_upper(pose)
        wait(lambda:bool(peer.upper), 'complete upper stream', keepalive=lambda:robot.set_upper(pose))
        assert all(len(packet.joints)==10 for packet in peer.upper)
        assert all(joint.stiffness==20. and joint.damping==3. for packet in peer.upper for joint in packet.joints)
        robot.stop()
        wait(lambda:not peer.registered and not robot.snapshot()['source_active'], 'balanced stop and source release')
        time.sleep(.1)
        assert peer.action == 'BIPED_WALK_RUN'
        acquire_backend()
        peer.feedback = False
        wait(lambda:bool(robot.snapshot()['fault']), 'independent feedback watchdog')
        assert all(packet.forward_velocity==0. for _,packet in peer.velocity[-3:])
        print(json.dumps(dict(passed=True,transport='real ROS2 Humble DDS, fake MC peer',
            readonly_startup=True,source_arbitration=True,velocity_heartbeat=True,
            physical_gamepad_pipeline=True,joystick_recenter_zero=True,rb_latch_toggle=True,body_yaw_both_signs=True,pad_disconnect_zero=True,
            joint_format="sensor_msgs/JointState" if args.standard_joints else "aimdk_msgs/JointStateArray",
            complete_upper_stream=True,balanced_stop_release=True,feedback_watchdog=True)))
    finally:
        if robot:
            robot.close()
        executor.shutdown(timeout_sec=1)
        worker.join(timeout=1)
        peer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
