"""Exercise actual Humble messages/DDS locally, with a fake MC peer and no robot.

Run only in the documented --network none container; never on the robot LAN.
"""
import json
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import aimdk_msgs.msg as msg
import aimdk_msgs.srv as srv

from q1_edge.backends import Ros2Backend, UPPER_HOME
from q1_edge.config import load_config


class Peer(Node):
    def __init__(self):
        super().__init__('fake_q1_loopback')
        self.action = 'BIPED_WALK_RUN'
        self.owner = ''
        self.registered = False
        self.feedback = True
        self.pose = dict(UPPER_HOME)
        self.calls, self.velocity, self.upper = [], [], []
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joints = self.create_publisher(msg.JointStateArray, '/aima/hal/joint/state', best)
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
            response.response.state.value = msg.CommonState.SUCCESS
            if name == 'SetMcAction':
                self.action = request.command.action_desc
            elif name == 'SetMcInputSource':
                self.registered = request.action.value == 1001
                if not self.registered:
                    self.owner = ''
            else:
                response.input_source.name = self.owner
        return response

    def publish_joints(self):
        if not self.feedback:
            return
        packet = msg.JointStateArray()
        packet.header.stamp = self.get_clock().now().to_msg()
        for name, value in self.pose.items():
            joint = msg.JointState()
            joint.name, joint.position = name, value
            packet.joints.append(joint)
        self.joints.publish(packet)

    def on_velocity(self, packet):
        self.velocity.append((time.monotonic(), packet))
        if self.registered:
            self.owner = packet.source

    def on_upper(self, packet):
        self.upper.append(packet)
        self.pose = {joint.name:joint.position for joint in packet.joints}


def main():
    rclpy.init()
    peer = Peer()
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
        robot.enable()
        wait(lambda:robot.snapshot()['source_active'], 'source arbitration')
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
        robot.enable()
        wait(lambda:robot.snapshot()['source_active'], 'explicit re-enable')
        peer.feedback = False
        wait(lambda:bool(robot.snapshot()['fault']), 'independent feedback watchdog')
        assert all(packet.forward_velocity==0. for _,packet in peer.velocity[-3:])
        print(json.dumps(dict(passed=True,transport='real ROS2 Humble DDS, fake MC peer',
            readonly_startup=True,source_arbitration=True,velocity_heartbeat=True,
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
