#!/usr/bin/env python3
import math
import random
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from custom_msg.msg import Event

class SwarmSim(Node):
    """
    Simple orchestrator:
      - Subscribes to /subA|subB|subC/pose (logs)
      - Periodically publishes events on /events/event_1 to drive reactions:
          * proximity_hit
          * foreign_uuv
          * swarm_uuv
          * pipeline break
      - Lets you change cadence with params.
    """

    def __init__(self):
        super().__init__('swarm_sim')

        self.declare_parameter('proximity_period_s', 20.0)
        self.declare_parameter('foreign_period_s',   35.0)
        self.declare_parameter('swarm_period_s',     50.0)
        self.declare_parameter('pipeline_period_s',  65.0)

        self.prox_T = float(self.get_parameter('proximity_period_s').value)
        self.foreign_T = float(self.get_parameter('foreign_period_s').value)
        self.swarm_T = float(self.get_parameter('swarm_period_s').value)
        self.pipe_T = float(self.get_parameter('pipeline_period_s').value)

        qos = QoSProfile(depth=10)

        # --- pose subscribers for 3 subs ---
        self.poses: Dict[str, PoseStamped] = {}
        for sub_id in ('subA', 'subB', 'subC'):
            topic = f'/{sub_id}/pose'
            self.create_subscription(PoseStamped, topic,
                                     self._mk_pose_cb(sub_id), qos)
            self.get_logger().info(f'Listening: {topic}')

        # --- event publisher (what your C++ node listens to) ---
        self.pub_evt = self.create_publisher(Event, '/events/event_1', qos)

        # --- timers to generate events ---
        self.create_timer(self.prox_T,    self.send_proximity)
        self.create_timer(self.foreign_T, self.send_foreign)
        self.create_timer(self.swarm_T,   self.send_swarm_request)
        self.create_timer(self.pipe_T,    self.send_pipeline_break)

        self.get_logger().info('SwarmSim ready: publishing events to /events/event_1')

    def _mk_pose_cb(self, sub_id: str):
        def _cb(msg: PoseStamped):
            self.poses[sub_id] = msg
            # compact log every few seconds (don’t spam; here we just log occasionally)
        return _cb

    # --- helpers to craft an Event ---
    def _evt(self, frame_id: str, etype: str, message: str, x=0.0, y=0.0, z=-20.0) -> Event:
        e = Event()
        e.header.stamp = self.get_clock().now().to_msg()
        e.header.frame_id = frame_id
        e.event_type = etype
        e.message = message
        e.pose.position.x = float(x)
        e.pose.position.y = float(y)
        e.pose.position.z = float(z)
        return e

    # --- timers fire these ---
    def send_proximity(self):
        # Pick a target sub to “nudge”
        target = random.choice(('subA','subB','subC'))
        p = self.poses.get(target)
        if p:
            e = self._evt(frame_id=target, etype='proximity_hit',
                          message=f'auto-sim proximity near {target}',
                          x=p.pose.position.x, y=p.pose.position.y, z=p.pose.position.z)
        else:
            e = self._evt(frame_id=target, etype='proximity_hit',
                          message='auto-sim proximity without known pose')
        self.pub_evt.publish(e)
        self.get_logger().info(f'[sim] proximity_hit -> {target}')

    def send_foreign(self):
        # “Foreign UUV” sighting around whichever patrol we know
        patrol = random.choice(('subA','subB'))
        px = random.uniform(0, 200)
        py = random.uniform(0, 100)
        e = self._evt(frame_id=patrol, etype='foreign_uuv',
                      message='unknown contact detected',
                      x=px, y=py, z=-18.0)
        self.pub_evt.publish(e)
        self.get_logger().info(f'[sim] foreign_uuv -> {patrol} @({px:.1f},{py:.1f})')

    def send_swarm_request(self):
        # Ask mid-tier (or any) to dump data (your C++ node replies with DATA_DUMP on /events/status)
        asker = random.choice(('subA','subB'))
        e = self._evt(frame_id=asker, etype='swarm_uuv', message='requesting recent telemetry')
        self.pub_evt.publish(e)
        self.get_logger().info(f'[sim] swarm_uuv -> {asker} (request DATA_DUMP)')

    def send_pipeline_break(self):
        # Simulate a pipeline break near random coords
        px = random.uniform(20, 180)
        py = random.uniform(10, 90)
        e = self._evt(frame_id='sim', etype='pipeline break',
                      message='pressure drop & leak signatures',
                      x=px, y=py, z=-20.0)
        self.pub_evt.publish(e)
        self.get_logger().warn(f'[sim] pipeline break @({px:.1f},{py:.1f})')

def main():
    rclpy.init()
    node = SwarmSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
