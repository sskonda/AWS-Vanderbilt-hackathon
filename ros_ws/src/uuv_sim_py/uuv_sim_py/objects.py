import json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from custom_msg.msg import Event

class Event:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()
        self.event_type = ""
        self.message = ""

def xyz(ps: PoseStamped):
    p = ps.pose.position
    return float(p.x), float(p.y), float(p.z)

class ProximityEvent(Node):
    """Publishes a custom_msg/Event when a sub (header.frame_id starts with 'sub')
       enters a proximity radius around this node's own PoseStamped."""
    def __init__(self):
        super().__init__('ProximityEvent')

        # Parameters 
        self.declare_parameters('', [
            ('self_topic', '/event/pose'),
            ('event_topic', '/events/event_1'),
            ('event_type', 'PROXIMITY_HIT'),
            ('message', 'Sub entered event radius'),
            ('threshold_m', 50.0),
            ('min_interval_s', 2.0),
        ])
        gp = self.get_parameter
        self.self_topic     = gp('self_topic').value
        self.event_topic    = gp('event_topic').value
        self.event_type     = gp('event_type').value
        self.message_text   = gp('message').value
        self.threshold_m    = float(gp('threshold_m').value)
        self.min_interval_s = float(gp('min_interval_s').value)

        qos = QoSProfile(depth=10)
        self.pub_evt = self.create_publisher(Event, self.event_topic, qos)

        # Pose from this node’s own simulated object in gazebo????
        self.self_pose: PoseStamped | None = None
        self.create_subscription(PoseStamped, self.self_topic, self._cb_self, qos)

        # All incoming sub poses auto detect 
        self.sub_poses = {}      # frame_id -> PoseStamped
        self.last_inside = {}
        self.last_pub = {}

        # sub to pose stamped 
        self.create_subscription(PoseStamped, '/all_pose_data', self._cb_all_poses, qos)

        # Background check loop
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'ProximityEvent watching all PoseStamped messages with frame_id starting "sub", '
            f'threshold={self.threshold_m}m, publishing to {self.event_topic}'
        )

    def _cb_self(self, msg: PoseStamped):
        """Store own position."""
        self.self_pose = msg

    def _cb_all_poses(self, msg: PoseStamped):
        """Capture all PoseStamped topics with header.frame_id starting 'sub'."""
        fid = msg.header.frame_id.lower()
        if fid.startswith('sub'):
            self.sub_poses[fid] = msg

    def _tick(self):
        """Periodically check proximity and publish events."""
        if self.self_pose is None or not self.sub_poses:
            return

        sx, sy, sz = xyz(self.self_pose)
        now = time.time()

        for fid, ps in list(self.sub_poses.items()):
            x, y, z = xyz(ps)
            dist = math.sqrt((sx-x)**2 + (sy-y)**2 + (sz-z)**2)
            inside = dist <= self.threshold_m
            was_inside = self.last_inside.get(fid, False)

            if inside and not was_inside and (now - self.last_pub.get(fid, 0.0)) >= self.min_interval_s:
                self.last_pub[fid] = now

                evt = Event()
                evt.header.stamp = self.get_clock().now().to_msg()
                evt.header.frame_id = self.self_pose.header.frame_id or 'world'
                evt.pose = self.self_pose.pose
                evt.event_type = self.event_type
                evt.message = f'{self.message_text} (triggered by {fid}, d={dist:.2f} m)'

                self.pub_evt.publish(evt)
                self.get_logger().info(f"Event published for {fid}: {evt.message}")

            self.last_inside[fid] = inside


def main():
    rclpy.init()
    node = ProximityEvent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# example use case run node 
# ros2 run uuv_sim_py ProximityEvent_pub \
#   --ros-args -p threshold_m:=50.0 -p event_topic:=/events/prox_hits