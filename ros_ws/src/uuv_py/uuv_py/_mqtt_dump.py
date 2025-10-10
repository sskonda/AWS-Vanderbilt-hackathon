import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from custom_msg.msg import Event

from awsiot import mqtt5_client_builder
from awscrt import mqtt5

class MQTT5Dump(Node):
    """Publishes a custom_msg/Event when a sub (header.frame_id starts with 'sub')
       enters a proximity radius around this node's own PoseStamped."""
    def __init__(self):
        super().__init__('MQTT5Dump')
        self.declare_parameters('', [
            ('client_id', 'basicPubSub'),
            ('endpoint', 'a2g3u8sh1ylxq-ats.iot.us-east-1.amazonaws.com'),
            ('cert_filepath', '/home/kapow/Development/AWS-Vanderbilt-hackathon/aws_iot_core/Nikhil-LAPTOP.cert.pem'),
            ('pri_key_filepath', '/home/kapow/Development/AWS-Vanderbilt-hackathon/aws_iot_core/Nikhil-LAPTOP.private.key'),
            ('ca_filepath', '/home/kapow/Development/AWS-Vanderbilt-hackathon/aws_iot_core/root-CA.crt'), 
        ])

        self.client = mqtt5_client_builder.mtls_from_path(
            endpoint=self.get_parameter('endpoint').value,
            cert_filepath=self.get_parameter('cert_filepath').value,
            pri_key_filepath=self.get_parameter('pri_key_filepath').value,
            client_id=self.get_parameter('client_id').value,
            ca_filepath=self.get_parameter('ca_filepath').value,
            on_publish_received=self._on_publish_received,
            on_lifecycle_stopped=self._on_lifecycle_stopped,
            on_lifecycle_attempting_connect=self._on_lifecycle_attempting_connect,
            on_lifecycle_connection_success=self._on_lifecycle_connection_success,
            on_lifecycle_connection_failure=self._on_lifecycle_connection_failure,
            on_lifecycle_disconnection=self._on_lifecycle_disconnection,
        )
        self.connected = False

        # Parameters
        qos = QoSProfile(depth=10)
        self.create_subscription(PoseStamped, '/submarines/pose', self._on_pose, qos)

        self.create_timer(1.0, self._tick)
        self.pose_buffer = []
        self.client.start()

    def _on_pose(self, msg: PoseStamped):
        """Store own position."""
        # self.get_logger().info(f"Pose received: {msg}")
        if not self.pose_buffer:
            self.pose_buffer.append(PoseStamped(pose=msg.pose, header=msg.header))
        self.pose_buffer[0] = PoseStamped(pose=msg.pose, header=msg.header)

    def _on_publish_received(self, publish_packet_data):
        self.get_logger().info(f"Published: {publish_packet_data.puback.reason_code}")
        
    def _on_lifecycle_stopped(self, lifecycle_stopped_data):
        self.get_logger().info("MQTT client stopped")
        self.connected = False
    
    def _on_lifecycle_attempting_connect(self, lifecycle_attempting_connect_data):
        self.get_logger().info("Attempting to connect to AWS IoT Core")
        self.connected = False

    def _on_lifecycle_connection_success(self, lifecycle_connection_success_data):
        self.get_logger().info("Connected to AWS IoT Core")
        self.connected = True
        
    def _on_lifecycle_connection_failure(self, lifecycle_connection_failure_data):
        self.get_logger().error("Connection to AWS IoT Core failed")
        self.connected = False
        
    def _on_lifecycle_disconnection(self, lifecycle_disconnection_data):
        self.get_logger().warning("Disconnected from AWS IoT Core")
        self.connected = False

    def _tick(self):
        """Periodically check connection and dump to IoT."""
        if self.connected:
            while self.pose_buffer:
                pose_msg = self.pose_buffer.pop()
                payload = f"Pose at {pose_msg.header.stamp.sec}.{pose_msg.header.stamp.nanosec}: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}, {pose_msg.pose.position.z}"
                publish_future = self.client.publish(mqtt5.PublishPacket(
                    topic="uuv/pose",
                    payload=payload,
                    qos=mqtt5.QoS.AT_LEAST_ONCE
                ))
                publish_future.add_done_callback(self._on_publish_received)
                # self.get_logger().info(f"Published to IoT: {payload}")

def main():
    rclpy.init()
    node = MQTT5Dump()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# example use case run node 
# ros2 run uuv_sim_py ProximityEvent_pub \
#   --ros-args -p threshold_m:=50.0 -p event_topic:=/events/prox_hits