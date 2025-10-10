import rclpy
import threading
from std_msgs.msg import String
from rclpy.node import Node

from awsiot import mqtt5_client_builder
from awscrt import mqtt5

class MQTT5Dump(Node):
    def __init__(self):
        super().__init__('mqtt5_dump')

        # --- Parameters
        self.declare_parameters('', [
            ('endpoint', 'a2g3u8sh1ylxq-ats.iot.us-east-1.amazonaws.com'),
            ('cert_filepath', '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/Nikhil-LAPTOP.cert.pem'),
            ('pri_key_filepath', '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/Nikhil-LAPTOP.private.key'),
            ('ca_filepath', '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/root-CA.crt'),
            ('client_id', 'subA_2'),

            ('ros_input_topic', '/relay/snapshot_json'),   # your planner publishes JSON String here
            ('mqtt_topic', 'uuv/relay/snapshot')           # where to publish in AWS IoT
        ])

        self.connected = False
        self.stopped_event = threading.Event()
        self.connection_success_event = threading.Event()
        self.out_buffer = []  # queue of JSON strings to send

        # --- MQTT5 client
        self.client = mqtt5_client_builder.mtls_from_path(
            endpoint=self.get_parameter('endpoint').value,
            cert_filepath=self.get_parameter('cert_filepath').value,
            pri_key_filepath=self.get_parameter('pri_key_filepath').value,
            ca_filepath=self.get_parameter('ca_filepath').value,
            client_id=self.get_parameter('client_id').value,
            on_publish_received=self.on_publish_received,
            on_lifecycle_stopped=self.on_lifecycle_stopped,
            on_lifecycle_attempting_connect=self.on_lifecycle_attempting_connect,
            on_lifecycle_connection_success=self.on_lifecycle_connection_success,
            on_lifecycle_connection_failure=self.on_lifecycle_connection_failure,
            on_lifecycle_disconnection=self.on_lifecycle_disconnection,
        )

        # Start once and keep alive
        self.client.start()

        # --- Subscribe to planner JSON
        ros_input_topic = self.get_parameter('ros_input_topic').value
        self.json_subscription = self.create_subscription(
            String, ros_input_topic, self._on_json, 10
        )

        # Publish at a modest rate (don’t spam MQTT)
        self.tick = self.create_timer(0.5, self._tick)

    def _on_json(self, msg: String):
        """Buffer a JSON message from the planner for MQTT publish."""
        self.get_logger().debug(f"JSON received: {len(msg.data)} bytes")
        self.out_buffer.append(msg.data.encode('utf-8'))

    def _tick(self):
        if not self.out_buffer:
            return

        if not self.connected:
            # Wait for connection to establish; non-blocking here
            self.get_logger().warn("MQTT not connected yet; will retry next tick")
            return

        payload = self.out_buffer.pop(0)
        topic = self.get_parameter('mqtt_topic').value
        self.get_logger().info(f"Publishing {len(payload)} bytes to '{topic}'")
        publish_future = self.client.publish(mqtt5.PublishPacket(
            topic=topic,
            payload=payload,
            qos=mqtt5.QoS.AT_LEAST_ONCE
        ))
        _ = publish_future.result(10)

    # --- MQTT lifecycle callbacks
    def on_publish_received(self, publish_packet_data):
        publish_packet = publish_packet_data.publish_packet
        self.get_logger().info(f"RX on '{publish_packet.topic}': {publish_packet.payload.decode('utf-8')}")

    def on_lifecycle_stopped(self, _):
        self.get_logger().warn("MQTT Lifecycle Stopped")
        self.connected = False
        self.stopped_event.set()

    def on_lifecycle_attempting_connect(self, _):
        self.get_logger().info("MQTT attempting connect…")
        self.connected = False

    def on_lifecycle_connection_success(self, lifecycle_connect_success_data):
        rc = lifecycle_connect_success_data.connack_packet.reason_code
        self.get_logger().info(f"MQTT connected ({rc})")
        self.connection_success_event.set()
        self.connected = True

    def on_lifecycle_connection_failure(self, lifecycle_connection_failure):
        self.get_logger().error(f"MQTT connection failure: {lifecycle_connection_failure.exception}")
        self.connected = False

    def on_lifecycle_disconnection(self, lifecycle_disconnect_data):
        reason = lifecycle_disconnect_data.disconnect_packet.reason_code if lifecycle_disconnect_data.disconnect_packet else "None"
        self.get_logger().warn(f"MQTT disconnected (reason={reason})")
        self.connected = False

def main(args=None):
    rclpy.init(args=args)
    node = MQTT5Dump()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
