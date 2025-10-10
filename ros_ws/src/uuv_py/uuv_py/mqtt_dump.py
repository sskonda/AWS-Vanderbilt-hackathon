import rclpy
import threading
from std_msgs.msg import String 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose

from awsiot import mqtt5_client_builder
from awscrt import mqtt5


class MQTT5Dump(Node):
    def __init__(self):
        super().__init__('MQTT5Dump')
        self.declare_parameters('', [
            ('endpoint', 'a2g3u8sh1ylxq-ats.iot.us-east-1.amazonaws.com'),
            ('cert_filepath', '/home/kapow/Development/AWS-Vanderbilt-hackathon/aws_iot_core/Nikhil-LAPTOP.cert.pem'),
            ('pri_key_filepath', '/home/kapow/Development/AWS-Vanderbilt-hackathon/aws_iot_core/Nikhil-LAPTOP.private.key'),
            ('ca_filepath', '/home/kapow/Development/AWS-Vanderbilt-hackathon/aws_iot_core/root-CA.crt'), 
            ('client_id', 'Nikhil-LAPTOP'),
        ])
        
        self.client = mqtt5_client_builder.mtls_from_path(
            endpoint=self.get_parameter('endpoint').value,
            cert_filepath=self.get_parameter('cert_filepath').value,
            pri_key_filepath=self.get_parameter('pri_key_filepath').value,
            on_publish_received=self.on_publish_received,
            on_lifecycle_stopped=self.on_lifecycle_stopped,
            on_lifecycle_attempting_connect=self.on_lifecycle_attempting_connect,
            on_lifecycle_connection_success=self.on_lifecycle_connection_success,
            on_lifecycle_connection_failure=self.on_lifecycle_connection_failure,
            on_lifecycle_disconnection=self.on_lifecycle_disconnection,
            client_id=self.get_parameter('client_id').value,
            ca_filepath=self.get_parameter('ca_filepath').value
        )
        self.connected = False

        self.stopped_event = threading.Event()
        self.connection_success_event = threading.Event()
        self.pose_buffer = []

        # # Subscribe
        # print("==== Subscribing to topic '{}' ====".format(message_topic))
        # subscribe_future = client.subscribe(subscribe_packet=mqtt5.SubscribePacket(
        #     subscriptions=[mqtt5.Subscription(
        #         topic_filter=message_topic,
        #         qos=mqtt5.QoS.AT_LEAST_ONCE)]
        # ))
        # suback = subscribe_future.result(TIMEOUT)
        # print("Suback received with reason code:{}\n".format(suback.reason_codes))

        self.pose_subscription = self.create_subscription(
            String,
            '/submarines/pose',
            self._on_pose,
            10
        )

        self.tick = self.create_timer(5, self._tick)
    
    def _on_pose(self, msg: PoseStamped):
        """Store own position."""
        self.get_logger().info(f"Pose received: {msg.data}")
        if not self.pose_buffer:
            self.pose_buffer.append(msg.data)
        self.pose_buffer[0] = msg.data
    
    def _tick(self):
        self.get_logger().info(f'{len(self.pose_buffer)}')
        if self.pose_buffer:
            if not self.connected:
                self.client.start()

            # We await the `on_lifecycle_connection_success` callback to be invoked.
            if not self.connection_success_event.wait(10):
                raise TimeoutError("Connection timeout")
        
            pose = self.pose_buffer.pop()
            # message = f"Pose: Position({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}) Orientation({pose.pose.orientation.x}, {pose.pose.orientation.y}, {pose.pose.orientation.z}, {pose.pose.orientation.w})"
            message = pose
            message_topic = "Nikhil-LAPTOP/test"
            self.get_logger().info(f"Publishing message to topic '{message_topic}': {message}")
            publish_future = self.client.publish(mqtt5.PublishPacket(
                topic=message_topic,
                payload=message,
                qos=mqtt5.QoS.AT_LEAST_ONCE
            ))
            publish_completion_data = publish_future.result(10)
            self.get_logger().info(f"PubAck received with {repr(publish_completion_data.puback.reason_code)}")        

            if len(self.pose_buffer) == 0:
                self.client.stop()

    # Callback when any publish is received
    def on_publish_received(self, publish_packet_data):
        publish_packet = publish_packet_data.publish_packet
        print("==== Received message from topic '{}': {} ====\n".format(
            publish_packet.topic, publish_packet.payload.decode('utf-8')))


    # Callback for the lifecycle event Stopped
    def on_lifecycle_stopped(self, lifecycle_stopped_data: mqtt5.LifecycleStoppedData):
        print("Lifecycle Stopped\n")
        self.stopped_event.set()
        self.connected = False


    # Callback for lifecycle event Attempting Connect
    def on_lifecycle_attempting_connect(self, lifecycle_attempting_connect_data: mqtt5.LifecycleAttemptingConnectData):
        print("Lifecycle Connection Attempt\nConnecting to endpoint")
        self.connected = False


    # Callback for the lifecycle event Connection Success
    def on_lifecycle_connection_success(self, lifecycle_connect_success_data: mqtt5.LifecycleConnectSuccessData):
        connack_packet = lifecycle_connect_success_data.connack_packet
        print("Lifecycle Connection Success with reason code:{}\n".format(
            repr(connack_packet.reason_code)))
        self.connection_success_event.set()
        self.connected = True


    # Callback for the lifecycle event Connection Failure
    def on_lifecycle_connection_failure(self, lifecycle_connection_failure: mqtt5.LifecycleConnectFailureData):
        print("Lifecycle Connection Failure with exception:{}".format(
            lifecycle_connection_failure.exception))
        self.connected = False


    # Callback for the lifecycle event Disconnection
    def on_lifecycle_disconnection(self, lifecycle_disconnect_data: mqtt5.LifecycleDisconnectData):
        print("Lifecycle Disconnected with reason code:{}".format(
            lifecycle_disconnect_data.disconnect_packet.reason_code if lifecycle_disconnect_data.disconnect_packet else "None"))
        self.connected = False

def main(args=None):
    rclpy.init(args=args)
    node = MQTT5Dump()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
