# bridge_mqtt5.py
import os
import json
import math
import socket
import uuid
import threading
import rclpy
from rclpy.node import Node

from awsiot import mqtt5_client_builder
from awscrt import mqtt5

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String as StringMsg
from custom_msg.msg import Plan

def to_stamp(unix_time: float) -> TimeMsg:
    st = TimeMsg()
    if unix_time is None or unix_time < 0:
        st.sec = 0; st.nanosec = 0; return st
    sec = int(math.floor(float(unix_time)))
    nsec = int(round((float(unix_time) - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1; nsec -= 1_000_000_000
    st.sec = sec; st.nanosec = nsec
    return st

def plan_from_array(points, frame_id: str = "relay") -> Plan:
    out = Plan()
    out.header.frame_id = frame_id
    path = Path(); path.header.frame_id = frame_id
    poses = []
    for p in points or []:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = to_stamp(p.get("t", 0.0))
        ps.pose.position.x = float(p.get("x", 0.0))
        ps.pose.position.y = float(p.get("y", 0.0))
        ps.pose.position.z = float(p.get("z", 0.0))
        ps.pose.orientation.w = 1.0
        poses.append(ps)
    path.poses = poses
    out.paths.append(path)
    return out

class mqtt_upload(Node):
    def __init__(self):
        super().__init__('mqtt_upload')

        # ---------------- Parameters ----------------
        self.declare_parameters('', [
            ('endpoint',        'a2z539demks74-ats.iot.us-east-1.amazonaws.com'),
            ('cert_filepath',   '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/subA.cert.pem'),
            ('pri_key_filepath','/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/subA.private.key'),
            ('ca_filepath',     '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/root-CA.crt'),
            ('client_id',       f'subA'),
            ('frame_id',        'relay'),
            # AWS topics
            ('mqtt2ros', []),
            ('ros2mqtt', []),
            ('mqtt_topic_sub',  'uuv/relay/downlink'),  # AWS -> ROS
            ('mqtt_topic_pub',  'uuv/relay/snapshot'),  # ROS -> AWS
            ('qos',             1),
            # ROS topic that carries JSON string to send up to AWS
            ('ros_input_topic', '/relay/snapshot_json'),
        ])

        # Validate files early
        for p in ('cert_filepath', 'pri_key_filepath', 'ca_filepath'):
            path = self.get_parameter(p).value
            if not os.path.isfile(path):
                self.get_logger().error(f"{p} not found: {path}")
                raise FileNotFoundError(path)

        # --------------- ROS pubs/subs ---------------
        # ROS -> AWS (upload)
        self._out_buffer = []  # bytes ready to send to AWS
        self.sub_snapshot = self.create_subscription(
            StringMsg,
            self.get_parameter('ros_input_topic').value,
            self._on_snapshot_json,
            10
        )

        # AWS -> ROS (downlink)
        self.pub_sub1 = self.create_publisher(Plan, '/plans/sub1', 10)
        self.pub_sub2 = self.create_publisher(Plan, '/plans/sub2', 10)
        self.pub_mid  = self.create_publisher(Plan, '/plans/mid',  10)
        self._rx = []  # bytes received from AWS

        # --------------- MQTT5 client ----------------
        qos = mqtt5.QoS.AT_LEAST_ONCE if int(self.get_parameter('qos').value) == 1 else mqtt5.QoS.AT_MOST_ONCE
        self._want_subscribe = False
        self._sub_future = None
        self.connected = False
        self.subscribed = False
        self.stopped_event = threading.Event()

        cid = self.get_parameter('client_id').value  # MUST be unique per running client
        self.client = mqtt5_client_builder.mtls_from_path(
            endpoint=self.get_parameter('endpoint').value,
            cert_filepath=self.get_parameter('cert_filepath').value,
            pri_key_filepath=self.get_parameter('pri_key_filepath').value,
            ca_filepath=self.get_parameter('ca_filepath').value,
            client_id=cid,
            on_publish_received=self._on_mqtt_publish,
            on_lifecycle_stopped=self._on_mqtt_stopped,
            on_lifecycle_attempting_connect=self._on_mqtt_attempting_connect,
            on_lifecycle_connection_success=self._on_mqtt_connected,
            on_lifecycle_connection_failure=self._on_mqtt_conn_fail,
            on_lifecycle_disconnection=self._on_mqtt_disconnected,
        )
        self.client.start()
        self.get_logger().info(f"Using MQTT client_id: {cid}")

        # --------------- Timers ----------------------
        # one timer handles subscribe attempts, RX drain, and TX drain
        self.create_timer(0.1, self._tick)

        self._aws_qos = qos
        self._aws_topic_sub = self.get_parameter('mqtt_topic_sub').value
        self._aws_topic_pub = self.get_parameter('mqtt_topic_pub').value

    # ===== ROS handlers =====
    def _on_snapshot_json(self, msg: StringMsg):
        # Buffer JSON string to publish to AWS
        data = (msg.data or '').encode('utf-8', errors='ignore')
        if not data:
            return
        self._out_buffer.append(data)

    # ===== MQTT lifecycle =====
    def _on_mqtt_attempting_connect(self, _):
        self.get_logger().info("MQTT attempting connect…")
        self.connected = False
        self.subscribed = False
        self._want_subscribe = False
        self._sub_future = None

    def _on_mqtt_connected(self, data):
        self.connected = True
        rc = data.connack_packet.reason_code
        self.get_logger().info(f"MQTT connected ({rc})")
        self._want_subscribe = True  # do subscribe in timer (non-blocking)

    def _on_mqtt_conn_fail(self, e):
        self.connected = False
        self.subscribed = False
        self._want_subscribe = False
        self._sub_future = None
        self.get_logger().error(f"MQTT connection failure: {e.exception}")

    def _on_mqtt_disconnected(self, data):
        reason = data.disconnect_packet.reason_code if data.disconnect_packet else "None"
        self.get_logger().warn(f"MQTT disconnected (reason={reason})")
        self.connected = False
        self.subscribed = False
        self._want_subscribe = False
        self._sub_future = None

    def _on_mqtt_stopped(self, _):
        self.connected = False
        self.subscribed = False
        self._want_subscribe = False
        self._sub_future = None
        self.stopped_event.set()
        self.get_logger().warn("MQTT stopped")

    # ===== MQTT RX =====
    def _on_mqtt_publish(self, publish_packet_data):
        pkt = publish_packet_data.publish_packet
        payload = pkt.payload or b''
        self.get_logger().info(f"AWS RX {len(payload)} bytes on '{pkt.topic}'")
        self._rx.append(payload)

    # ===== Main tick: subscribe + drain RX/TX =====
    def _tick(self):
        # subscribe (deferred, non-blocking)
        if self.connected and not self.subscribed and self._want_subscribe and self._sub_future is None:
            try:
                self.get_logger().info(f"Subscribing to '{self._aws_topic_sub}' (QoS={1 if self._aws_qos==mqtt5.QoS.AT_LEAST_ONCE else 0})")
                self._sub_future = self.client.subscribe(mqtt5.SubscribePacket(
                    subscriptions=[mqtt5.Subscription(topic_filter=self._aws_topic_sub, qos=self._aws_qos)]
                ))
                def _on_done(fut):
                    try:
                        suback = fut.result()
                        self.get_logger().info(f"SubAck: {repr(suback.reason_codes)}")
                        self.subscribed = True
                    except Exception as e:
                        self.get_logger().error(f"Subscribe failed: {e}")
                    finally:
                        self._sub_future = None
                        self._want_subscribe = False
                self._sub_future.add_done_callback(_on_done)
            except Exception as e:
                self.get_logger().error(f"Subscribe attempt error: {e}")
                self._sub_future = None
                self._want_subscribe = True  # retry next tick

        # Drain RX → ROS
        if self._rx:
            raw = self._rx.pop(0)
            self._handle_downlink_payload(raw)

        # Drain TX (ROS → AWS)
        if self._out_buffer and self.connected:
            payload = self._out_buffer.pop(0)
            try:
                self.get_logger().info(f"AWS TX {len(payload)} bytes to '{self._aws_topic_pub}'")
                fut = self.client.publish(mqtt5.PublishPacket(
                    topic=self._aws_topic_pub,
                    payload=payload,
                    qos=self._aws_qos
                ))
                # fire-and-forget (no blocking); optionally fut.add_done_callback(...)
            except Exception as e:
                self.get_logger().error(f"Publish failed: {e}")

    # ===== Downlink payload handling (AWS -> ROS) =====
    def _handle_downlink_payload(self, raw: bytes):
        try:
            obj = json.loads(raw.decode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"JSON decode error: {e}")
            return

        frame_id = self.get_parameter('frame_id').value
        now = self.get_clock().now().to_msg()

        # Shape A: {"plans":{"mid":[..], "sub1":[..], "sub2":[..]}}
        if isinstance(obj, dict) and isinstance(obj.get('plans'), dict):
            plans = obj['plans']
            for who, pub in (('mid', self.pub_mid), ('sub1', self.pub_sub1), ('sub2', self.pub_sub2)):
                arr = plans.get(who)
                if arr:
                    msg = plan_from_array(arr, frame_id=frame_id)
                    msg.header.stamp = now
                    msg.paths[0].header.stamp = now
                    pub.publish(msg)
                    self.get_logger().info(f"Published /plans/{who} with {len(msg.paths[0].poses)} poses")
            return

        # Shape B: {"who":"sub1","plan":[..]}
        if isinstance(obj, dict) and obj.get('who') and isinstance(obj.get('plan'), list):
            who = str(obj['who']).lower()
            pub = self.pub_sub1 if who == 'sub1' else self.pub_sub2 if who == 'sub2' else self.pub_mid
            msg = plan_from_array(obj['plan'], frame_id=frame_id)
            msg.header.stamp = now
            msg.paths[0].header.stamp = now
            pub.publish(msg)
            self.get_logger().info(f"Published /plans/{who} with {len(msg.paths[0].poses)} poses")
            return

        self.get_logger().warn("AWS payload did not match expected shapes; ignored.")

def main(args=None):
    rclpy.init(args=args)
    node = mqtt_upload()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
