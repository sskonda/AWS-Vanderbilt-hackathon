from launch import LaunchDescription
from launch_ros.actions import Node

def make_uuv(sub_id: str, id_no: int) -> Node:
    return Node(
        package='uuv',                
        executable='uuv_planner',      
        name=f'{sub_id}_planner',
        namespace=sub_id,            
        output='screen',
        parameters=[
            {'sub_id': sub_id},                     
            {'Identity': int(id_no)},              
            {'team_ids': ['subMID','subA','subB']},
            {'leader_id': 'subMID'},
            {'state_dwell_s': 20.0},
            {'publish_current_pose': False},
        ],
    )

def generate_launch_description():
    sub_mid = make_uuv('subMID', 0)
    sub_a   = make_uuv('subA',   1)
    sub_b   = make_uuv('subB',   2)

    # MQTT bridge
    mqtt_bridge = Node(
        package='uuv_py',
        executable='mqtt_upload',     
        name='mqtt_upload',
        output='screen',
        parameters=[
            {'endpoint': 'a2z539demks74-ats.iot.us-east-1.amazonaws.com'},
            {'cert_filepath': '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/subA.cert.pem'},
            {'pri_key_filepath': '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/subA.private.key'},
            {'ca_filepath':   '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/aws_iot_core/root-CA.crt'},
            {'client_id': 'subA'},              
            # Downlink → ROS plans
            {'mqtt_topic_sub': 'uuv/relay/downlink'},
            {'frame_id': 'relay'},
            {'qos': 1},
            # Uplink (if your bridge supports it)
            {'ros_input_topic': '/relay/snapshot_json'},
            {'mqtt_topic_pub': 'uuv/relay/snapshot'},
        ],
    )

    return LaunchDescription([
        sub_mid,
        sub_a,
        sub_b,
        mqtt_bridge
    ])
