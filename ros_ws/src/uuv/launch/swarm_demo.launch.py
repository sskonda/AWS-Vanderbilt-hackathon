from launch import LaunchDescription
from launch_ros.actions import Node


def make_sub(name, role, phase):
    return Node(
        package='uuv',
        executable='uuv_planner',   
        name=f'{name}_node',
        output='screen',
        parameters=[
            {'sub_id': name},
            {'role': role},
            {'team_ids': ['subA', 'subB', 'subC']},
            {'start_phase': phase},        # route offsets 
            {'phase_rate': 1.0},          
            {'tick_dt_s': 0.5},          
            {'comm_period_s': 25.0},       # when does mid their comm
            {'publish_current_pose': False} # publish only the target pose
        ],
    )


def generate_launch_description():
    subA = make_sub('subA', 'PATROL', 0.00)
    subB = make_sub('subB', 'PATROL', 0.33)
    subC = make_sub('subC', 'MID_TIER', 0.66)

    # Event simulator (publishes to /events/event_1)
    sim = Node(
        package='uuv_sim_py',
        executable='sim',
        name='swarm_sim',
        output='screen',
        parameters=[
            {'proximity_period_s': 20.0},  # trigger simulated proximity hits
            {'foreign_period_s':   55.0},  # trigger foreign UUV detections
            {'swarm_period_s':     50.0},  # trigger swarm data requests , nearby comes up out of cycle 
            {'pipeline_period_s':  65.0},  # trigger pipeline break events
        ],
    )

    return LaunchDescription([subA, subB, subC, sim])
