from launch import LaunchDescription
from launch_ros.actions import Node

def make_sub(name, role):
    return Node(
        package='uuv',
        executable='uuv_planner',
        name=f'{name}_node',
        output='screen',
        parameters=[
            {'sub_id': name},
            {'role': role},
            {'team_ids': ['subA','subB','subC']},
        ],
    )

def generate_launch_description(): # makes all the subs and stagger them 
    subA = make_sub('subA', 'PATROL')
    subB = make_sub('subB', 'PATROL')
    subC = make_sub('subC', 'MID_TIER')

    sim = Node(
        package='uuv_sim_py',
        executable='sim',
        name='swarm_sim',
        output='screen',
        parameters=[
            {'proximity_period_s': 20.0},
            {'foreign_period_s':   35.0},
            {'swarm_period_s':     50.0},
            {'pipeline_period_s':  65.0},
        ],
    )

    return LaunchDescription([subA, subB, subC, sim])
