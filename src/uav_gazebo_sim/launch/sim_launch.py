import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_gazebo_sim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    models_dir = os.path.join(pkg_dir, 'models')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[models_dir, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    disable_online_models = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    # Cargar la ruta del archivo uav_world.world
    world_file = os.path.join(pkg_dir, 'worlds', 'uav_world.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    shahed_sdf = os.path.join(models_dir, 'shahed136', 'model.sdf')
    sting_sdf = os.path.join(models_dir, 'sting2', 'model.sdf')

    spawn_shahed = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'shahed136', '-file', shahed_sdf, '-x', '20.0', '-y', '0.0', '-z', '25.0'],
                output='screen'
            )
        ]
    )

    spawn_sting = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'sting2', '-file', sting_sdf, '-x', '0.0', '-y', '0.0', '-z', '0.1'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        disable_online_models,
        gazebo_launch,
        spawn_shahed,
        spawn_sting
    ])