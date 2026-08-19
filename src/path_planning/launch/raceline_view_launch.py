import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    pp_share = get_package_share_directory('path_planning')

    map_yaml = LaunchConfiguration('map_yaml')
    csv_path = LaunchConfiguration('csv_path')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        # Default = par operativo (mapa GT + su raceline), no el SLAM v1 que ya
        # no se usa. Para ver la raceline del mapa propio: pasar
        # map_yaml:=.../slam_saopaulo_v3.yaml csv_path:=.../slam_saopaulo_v3.csv
        default_value='/home/carolina/roboracer-f1tenth/maps/SaoPaulo_map.yaml',
        description='Mapa de fondo para visualizar la raceline (default: GT).'
    )
    declare_csv_path = DeclareLaunchArgument(
        'csv_path',
        default_value=os.path.join(pp_share, 'racelines', 'saopaulo_gt.csv'),
        description='CSV de la raceline a publicar (default: saopaulo_gt).'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pp_share, 'config', 'raceline_view.rviz'),
        description='Config de RViz.'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Lanzar RViz.'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml},
                    {'use_sim_time': False}],
        output='screen'
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_raceline',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
        output='screen'
    )
    raceline_node = Node(
        package='path_planning',
        executable='raceline_publisher',
        name='raceline_publisher',
        parameters=[{'csv_path': csv_path},
                    {'frame_id': 'map'}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    ld.add_action(declare_map_yaml)
    ld.add_action(declare_csv_path)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_use_rviz)

    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(raceline_node)
    ld.add_action(rviz_node)

    return ld
