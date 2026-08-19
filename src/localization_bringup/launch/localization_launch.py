import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    loc_share = get_package_share_directory('localization_bringup')
    sim_share = get_package_share_directory('f1tenth_gym_ros')

    # --- Launch arguments ---
    # use_sim_time: gym_bridge NO publica /clock → dejar en false (reloj de pared).
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map_yaml')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    scan_rate_hz = LaunchConfiguration('scan_rate_hz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar reloj de simulación (/clock). gym_bridge no lo publica → false.'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(loc_share, 'config', 'amcl_params.yaml'),
        description='YAML de parámetros de amcl + map_server + lifecycle.'
    )
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        # Default = mapa GROUND-TRUTH (SaoPaulo_map): es el mapa contra el que
        # se validó el perfil racing (error de pose 0.11 m) y el que usan TODOS
        # los runbooks de PP/MPC — antes el default apuntaba a slam_saopaulo.yaml
        # (mapa SLAM v1) que en la práctica nunca se usaba ("default engañoso").
        # Alternativa auto-consistente (pipeline SLAM→localización real, sin GT):
        # maps/slam_saopaulo_v3.yaml (error de pose 0.36 m, ver slam_bringup).
        default_value='/home/carolina/roboracer-f1tenth/maps/SaoPaulo_map.yaml',
        description='Mapa contra el que se localiza (default: ground-truth '
                    'SaoPaulo_map, el validado con el perfil racing).'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(loc_share, 'config', 'localization.rviz'),
        description='Ruta al .rviz para ver la nube de partículas.'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Lanzar RViz junto con el sim y AMCL.'
    )
    declare_scan_rate_hz = DeclareLaunchArgument(
        'scan_rate_hz',
        # 40 Hz confirmado como el valor correcto (medido 7 jul): a ss=0.8
        # (v 6.4 m/s) subir a 80 Hz NO reduce el error de pose (media 0.096 vs
        # 0.084 m, p95 0.21 en ambos = idéntico dentro del ruido) y una de 2
        # corridas a 80 Hz DIVERGIÓ (pose a 96 m estando el coche a 6 m/s). No
        # hay ganancia y sí riesgo → 40 Hz. Ver README §"Test de scan_rate".
        default_value='40.0',
        description='Tasa máxima (Hz) del scan reentregado a AMCL por scan_throttle. '
                    '40 medido como óptimo; subirla no mejora la pose y acerca '
                    'al gotcha de saturación a 250 Hz.'
    )

    sim_config = os.path.join(sim_share, 'config', 'sim.yaml')
    ego_xacro = os.path.join(sim_share, 'launch', 'ego_racecar.xacro')

    # --- Nodos del simulador ---
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', ego_xacro]),
                     'use_sim_time': use_sim_time}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    # gym_bridge publica /scan a 250 Hz (paso de física del sim) y satura el
    # message filter de AMCL → diverge. Este nodo reentrega /scan a ~40 Hz en
    # /scan_amcl (lo que AMCL consume). Ver README §Gotcha del scan a 250 Hz.
    scan_throttle_node = Node(
        package='localization_bringup',
        executable='scan_throttle',
        name='scan_throttle',
        # Si el nodo muere (se ha visto un segfault esporádico de rclpy/DDS),
        # AMCL se queda sin scans y el TF map→odom se congela → el control
        # pierde la pose. Relanzarlo automáticamente.
        respawn=True,
        respawn_delay=1.0,
        parameters=[{'input_topic': '/scan'},
                    {'output_topic': '/scan_amcl'},
                    # ParameterValue con value_type=float: mismo gotcha que
                    # speed_scale/lookahead_k en pure_pursuit_launch (sin esto
                    # el launch falla con type error al pasar el arg).
                    {'rate_hz': ParameterValue(scan_rate_hz, value_type=float)},
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- Localización (nav2) ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[params_file,
                    {'yaml_filename': map_yaml},
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}],
        output='screen'
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_scan_rate_hz)

    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(scan_throttle_node)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)

    return ld
