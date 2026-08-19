import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    pp_share = get_package_share_directory('pure_pursuit')
    loc_share = get_package_share_directory('localization_bringup')
    plan_share = get_package_share_directory('path_planning')

    use_rviz = LaunchConfiguration('use_rviz')
    pose_source = LaunchConfiguration('pose_source')
    csv_path = LaunchConfiguration('csv_path')
    map_yaml = LaunchConfiguration('map_yaml')
    speed_scale = LaunchConfiguration('speed_scale')
    lookahead_k = LaunchConfiguration('lookahead_k')
    start_controller = LaunchConfiguration('start_controller')

    # Default = raceline operativa (mapa GT, comparte frame con la física del
    # sim). Antes apuntaba a slam_saopaulo.csv (mapa SLAM v1, desfasado 2-3 m
    # y pre-círculo-de-fricción) — justo la raceline que causaba los choques
    # originales de PP. Corregido en la revisión del 7 jul (ver
    # path_planning §"Limpieza de racelines").
    default_csv = os.path.join(plan_share, 'racelines', 'saopaulo_gt.csv')

    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    declare_pose_source = DeclareLaunchArgument(
        'pose_source', default_value='tf',
        description="tf = pose de AMCL (valida la cadena); odom = ground-truth del sim")
    declare_speed_scale = DeclareLaunchArgument(
        'speed_scale', default_value='1.0',
        description='Factor sobre la v de la raceline (bajar para pruebas/tuning).')
    declare_lookahead_k = DeclareLaunchArgument(
        'lookahead_k', default_value='0.4',
        description='Lookahead por unidad de velocidad: L = clip(k*v, min, max).')
    # RViz va retrasado 6 s (ver TimerAction abajo) y en máquinas lentas puede
    # tardar más — el controlador arranca YA y el auto ya está en marcha (o
    # chocado) cuando por fin se ve algo. start_controller:=false levanta
    # sim+AMCL+raceline+RViz SIN el auto moviéndose; lanzar el controlador
    # aparte (ros2 run) una vez RViz ya está abierto y listo (ver README
    # §"Corrida directa").
    declare_start_controller = DeclareLaunchArgument(
        'start_controller', default_value='true',
        description='false = NO arranca pure_pursuit_node (para levantar '
                    'primero sim+RViz y lanzar el controlador aparte).')
    declare_csv_path = DeclareLaunchArgument(
        'csv_path', default_value=default_csv,
        description='Raceline CSV a seguir (y a publicar en /raceline).')
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        # Default = mapa GT (operativo, comparte frame con la física del sim),
        # igual que localization_launch. Antes: slam_saopaulo.yaml (mapa SLAM
        # v1, ya no usado).
        default_value='/home/carolina/roboracer-f1tenth/maps/SaoPaulo_map.yaml',
        description='Mapa para localización y visualización.')
    # NOTA: este arg se llamaba 'params_file' — CHOCABA con el 'params_file'
    # que declara localization_launch.py (para amcl_params.yaml). Con
    # forwarding=True (ver abajo) el valor de este launch (pure_pursuit_
    # params.yaml) se filtraba DENTRO del include y pisaba el default de
    # amcl_params.yaml → AMCL arrancaba con parámetros de fábrica (base_frame
    # 'base_footprint', scan_topic '/scan' sin throttle) en vez del perfil
    # racing, TF map→odom desconectada de ego_racecar/* → "Fixed Frame"/
    # "RobotModel" con error en RViz. Renombrado para que la colisión sea
    # imposible aunque forwarding vuelva a activarse por error.
    declare_params_file = DeclareLaunchArgument(
        'pp_params_file',
        default_value=os.path.join(pp_share, 'config', 'pure_pursuit_params.yaml'),
        description='Parámetros del pure_pursuit_node.')
    params_file = LaunchConfiguration('pp_params_file')

    # --- Localización completa (sim + AMCL + map_server + scan_throttle), sin RViz ---
    # GroupAction(scoped=True): AÍSLA el include. Sin esto, el
    # `use_rviz: 'false'` que le pasamos a localization_launch se filtra al
    # scope del padre (SetLaunchConfiguration no aislado) y sobrescribe el
    # use_rviz del padre → el rviz_node de ESTE launch, que se evalúa después,
    # veía 'false' y NUNCA arrancaba (gotcha #1, resuelto 7 jul). Con el grupo
    # scoped, ese cambio queda dentro del include y no toca al padre.
    # forwarding=True (default) SÍ hace falta: la sustitución `map_yaml` de
    # abajo se resuelve DENTRO del scope del grupo, y necesita ver el
    # LaunchConfiguration('map_yaml') del padre para poder sustituirlo — sin
    # forwarding, falla con "launch configuration 'map_yaml' does not exist".
    # El riesgo real de forwarding=True era la colisión de NOMBRES con
    # 'params_file' (ver arriba, ahora renombrado a 'pp_params_file') — con
    # nombres únicos, forwarding=True es seguro: solo 'use_rviz' y 'map_yaml'
    # se comparten con el include, y ambos están sobreescritos a propósito
    # por el `launch_arguments=` explícito de abajo.
    localization = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'localization_launch.py')),
            launch_arguments={'use_rviz': 'false', 'map_yaml': map_yaml}.items())
    ], scoped=True, forwarding=True)

    # --- Publicador de la raceline (para RViz y como referencia) ---
    raceline_publisher = Node(
        package='path_planning',
        executable='raceline_publisher',
        name='raceline_publisher',
        parameters=[{'csv_path': csv_path}, {'frame_id': 'map'}],
        output='screen')

    # --- Controlador (condicionado a start_controller) ---
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[params_file,
                    {'csv_path': csv_path},
                    {'pose_source': pose_source},
                    {'speed_scale': ParameterValue(speed_scale, value_type=float)},
                    {'lookahead_k': ParameterValue(lookahead_k, value_type=float)}],
        condition=IfCondition(start_controller),
        output='screen')

    # RViz retrasado 6 s (TimerAction): arrancado junto al resto del stack,
    # la init de OpenGL/OGRE de rviz2 competía por la GPU con gym_bridge+AMCL
    # y segfaultaba (-11) — standalone nunca falla. Dejarlo arrancar cuando el
    # stack ya está levantado lo estabiliza. (Segundo problema, encontrado al
    # arreglar el gotcha #1; el primero era la fuga de use_rviz del include.)
    rviz_node = TimerAction(period=6.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(pp_share, 'config', 'pure_pursuit.rviz')],
            condition=IfCondition(use_rviz))
    ])

    ld.add_action(declare_use_rviz)
    ld.add_action(declare_pose_source)
    ld.add_action(declare_csv_path)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_params_file)
    ld.add_action(declare_speed_scale)
    ld.add_action(declare_lookahead_k)
    ld.add_action(declare_start_controller)

    ld.add_action(localization)
    ld.add_action(raceline_publisher)
    ld.add_action(pure_pursuit_node)
    ld.add_action(rviz_node)

    return ld
