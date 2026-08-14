"""シミュレータを立ち上げ、CoRE ステージをワールドとして読み込む。

Isaac 版の bring_up_core_stage.launch.py に対応します。Isaac は launcher ノードが
Isaac Sim 本体を起動して USD ステージを開いていましたが、Unity 版は

  1. Unity のシミュレータ実行ファイル (別プロセス)
  2. ROS-TCP-Endpoint (シミュレータと ROS 2 をつなぐ)
  3. load_world サービス呼び出し (SDF のステージを読ませる)

の 3 段構えになります。この launch はその 3 つと、試合管理 (game_manager)・
ブラウザ操縦用の rosbridge をまとめて起動します。

launch 引数:
  simulator_path   シミュレータ実行ファイル。既定は docker イメージ内の展開先。
  launch_simulator false にするとシミュレータは起動しない (すでに手で起動して
                   いる場合や、別 PC で動かしている場合)。
  launch_endpoint  false にすると ROS-TCP-Endpoint を起動しない。
  world_path       読み込むワールド。既定は core_stage_description の SDF。
  startup_delay    シミュレータ起動から load_world を試すまでの待ち [s]。
                   load_world ノード自体もサービスが出るまで待つので、
                   足りなくても即失敗にはならない。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_SIMULATOR_PATH = os.path.join(
    os.path.expanduser('~'),
    'Unity_ROS2_Robot_Simulator_v1.0.2_Linux_amd64',
    'Unity_ROS2_Robot_Simulator.x86_64')


def generate_launch_description():
    simulator_path = LaunchConfiguration('simulator_path')
    launch_simulator = LaunchConfiguration('launch_simulator')
    launch_endpoint = LaunchConfiguration('launch_endpoint')
    world_path = LaunchConfiguration('world_path')
    startup_delay = LaunchConfiguration('startup_delay')

    default_world_path = os.path.join(
        get_package_share_directory('core_stage_description'),
        'worlds', 'core_stage.world')

    declare_args = [
        DeclareLaunchArgument('simulator_path', default_value=DEFAULT_SIMULATOR_PATH),
        DeclareLaunchArgument('launch_simulator', default_value='true'),
        DeclareLaunchArgument('launch_endpoint', default_value='true'),
        DeclareLaunchArgument('world_path', default_value=default_world_path),
        DeclareLaunchArgument('startup_delay', default_value='15.0'),
    ]

    simulator = ExecuteProcess(
        cmd=[simulator_path],
        output='screen',
        condition=IfCondition(launch_simulator),
    )

    # ROS_IP=0.0.0.0 で待ち受けるのは、Unity 側を同一 PC 以外
    # (Windows ホスト等) で動かす場合にも届くようにするため。
    tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[{'ROS_IP': '0.0.0.0'}],
        output='screen',
        condition=IfCondition(launch_endpoint),
    )

    # シミュレータと endpoint が繋がるまでサービスが出ないので、少し遅らせてから
    # 呼ぶ。ノード側でもサービス待ちをするので、この遅延は「無駄なログを出さない」
    # ためのもの。
    load_world = TimerAction(
        period=startup_delay,
        actions=[
            Node(
                package='core_sim_utils',
                executable='load_world',
                name='load_core_stage',
                parameters=[{'world_path': world_path}],
                output='screen',
            ),
        ],
    )

    game_manager = Node(
        package='game_manager',
        executable='manager_node',
        name='game_manager_node',
        parameters=[{'initial_time': 120}],
        remappings=[
            ('robot1_hp', '/sample_robot_1/robot_hp'),
            ('robot2_hp', '/sample_robot_2/robot_hp'),
            ('robot3_hp', '/sample_robot_3/robot_hp'),
            ('robot4_hp', '/sample_robot_4/robot_hp'),
            ('robot5_hp', '/sample_robot_5/robot_hp'),
            ('robot6_hp', '/sample_robot_6/robot_hp'),
            ('robot7_hp', '/sample_robot_7/robot_hp'),
            ('robot8_hp', '/sample_robot_8/robot_hp'),
        ],
        output='screen',
    )

    # ブラウザ操縦 (tools/robot_N_control.html) の通信口。ポートは既定の 9090。
    # delay_between_messages は Jazzy では float でないと型エラーになる。
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'delay_between_messages': 0.0,
        }],
    )

    return LaunchDescription(declare_args + [
        simulator,
        tcp_endpoint,
        load_world,
        game_manager,
        rosbridge_websocket,
    ])
