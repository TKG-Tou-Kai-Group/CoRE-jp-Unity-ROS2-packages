"""シミュレータを立ち上げ、CoRE ステージをワールドとして読み込む。

Isaac 版の bring_up_core_stage.launch.py に対応します。Isaac は launcher ノードが
Isaac Sim 本体を起動して USD ステージを開いていましたが、Unity 版は

  1. Unity のシミュレータ実行ファイル (別プロセス)
  2. ROS-TCP-Endpoint (シミュレータと ROS 2 をつなぐ)
  3. load_world サービス呼び出し (SDF のステージを読ませる)

の 3 段構えになります。この launch はその 3 つと、フライングディスクの供給
(flying_disc_feeder)・映像配信 (operator_stream)・試合管理 (game_manager)・
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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

DEFAULT_SIMULATOR_PATH = os.path.join(
    os.path.expanduser('~'),
    'Unity_ROS2_Robot_Simulator_v1.2.0_Linux_amd64',
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
        DeclareLaunchArgument('video_input', default_value='jpeg'),
    ]

    # シミュレータの起動時設定 (物理レートなど)。既定は 50 Hz だが、オムニホイールの
    # フリーローラが 50 Hz では空転するので 200 Hz にしている (config/ の json 参照)。
    sim_resources = os.path.join(
        get_package_share_directory('sample_robot_sim'),
        'config', 'simulation_resources.json')

    simulator = ExecuteProcess(
        cmd=[simulator_path],
        output='screen',
        additional_env={'SIMULATION_RESOURCES_CONFIG': sim_resources},
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

    # フライングディスクの供給。以前は 1 台につき 20 枚を起動時に積み上げて
    # いたが、接触したまま積み重なった剛体はソルバのコストが高く、2 台ぶんの
    # 40 枚で物理が実時間に収まらなくなっていた (カメラ映像が 2.8 FPS まで低下)。
    # 装填口には常に 1 枚だけ置き、撃つたびに次の 1 枚を作る。
    # 場に残るディスクは全機体あわせて max_alive 枚に保つ。
    flying_disc_urdf_path = os.path.join('/tmp', 'flying_disc.urdf')
    flying_disc_doc = xacro.process_file(os.path.join(
        get_package_share_directory('flying_disc_description'),
        'urdf', 'flying_disc.urdf.xacro'))
    with open(flying_disc_urdf_path, 'w') as f:
        f.write(flying_disc_doc.toprettyxml(indent='  '))

    flying_disc_feeder = Node(
        package='core_sim_utils',
        executable='flying_disc_feeder',
        name='flying_disc_feeder',
        parameters=[{
            'urdf_path': flying_disc_urdf_path,
            'max_alive': 24,
        }],
        output='screen',
    )

    # 操縦画面の映像を H.264 で配信する。ブラウザは ws://<host>:9091/robot<N>
    # を開く (tools/robot_<N>_control.html)。
    #
    # rosbridge 経由の JPEG (base64) だと 1 系統 2.8 Mbps、8 系統で 23 Mbps に
    # なる。H.264 なら同じ画をおよそ 1/3 で送れる。視聴者が居ない系統は
    # エンコードしないので、誰も見ていない機体のぶんは CPU を使わない。
    # video_input:=raw にすると、JPEG を経由せず生画像を受け取る。
    # 両端 (カメラ配信の符号化と、ここでの復号) を省ける。
    # ロボット側も camera_output_format:=raw で起動すること。
    operator_stream = Node(
        package='core_sim_utils',
        executable='operator_stream',
        name='operator_stream',
        output='screen',
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration('video_input'), "' == 'raw'"])),
    )
    operator_stream_raw = Node(
        package='core_sim_utils',
        executable='operator_stream',
        name='operator_stream',
        arguments=['--raw'],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('video_input'), "' == 'raw'"])),
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
        flying_disc_feeder,
        operator_stream,
        operator_stream_raw,
        game_manager,
        rosbridge_websocket,
    ])
