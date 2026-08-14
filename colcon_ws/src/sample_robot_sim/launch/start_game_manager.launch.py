"""試合管理ノードだけを立ち上げる。

bring_up_core_stage.launch.py にも game_manager は入っていますが、試合時間や
個人戦/チーム戦を変えて別途動かしたいときにこちらを使います
(bring_up 側の game_manager は落としてから使ってください)。

Isaac 版との違いは game_manager の中身だけです。リセット要求は共有メモリ
'isaac_sim_reset' ではなく simulation_interfaces の reset_simulation
(SCOPE_STATE) で送ります。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    initial_time = LaunchConfiguration('initial_time')
    game_kind = LaunchConfiguration('game_kind')

    declare_args = [
        DeclareLaunchArgument('initial_time', default_value='20',
                              description='試合時間 [s]'),
        DeclareLaunchArgument('game_kind', default_value='0',
                              description='0: 個人戦, 1: チーム戦 (奇数番=Blue / 偶数番=Red)'),
    ]

    game_manager = Node(
        package='game_manager',
        name='game_manager_node',
        executable='manager_node',
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
        parameters=[{
            # launch 引数は文字列で来るので、宣言側 (integer) に合わせて型を明示する。
            # そのまま渡すと declare_parameter の型と食い違って落ちる。
            'initial_time': ParameterValue(initial_time, value_type=int),
            'game_kind': ParameterValue(game_kind, value_type=int),
        }],
        output='screen',
    )

    return LaunchDescription(declare_args + [game_manager])
