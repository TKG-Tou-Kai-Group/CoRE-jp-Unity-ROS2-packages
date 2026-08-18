"""sample_robot_5 を Unity のシミュレータへスポーンし、制御系を立ち上げる。

Isaac 版の sample_robot_5_spawn_for_core.launch.py に対応します。違いは 2 点です。

- スポーンは isaac_ros2_scripts/spawn_robot ではなく simulation_ros2_utils/spawn_entity
  (simulation_interfaces の spawn_entity サービス) 経由。
- フライングディスクは起動時に積まない。bring_up_core_stage が起動する
  core_sim_utils/flying_disc_feeder が、装填口に常に 1 枚だけ置き、
  撃つたびに次の 1 枚を作る。

走行の指令はシミュレータが /<エンティティ名>/cmd_vel を直接購読します。
オムニホイールを車体への直接加力 (body_twist_drive) へ置き換えたため、
cmd_vel を車輪速度へ変換する omni_wheel_controller と velocity_pub は
不要になりました。ros2_control が担うのは射出機構だけです。

センサのトピック名はシミュレータが /<エンティティ名>/<リンク名>/... で決めます。
Isaac のようにリンクの親子関係は入らないので、remap もそれに合わせてあります。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

ROBOT_NAME = 'sample_robot_5'
ROBOT_START_POSITION = [-4.5, -11.25, 0.1]
ROBOT_START_YAW = 1.57



def wrap_yaml_text(input_path: str, robot_name: str, output_path: str) -> None:
    """controller_manager の yaml をロボット名の名前空間で包み直す。

    ros2_control のノードを namespace 付きで起動するため、パラメータ側も
    <robot_name>: でインデントし直す必要がある。
    """
    with open(input_path, 'r') as fin:
        lines = fin.readlines()

    with open(output_path, 'w') as fout:
        fout.write(f"{robot_name}:\n")
        for line in lines:
            if line.strip() == "":
                fout.write("\n")
            else:
                fout.write(f"  {line}")


def generate_launch_description():
    sample_robot_description_path = get_package_share_directory('sample_robot_description')

    # --- ロボットの URDF を xacro から生成 -------------------------------
    sample_robot_xacro_file = os.path.join(
        sample_robot_description_path, 'robots', ROBOT_NAME + '.urdf.xacro')
    sample_robot_urdf_path = os.path.join('/tmp', ROBOT_NAME + '.urdf')
    sample_robot_doc = xacro.process_file(sample_robot_xacro_file, mappings={'use_sim': 'true'})
    sample_robot_desc = sample_robot_doc.toprettyxml(indent='  ')
    with open(sample_robot_urdf_path, 'w') as f:
        f.write(sample_robot_desc)

    params = {'robot_description': sample_robot_desc}


    rviz_config_file = os.path.join(
        sample_robot_description_path, 'config', 'sample_robot_description.rviz')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ROBOT_NAME,
        output='screen',
        parameters=[params],
    )

    # simulation_interfaces の spawn_entity を叩く。robot_name がそのまま
    # シミュレータ上のエンティティ名になり、センサのトピック接頭辞にもなる。
    spawn_robot = Node(
        package='simulation_ros2_utils',
        executable='spawn_entity',
        name='spawn_entity',
        # 名前空間は付けない。ノードは 'spawn_entity' を相対名で呼ぶので、
        # 名前空間を付けると /<robot>/spawn_entity を探しに行って永久に待つ。
        parameters=[{
            'urdf_path': sample_robot_urdf_path,
            'robot_name': ROBOT_NAME,
            'x': ROBOT_START_POSITION[0],
            'y': ROBOT_START_POSITION[1],
            'z': ROBOT_START_POSITION[2],
            'R': 0.0,
            'P': 0.0,
            'Y': ROBOT_START_YAW,
        }],
        output='screen',
    )


    robot_config_path = os.path.join(
        sample_robot_description_path, 'config', 'sample_robot_config.yaml')
    tmp_robot_controllers_path = os.path.join('/tmp', f'{ROBOT_NAME}_sim.yaml')
    wrap_yaml_text(robot_config_path, ROBOT_NAME, tmp_robot_controllers_path)

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=ROBOT_NAME,
        parameters=[params, tmp_robot_controllers_path],
        ros_arguments=[],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/' + ROBOT_NAME + '/controller_manager'],
        ros_arguments=[],
    )


    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller',
                   '--controller-manager', '/' + ROBOT_NAME + '/controller_manager'],
        ros_arguments=[],
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=ROBOT_NAME,
        output='log',
        arguments=['-d', rviz_config_file],
    )

    teleop_twist_joy = Node(
        package='teleop_twist_joy_for_sample_robot',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('sample_robot_description'),
                'config',
                'sample_robot_controller.config.yaml',
            ])
        ],
        namespace=ROBOT_NAME,
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('commands', 'velocity_controller/commands'),
        ],
    )

    # シミュレータのカメラトピックは /<エンティティ名>/<リンク名>/image_raw。
    # Isaac 版の /<robot>/base_link/camera_link/image_raw から 1 階層減っている。
    # 映像の出し方。operator_stream が H.264 化するときの入力になる。
    #   jpeg … CompressedImage だけ出す (既定)
    #   raw  … Image だけ出す。JPEG の符号化と復号を両端で省ける
    #   both … 両方
    core_jp_camera_publisher = Node(
        package='core_jp_camera_publisher',
        name='publisher_node',
        executable='publisher_node',
        namespace=ROBOT_NAME,
        parameters=[{'output_format': LaunchConfiguration('camera_output_format')}],
        remappings=[
            ('input_image_topic', '/' + ROBOT_NAME + '/camera_link/image_raw'),
            ('top_view_image_topic', '/' + ROBOT_NAME + '/top_view_camera_link/image_raw'),
            ('output_image_topic', '/' + ROBOT_NAME + '/camera_link/image_compressed'),
            ('output_image_raw_topic', '/' + ROBOT_NAME + '/camera_link/image_raw_overlay'),
            ('game_status', '/game_status'),
            ('countdown', '/countdown'),
            ('robot1_hp', '/sample_robot_1/robot_hp'),
            ('robot2_hp', '/sample_robot_2/robot_hp'),
            ('robot3_hp', '/sample_robot_3/robot_hp'),
            ('robot4_hp', '/sample_robot_4/robot_hp'),
            ('robot5_hp', '/sample_robot_5/robot_hp'),
            ('robot6_hp', '/sample_robot_6/robot_hp'),
            ('robot7_hp', '/sample_robot_7/robot_hp'),
            ('robot8_hp', '/sample_robot_8/robot_hp'),
        ],
    )

    # contact センサは std_msgs/Bool を /<エンティティ名>/<リンク名>/contact に出す。
    hp_manager = Node(
        package='hp_manager',
        name='hp_manager_node',
        executable='manager_node',
        namespace=ROBOT_NAME,
        remappings=[
            ('armor_topic_1', '/' + ROBOT_NAME + '/armor1_link/contact_count'),
            ('armor_topic_2', '/' + ROBOT_NAME + '/armor2_link/contact_count'),
            ('armor_topic_3', '/' + ROBOT_NAME + '/armor3_link/contact_count'),
            ('armor_topic_4', '/' + ROBOT_NAME + '/armor4_link/contact_count'),
        ],
        parameters=[{
            'initial_hp': 200,
            'respawn_time_sec': 0.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_output_format', default_value='jpeg'),
        node_robot_state_publisher,
        spawn_robot,
        control_node,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner,
        # rviz,
        teleop_twist_joy,
        core_jp_camera_publisher,
        hp_manager,
    ])
