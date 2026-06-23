import launch
import launch_ros
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 启动 TTS 服务
    tts_node = launch_ros.actions.Node(
        package='paddle_speech_ros',
        executable='tts_action_server',
        name='tts_action_server',
        output='screen'
    )

    # 启动 Llama 服务
    llama_node = launch_ros.actions.Node(
        package='llama_ros',
        executable='llama_action_server',
        name='llama_action_server',
        output='screen'
    )

    # 启动机械臂管理服务
    arm_manage_node = launch_ros.actions.Node(
        package='arm_controller',
        executable='arm_manage_server',
        name='arm_manage_server',
        output='screen'
    )

    # 启动 Whisper 服务
    whisper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('whisper_bringup'),
                'launch',
                'whisper.launch.py'
            ])
        ]),
        launch_arguments={}.items()
    )

    # 启动相机（使用 XML 文件的正确方式）
    astra_launch_path = os.path.join(
        FindPackageShare('astra_camera').find('astra_camera'),
        'launch',
        'astra_pro.launch.xml'
    )
    astra_launch = IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(astra_launch_path),
        launch_arguments={}.items()
    )

    # 延迟启动连续语音交互节点（等待 Whisper 服务启动）
    delayed_whisper_demo = TimerAction(
        period=8.0,  # 等待8秒确保Whisper服务完全启动
        actions=[
            launch_ros.actions.Node(
                package='whisper_demos',
                executable='whisper_demo_nonstop_node',
                name='whisper_demo_nonstop_node',
                output='screen'
            )
        ]
    )

    # 启动苹果检测节点（最后启动）
    apple_detect_node = launch_ros.actions.Node(
        package='object_detect',
        executable='apple_detect',
        name='apple_detect',
        output='screen'
    )

    return launch.LaunchDescription([
        tts_node,
        llama_node,
        arm_manage_node,
        whisper_launch,
        astra_launch,
        delayed_whisper_demo,
        apple_detect_node
    ])
