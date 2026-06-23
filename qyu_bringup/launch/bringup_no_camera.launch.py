import launch
import launch_ros
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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

    # 延迟启动连续语音交互节点（等待 Whisper 服务启动）
    delayed_whisper_demo = TimerAction(
        period=8.0,
        actions=[
            launch_ros.actions.Node(
                package='whisper_demos',
                executable='whisper_demo_nonstop_node',
                name='whisper_demo_nonstop_node',
                output='screen'
            )
        ]
    )

    return launch.LaunchDescription([
        tts_node,
        llama_node,
        whisper_launch,
        delayed_whisper_demo
    ])
