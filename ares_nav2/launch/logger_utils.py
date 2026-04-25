import os
import shutil
from datetime import datetime

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, LaunchLogDir


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _workspace_root():
    first_prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if first_prefix and os.path.basename(first_prefix) == 'install':
        return os.path.dirname(first_prefix)
    return os.getcwd()


def _absolute_logger_root(root):
    root = os.path.expanduser(root)
    if os.path.isabs(root):
        return root
    return os.path.abspath(os.path.join(_workspace_root(), root))


def make_logger_actions(launch_name, default_topics):
    """Create common launch actions for terminal-log and rosbag recording."""
    session_config = f'{launch_name}_logger_session_dir'
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_topic_string = ' '.join(default_topics)

    def prepare_session(context, *args, **kwargs):
        root = _absolute_logger_root(
            LaunchConfiguration('logger_root').perform(context)
        )
        session_dir = os.path.join(root, launch_name, timestamp)
        os.makedirs(session_dir, exist_ok=True)

        metadata_path = os.path.join(session_dir, 'metadata.txt')
        with open(metadata_path, 'w', encoding='utf-8') as metadata_file:
            metadata_file.write(f'launch_name: {launch_name}\n')
            metadata_file.write(f'timestamp: {timestamp}\n')
            metadata_file.write(
                'topic_recording_enabled: '
                f'{LaunchConfiguration("logger_record_topics").perform(context)}\n'
            )
            metadata_file.write(
                'topics: '
                f'{LaunchConfiguration("logger_topics").perform(context)}\n'
            )

        context.launch_configurations[session_config] = session_dir
        return [
            LogInfo(msg=f'[{launch_name}] logger session: {session_dir}'),
        ]

    def start_topic_logger(context, *args, **kwargs):
        if not _as_bool(LaunchConfiguration('logger_record_topics').perform(context)):
            return []

        topics = LaunchConfiguration('logger_topics').perform(context).split()
        if not topics:
            return [
                LogInfo(msg=f'[{launch_name}] topic logger skipped: no topics')
            ]

        session_dir = LaunchConfiguration(session_config).perform(context)
        bag_dir = os.path.join(session_dir, 'rosbag')
        cmd = ['ros2', 'bag', 'record', '--output', bag_dir]
        cmd.extend(topics)

        return [
            ExecuteProcess(
                cmd=cmd,
                name=f'{launch_name}_topic_logger',
                output='screen',
            )
        ]

    def save_terminal_log(context, *args, **kwargs):
        if not _as_bool(LaunchConfiguration('logger_save_terminal').perform(context)):
            return []

        session_dir = LaunchConfiguration(session_config).perform(context)
        launch_log_dir = LaunchLogDir().perform(context)
        launch_log_path = os.path.join(launch_log_dir, 'launch.log')
        terminal_log_path = os.path.join(session_dir, 'terminal.log')

        if os.path.exists(launch_log_path):
            shutil.copy2(launch_log_path, terminal_log_path)
            print(f'[{launch_name}] saved terminal log: {terminal_log_path}')
        else:
            print(f'[{launch_name}] launch.log not found: {launch_log_path}')

        raw_log_dir = os.path.join(session_dir, 'ros_launch_raw')
        if os.path.isdir(launch_log_dir):
            shutil.copytree(launch_log_dir, raw_log_dir, dirs_exist_ok=True)

        return []

    return [
        DeclareLaunchArgument(
            'logger_root',
            default_value='logger',
            description='Directory where launch logs and rosbag files are saved.',
        ),
        DeclareLaunchArgument(
            'logger_save_terminal',
            default_value='true',
            description='Copy launch terminal output to logger_root on shutdown.',
        ),
        DeclareLaunchArgument(
            'logger_record_topics',
            default_value='true',
            description='Record logger_topics with ros2 bag unless set to false.',
        ),
        DeclareLaunchArgument(
            'logger_topics',
            default_value=default_topic_string,
            description='Space-separated topic list for ros2 bag record.',
        ),
        SetEnvironmentVariable('OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'both'),
        OpaqueFunction(function=prepare_session),
        OpaqueFunction(function=start_topic_logger),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=save_terminal_log)])
        ),
    ]
