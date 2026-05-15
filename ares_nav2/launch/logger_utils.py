import os
import shutil
from datetime import datetime

from launch.actions import (
    DeclareLaunchArgument,
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


def _log_files_in(log_dir):
    log_files = []
    for dirpath, _, filenames in os.walk(log_dir):
        for filename in filenames:
            path = os.path.join(dirpath, filename)
            if os.path.isfile(path):
                log_files.append(path)
    return sorted(log_files, key=lambda path: os.path.relpath(path, log_dir))


def _write_combined_terminal_log(log_dir, terminal_log_path):
    with open(terminal_log_path, 'w', encoding='utf-8') as terminal_log:
        for log_path in _log_files_in(log_dir):
            relative_path = os.path.relpath(log_path, log_dir)
            terminal_log.write(f'\n===== {relative_path} =====\n')
            try:
                with open(log_path, 'r', encoding='utf-8', errors='replace') as log_file:
                    shutil.copyfileobj(log_file, terminal_log)
            except OSError as exc:
                terminal_log.write(f'[logger] failed to read {relative_path}: {exc}\n')
            terminal_log.write('\n')


def make_logger_actions(launch_name):
    """Create common launch actions for saving terminal output."""
    session_config = f'{launch_name}_logger_session_dir'
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    def prepare_session(context, *args, **kwargs):
        root = _absolute_logger_root(
            LaunchConfiguration('logger_root').perform(context)
        )
        session_dir = os.path.join(root, launch_name, timestamp)
        os.makedirs(session_dir, exist_ok=True)

        context.launch_configurations[session_config] = session_dir
        return [
            LogInfo(msg=f'[{launch_name}] logger session: {session_dir}'),
        ]

    def save_terminal_log(context, *args, **kwargs):
        if not _as_bool(LaunchConfiguration('logger_save_terminal').perform(context)):
            return []

        session_dir = LaunchConfiguration(session_config).perform(context)
        launch_log_dir = LaunchLogDir().perform(context)
        launch_log_path = os.path.join(launch_log_dir, 'launch.log')
        terminal_log_path = os.path.join(session_dir, 'terminal.log')

        if os.path.isdir(launch_log_dir) and _log_files_in(launch_log_dir):
            _write_combined_terminal_log(launch_log_dir, terminal_log_path)
            print(f'[{launch_name}] saved combined terminal log: {terminal_log_path}')
        elif os.path.exists(launch_log_path):
            shutil.copy2(launch_log_path, terminal_log_path)
            print(f'[{launch_name}] saved terminal log: {terminal_log_path}')
        else:
            print(f'[{launch_name}] launch.log not found: {launch_log_path}')

        return []

    return [
        DeclareLaunchArgument(
            'logger_root',
            default_value='logger',
            description='Directory where terminal.log is saved.',
        ),
        DeclareLaunchArgument(
            'logger_save_terminal',
            default_value='true',
            description='Copy launch terminal output to logger_root on shutdown.',
        ),
        SetEnvironmentVariable('OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'both'),
        OpaqueFunction(function=prepare_session),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=save_terminal_log)])
        ),
    ]
