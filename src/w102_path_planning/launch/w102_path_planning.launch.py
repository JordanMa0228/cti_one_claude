from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Results are written alongside the workspace by default.
    # Override by setting W102_RESULTS_DIR in the environment.
    results_dir = os.path.join(
        os.path.dirname(__file__),
        '..', '..', '..', '..', '..', 'results'
    )

    return LaunchDescription([
        Node(
            package='w102_path_planning',
            executable='w102_path_sim',
            name='w102_path_sim',
            output='screen',
            emulate_tty=True,
            env={'W102_RESULTS_DIR': os.path.abspath(results_dir)},
        )
    ])
