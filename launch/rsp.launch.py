import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('simplearm').find('simplearm')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'simplearm.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params])

    rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return launch.LaunchDescription([
        rsp,
        rviz2])
