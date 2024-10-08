import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name = 'simplearmContainer',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [
            ComposableNode(
                package='simplearm',
                plugin='simplearm::Kinematic1',
                name='pub_joint_state',
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='both',
    )

    return launch.LaunchDescription([container])
