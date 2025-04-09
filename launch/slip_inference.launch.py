import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='slip_inference',
            executable='inference_node',  
            name='slip_inference_node',
            output='screen'
        )
    ])