import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
import launch.events
import launch_ros.event_handlers
from ament_index_python.packages import get_package_share_directory
import sys

def generate_launch_description():
    ld = LaunchDescription()

    # Define LifecycleNode for path_planner
    path_planner_node = LifecycleNode(
        package="path_planner",
        executable="path_planner",
        name="path_planner",
        namespace='',
        output="both",
        emulate_tty=True,
    )

    # When the path_planner reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_path_planner_reaches_inactive_state = RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=path_planner_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="node 'path_planner' reached the 'inactive' state, 'activating'."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(path_planner_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Make the path_planner node take the 'configure' transition.
    emit_event_to_request_that_path_planner_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(path_planner_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Add the actions to the launch description.
    ld.add_action(register_event_handler_for_path_planner_reaches_inactive_state)
    ld.add_action(path_planner_node)
    ld.add_action(emit_event_to_request_that_path_planner_does_configure_transition)

    return ld

if __name__ == '__main__':
    generate_launch_description()
