from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    change_parameters = [{"frecuency_spawn": 1}]

    spawner = Node(
        package="turtle_game",
        executable="turtle_spawner",
        parameters=change_parameters,
    )

    ld.add_action(spawner)

    logic = Node(package="turtle_game", executable="turtle_logic")

    ld.add_action(logic)

    turtle_sim = Node(package="turtlesim", executable="turtlesim_node")
    ld.add_action(turtle_sim)

    return ld
