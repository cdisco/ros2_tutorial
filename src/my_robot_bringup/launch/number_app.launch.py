
from launch import LaunchDescription
from launch_ros.actions import Node

# name of function must match (it's a hook)
def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    number_publisher_node = Node(
        package="my_py_pkg", 
        executable="number_publisher",
        name="my_renamed_publisher",
        remappings=[
            remap_number_topic
            ],
        parameters=[
            {"number_to_publish": 400},
            {"publish_frequency": 150}
        ]
    )

    number_pub_and_sub = Node(
        package="my_cpp_pkg", 
        executable="number_pub_and_sub",
        name="my_number_counter",
        remappings=[remap_number_topic,
                     ("number_count", "my_number_count")]
    )
    
    ld.add_action(number_publisher_node)
    ld.add_action(number_pub_and_sub)
    
    return ld