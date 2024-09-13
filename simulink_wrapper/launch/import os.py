import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('simulink_wrapper'),
        'config',
        'node_parameters.yaml'
    )

    # Nodo subscriber
    subscriber_node = Node(
        package='simulink_wrapper',
        name='subscriber',
        executable='subscriber'
    )

    # Nodo modelnode
    modelnode = Node(
        package='simulink_wrapper',
        name='modelnode',
        executable='modelnode',
        parameters=[config]
    )

    # Nodo publisher
    publisher_node = Node(
        package='simulink_wrapper',
        name='publisher',
        executable='publisher'
    )

    # Evento: Avvia modelnode solo dopo che subscriber è stato avviato
    modelnode_after_subscriber = RegisterEventHandler(
        OnProcessStart(
            target_action=subscriber_node,
            on_start=[modelnode]
        )
    )

    # Evento: Avvia publisher solo dopo che modelnode è stato avviato
    publisher_after_modelnode = RegisterEventHandler(
        OnProcessStart(
            target_action=modelnode,
            on_start=[publisher_node]
        )
    )

    # Aggiungi i nodi e gli eventi alla descrizione del lancio
    ld.add_action(subscriber_node)
    ld.add_action(modelnode_after_subscriber)
    ld.add_action(publisher_after_modelnode)

    return ld
