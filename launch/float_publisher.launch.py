"""Launch the float publisher demo

   ros2 launch JIB float_publisher.launch.py

   This is only intended to demonstrate the example.  Please
   edit/update as appropriate.

   This starts:
   1) The float_publisher.py
   2) The subscriber.py

"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the float_publisher.
    node_publisher = Node(
        name       = 'float',
        package    = 'JIB',
        executable = 'float_publisher',
        output     = 'screen',
        on_exit    = Shutdown())

    # Configure a node for the subscriber.
    node_subscriber = Node(
        name       = 'subscriber',
        package    = 'JIB',
        executable = 'subscriber',
        output     = 'screen',
        on_exit    = Shutdown())


    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        # Start the boolean pulisher and the subscriber.
        node_publisher,
        node_subscriber,
    ])
