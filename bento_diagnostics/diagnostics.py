#!/usr/bin/env python3

"""@author Sam (snaens) Pelz <snaens@snaens.net>"""
import sys
from time import sleep

import diagnostic_msgs
import diagnostic_updater
import rclpy
from rclpy.executors import ExternalShutdownException

from bento_diagnostics.diagnose_demo import Demo_Diagnostics

"""Watches the standard assortments of ROS nodes running on bento robots
for known irregularities and faults and publishes them as diagnostic messages with diagnostic_updater. 
"""


def main():
    rclpy.init()
    node = rclpy.create_node('bento_diagnostics')

    # The Updater class advertises to /diagnostics
    updater = diagnostic_updater.Updater(node)

    node.declare_parameter('hardware_id', "bento_robot")

    # use namespace (without slash at the front) or if there is no namespace use "bento_robot"
    updater.setHardwareID(node.get_namespace()[1:] if (node.get_namespace() != "/") else "bento_robot")

    # Broadcast that we are Initializing
    updater.broadcast(diagnostic_msgs.msg.DiagnosticStatus.OK,
                      'Initializing Diagnostics...')

    demo = Demo_Diagnostics(node, updater)
    updater.add('Diagnostics demonstration', demo.run)
    # updater.add('Teleop Subsystem', demo.run)

    # The state of the node just changed, we can force an immediate update.
    updater.force_update()

    while rclpy.ok():
        try:
            sleep(0.1)
            rclpy.spin_once(node, timeout_sec=1)

        except KeyboardInterrupt:
            pass
        except ExternalShutdownException:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)


if __name__ == '__main__':
    main()
