#!/usr/bin/env python3

"""@author Sam (snaens) Pelz <snaens@snaens.net>"""
import sys
from time import sleep

import yaml

import diagnostic_msgs
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
import rclpy.qos

from bento_diagnostics.log_diagnostics import LogDiagnostics
from bento_diagnostics.nodename_diagnostics import NodenameDiagnostics

"""Watches the standard assortments of ROS nodes running on bento robots
for known irregularities and faults and publishes them as diagnostic messages with diagnostic_updater. 
"""


# Dataflow:
# Since we're hooking a subscriber straight into the ROS log topic we need to cope with many Messages coming in quick
# so we simply check if the log matches any keywords stored in diagnostic_components.
# Then we take that list and publish it though diagnostic_updater


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('bento_diagnostics')

    node.declare_parameter("publish_rate", 1000)  # milliseconds
    node.declare_parameter("config_file", "../resource/diagnostics_config.yaml")

    # The Updater class advertises to /diagnostics
    updater = diagnostic_updater.Updater(node)

    # use namespace (without slash at the front) or if there is no namespace use "bento_robot"
    updater.setHardwareID(node.get_namespace()[1:] if (node.get_namespace() != "/") else "bento_robot")

    # Read data from a YAML config file
    with open(node.get_parameter("config_file").get_parameter_value().string_value, 'r') as file:
        loaded_data = yaml.safe_load(file)

    log_diagnostics = LogDiagnostics(loaded_data, updater)
    nodename_diagnostics = NodenameDiagnostics(loaded_data, updater)

    # Broadcast that we are Initializing
    updater.broadcast(diagnostic_msgs.msg.DiagnosticStatus.OK,
                      'Initializing Diagnostics...')

    qos = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        lifespan=rclpy.qos.Duration(seconds=10),
        history=rclpy.qos.HistoryPolicy.KEEP_ALL)
    node.create_subscription(Log, "/rosout", log_diagnostics.log_callback, qos)

    node.create_timer(0.5, lambda: nodename_diagnostics.nodename_callback(node.get_node_names_and_namespaces()))

    # The state of the node just changed, we can force an immediate update.
    updater.force_update()

    while rclpy.ok():
        try:
            rclpy.spin(node)

        except KeyboardInterrupt:
            pass
        except ExternalShutdownException:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)


if __name__ == '__main__':
    main()
