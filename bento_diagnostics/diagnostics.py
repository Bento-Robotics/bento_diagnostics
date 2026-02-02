#!/usr/bin/env python3

"""@author Sam (snaens) Pelz <snaens@snaens.net>"""
import sys
from time import sleep
from typing import List, Dict

import yaml

import diagnostic_msgs
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import rclpy
from diagnostic_msgs.msg._diagnostic_status import Metaclass_DiagnosticStatus
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
import rclpy.qos

"""Watches the standard assortments of ROS nodes running on bento robots
for known irregularities and faults and publishes them as diagnostic messages with diagnostic_updater. 
"""


# Dataflow:
# Since we're hooking a subscriber straight into the ROS log topic we need to cope with many Messages coming in quick
# so we simply check if the log matches any keywords stored in diagnostic_components.
# Then we take that list and publish it though diagnostic_updater

# TODO: decay to stale
# TODO: something to detect offline / crashed nodes

class ErrorMessage:
    def __init__(self):
        self.index_pointer = -1
        self.names: List[str] = []
        self.keywords: List[str] = []
        self.messages: List[str] = []
        self.levels: List[Metaclass_DiagnosticStatus] = []
        self.priorities: List[int] = []


diagnostic_components: Dict[str, ErrorMessage] = {}

# TODO: make this thing quicker - it builds up a queue of messages when a spam comes in
def log_callback(msg: Log):
    for component in diagnostic_components.values():
        counter = 0
        for keyword in component.keywords:
            if msg.msg.find(keyword) != -1:
                if component.priorities[counter] >= component.priorities[
                    component.index_pointer] if component.index_pointer != -1 else True:
                    component.index_pointer = counter
                # print(msg.msg)  # parrot all key log messages
            counter += 1


def update_diagnostics(component_name):
    component = diagnostic_components[component_name]

    def defined_checker(stat):
        if component.index_pointer == -1:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE, 'Not updated yet')
        else:
            stat.summary(component.levels[component.index_pointer], component.messages[component.index_pointer])
            stat.add('Henlo', 'frens')
        return stat

    return defined_checker


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

    # Parse YAML:
    # "diagnostics" top-level list of diagnostic thingies
    for diagnostic in loaded_data["diagnostics"]:
        # save the stuff from YAML into a ErrorMessage entry
        message = ErrorMessage()
        # "log_messages" list of indivitdual log keywords to look for and react to
        for error_message in diagnostic["log_messages"]:
            message.names.append(error_message["name"])
            message.keywords.append(error_message["keyword"])
            message.messages.append(error_message["message"]),
            # convert text to python code - just using numbers 0..3 didn't work :(
            message.levels.append(eval("diagnostic_msgs.msg.DiagnosticStatus." + error_message["level"]))
            try:  # this lets users leave out the priority in YAML and have 0 be default
                message.priorities.append(error_message["priority"])
            except KeyError:
                message.priorities.append(0)

        # and save that entry to diagnostic_components
        diagnostic_components[diagnostic["name"]] = message
        # add a diagnostic_updater hook for it too
        updater.add(diagnostic["name"], update_diagnostics(diagnostic["name"]))

    # Broadcast that we are Initializing
    updater.broadcast(diagnostic_msgs.msg.DiagnosticStatus.OK,
                      'Initializing Diagnostics...')

    qos = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        lifespan=rclpy.qos.Duration(seconds=10),
        history=rclpy.qos.HistoryPolicy.KEEP_ALL)
    node.create_subscription(Log, "/rosout",
                             log_callback, qos)

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
