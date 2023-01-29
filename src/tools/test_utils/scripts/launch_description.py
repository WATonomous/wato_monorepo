import unittest

import launch
import launch.actions
import launch_ros.actions

def launch_description(test_nodes, validators, data_generator):
    """
    Generate a launch description to run system tests.

    :param test_nodes:
    :param validators:
    :param data_generator:
    """
    ld = launch.LaunchDescription()
    ld.add_action(test_nodes)
    if data_generator is not None:
        ld.add_action(data_generator)
    for validator in validators:
        ld.add_action(validator)
    return ld
