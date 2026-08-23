#!/usr/bin/env python3
"""Reference-named ROS1 entry point for the fixed-point controller.

The implementation lives in ``fixed_point_offboard.py`` for backwards
compatibility with the first ROS1 launch file.  The public executable name
matches the reference ROS2 bringup: ``fixed_point_hover.py``.
"""

import importlib.util
import os

import rospy


def load_controller_class():
    # catkin_install_python executes this source file from a generated relay;
    # importing the sibling relay would not expose its source globals. Load
    # the implementation by its source path instead.
    source_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "fixed_point_offboard.py")
    spec = importlib.util.spec_from_file_location(
        "bringup_fixed_point_offboard_impl", source_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.FixedPointOffboard


if __name__ == "__main__":
    rospy.init_node("fixed_point_hover")
    try:
        load_controller_class()().run()
    except ValueError as error:
        rospy.logfatal("Fixed-point parameter error: %s", error)
