#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Entry point for carla simulator ROS bridge
"""

from cyber_py import cyber

import yaml
import carla

from carla_ros_bridge.bridge import CarlaRosBridge
from carla_ros_bridge.bridge_with_rosbag import CarlaRosBridgeWithBag


def main():
    """
    main function for carla simulator ROS bridge
    maintaiing the communication client and the CarlaRosBridge objects
    """
    cyber.init('carla_cyber_client')

    params = yaml.load(open("../../config/settings.yaml"))['carla']
    host = params['host']
    port = params['port']

    carla_client = carla.Client(host=host, port=port)
    carla_client.set_timeout(2000)

    carla_world = carla_client.get_world()

    carla_ros_bridge = CarlaRosBridge(
        carla_world=carla_client.get_world(), params=params)
    carla_ros_bridge.run()
    carla_ros_bridge = None

    del carla_world
    del carla_client

if __name__ == "__main__":
    main()
