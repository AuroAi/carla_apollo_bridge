#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a object sensor
"""
import carla

from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles

# TODO: hacky: since this isn't an interface to an actual Carla sensor, it's being 
# instantiated using the bridge constructor's params dict.
class ObjectSensor(object):
    def __init__(self, ego_vehicle):
        self.parent_actor = ego_vehicle.carla_actor
        self.bridge = ego_vehicle.parent
        self.seconds_since_write = 0
        params = self.bridge.params["object_sensor"]
        self.tick_rate = params.get("tick_rate") or 0.1
        self.range = params.get("range") or 100
        self.channel_name = params.get("channel_name") or "/apollo/perception/obstacles"
        self.callback_id = self.bridge.carla_world.on_tick(self.on_tick)

    def __del__(self):
        self.bridge.carla_world.remove_on_tick(self.callback_id)

    def on_tick(self, world_snapshot):
        """
        Get a cyber PerceptionObstacles message including all actors within range, except the ego

        """
        self.seconds_since_write += world_snapshot.delta_seconds
        if self.seconds_since_write >= self.tick_rate:
            self.seconds_since_write -= self.tick_rate
            obstacles = PerceptionObstacles()
            obstacles.header.CopyFrom(self.bridge.get_cyber_header())
            for actor in self.bridge.child_actors.values():
                if actor.carla_actor is not self.parent_actor:
                    if actor.carla_actor.get_location().distance(self.parent_actor.get_location()) <= self.range:
                        if isinstance(actor.carla_actor, carla.Vehicle):
                            obstacles.perception_obstacle.append(actor.get_cyber_obstacle_msg())
                        elif isinstance(actor.carla_actor, carla.Walker):
                            msg = actor.get_cyber_obstacle_msg()
                            msg.type = PerceptionObstacle.Type.PEDESTRIAN
                            obstacles.perception_obstacle.append(msg)
            self.bridge.write_cyber_message(self.channel_name, obstacles)
