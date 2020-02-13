#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Classes to handle Actor objects
"""

import logging
import math

from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle

from carla_cyber_bridge.child import Child
from carla_cyber_bridge.actor_id_registry import ActorIdRegistry

class Actor(Child):

    """
    Generic base class for all carla actors
    """

    global_id_registry = ActorIdRegistry()

    def __init__(self, carla_actor, parent, topic_prefix='', append_role_name_topic_postfix=True):
        """
        Constructor

        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        # each actor defines its own frame
        if append_role_name_topic_postfix:
            if carla_actor.attributes.has_key('role_name'):
                topic_prefix += '/' + carla_actor.attributes['role_name']
            else:
                topic_prefix += '/' + \
                    str(Actor.global_id_registry.get_id(carla_actor.id))
        super(Actor, self).__init__(
            carla_id=carla_actor.id, carla_world=carla_actor.get_world(),
            parent=parent, topic_prefix=topic_prefix)
        self.carla_actor = carla_actor
        logging.debug("Created Actor-{}(id={}, parent_id={},"
                       " type={}, topic_name={}, attributes={}".format(
                           self.__class__.__name__, self.get_id(),
                           self.get_parent_id(), self.carla_actor.type_id,
                           self.topic_name(), self.carla_actor.attributes))

        if self.__class__.__name__ == "Actor":
            logging.warning("Created Unsupported Actor(id={}, parent_id={},"
                          " type={}, attributes={}".format(
                              self.get_id(), self.get_parent_id(),
                              self.carla_actor.type_id, self.carla_actor.attributes))

    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove the reference to the carla.Actor object.
        Finally forward call to super class.

        :return:
        """
        logging.debug(
            "Destroying {}-Actor(id={})".format(self.__class__.__name__, self.get_id()))
        self.carla_actor = None
        super(Actor, self).destroy()

    def get_global_id(self):
        """
        Return a unique global id for the actor used for markers, object ids, etc.

        ros marker id should be int32, carla/unrealengine seems to use int64
        A lookup table is used to remap actor_id to small number between 0 and max_int32

        :return: mapped id of this actor (unique increasing counter value)
        :rtype: uint32
        """
        return Actor.global_id_registry.get_id(self.carla_actor.id)

    def get_cyber_obstacle_msg(self):
        obstacle = PerceptionObstacle()
        obstacle.id = self.get_global_id()
        transform = self.carla_actor.get_transform()
        velocity = self.carla_actor.get_velocity()
        obstacle.position.x = transform.location.x
        obstacle.position.y = -transform.location.y
        obstacle.position.z = transform.location.z
        obstacle.theta = -math.radians(transform.rotation.yaw)
        obstacle.velocity.x = velocity.x
        obstacle.velocity.y = -velocity.y 
        obstacle.velocity.z = velocity.z 
        obstacle.length = self.carla_actor.bounding_box.extent.x * 2.0
        obstacle.width = self.carla_actor.bounding_box.extent.y * 2.0
        obstacle.height = self.carla_actor.bounding_box.extent.z * 2.0
        return obstacle
