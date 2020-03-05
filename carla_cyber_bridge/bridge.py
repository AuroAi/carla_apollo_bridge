#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Cyberbridge class:

Class that handle communication between CARLA and Cyber
"""
import logging
import threading
import time
from cyber_py import cyber

from .parent import Parent
from .map import Map

class CarlaCyberBridge(Parent):

    """
    Carla Cyber bridge
    """

    def __init__(self, carla_world, params):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        cyber.init()
        self.cyber_node = cyber.Node('carla_cyber_client_node')
        self.params = params
        self.use_object_sensor = self.params.get("object_sensor") is not None
        super(CarlaCyberBridge, self).__init__(
            carla_id=0, carla_world=carla_world, frame_id='/map')

        self.timestamp_last_run = 0.0
        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.actor_list = []
        self.map = self.carla_world.get_map()

        # register callback to create/delete actors
        self.update_child_actors_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_update_child_actors)

        # register callback to update actors
        self.update_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_time_tick)

        self.writers = {}



    def destroy(self):
        """
        Function (virtual) to destroy this object.

        Lock the update mutex.
        Remove all publisher.
        Finally forward call to super class.

        :return:
        """
        if not self.update_child_actors_lock.acquire(False):
            logging.warning('Failed to acquire child actors lock')
        if not self.update_lock.acquire(False):
            logging.warning('Failed to acquire update lock')
        self.actor_list = []
        cyber.shutdown()
        super(CarlaCyberBridge, self).destroy()

    def write_cyber_message(self, channel, msg):
        if channel not in self.writers:
            self.writers[channel] = self.cyber_node.create_writer(channel, type(msg))
        self.writers[channel].write(msg)

    def get_param(self, key, default=None):
        """
        Function (override) to query global parameters passed from the outside.

        :param key: the key of the parameter
        :type key: string
        :param default: the default value of the parameter to return if key is not found
        :type default: string
        :return: the parameter string
        :rtype: string
        """
        return self.params.get(key, default)

    def topic_name(self):
        """
        Function (override) to get the topic name of this root entity.

        The topic root '/carla' is returned by this bridge class.

        :return: the final topic name of this
        :rtype: string
        """
        return "/carla"

    def _carla_time_tick(self, carla_timestamp):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates.

        After successful locking the update mutex
        (only perform trylock to respect bridge processing time)
        the clock and the children are updated.
        Finally the ROS messages collected to be published are sent out.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if cyber.ok():
            if self.update_lock.acquire(False):
                if self.timestamp_last_run < carla_timestamp.elapsed_seconds:
                    self.timestamp_last_run = carla_timestamp.elapsed_seconds
                    # can't do this since there's no 'sim time' in cyber
                    # self._update_clock(carla_timestamp)
                    self.update()
                self.update_lock.release()

    def _carla_update_child_actors(self, _):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates of the actors

        After successful locking the mutex
        (only perform trylock to respect bridge processing time)
        the existing actors are updated.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if cyber.ok():
            if self.update_child_actors_lock.acquire(False):
                # cache actor_list once during this update-loop
                self.actor_list = self.carla_world.get_actors()
                self.update_child_actors()
                # actors are only created/deleted around once per second
                time.sleep(1)
                self.update_child_actors_lock.release()

    def run(self):
        """
        Run the bridge functionality.

        Spins cyber.

        :return:
        """
        self.cyber_node.spin()
        self.destroy()

    def get_actor_list(self):
        """
        Function (override) to provide actor list

        :return: the list of actors
        :rtype: list
        """
        return self.actor_list
