#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla lidars
"""

import numpy

from cyber_py import cyber_time
from modules.drivers.proto.pointcloud_pb2 import PointXYZIT, PointCloud

from .sensor import Sensor
import transforms as trans


class Lidar(Sensor):

    """
    Actor implementation details for lidars
    """

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        if topic_prefix is None:
            topic_prefix = 'lidar'
        super(Lidar, self).__init__(carla_actor=carla_actor,
                                    parent=parent,
                                    topic_prefix=topic_prefix,
                                    append_role_name_topic_postfix=append_role_name_topic_postfix)
        self.msg = None
        self.seconds_per_rotation = 1.0 / float(carla_actor.attributes['rotation_frequency'])

    def get_tf_msg(self):
        """
        Function (override) to modify the tf messages sent by this lidar.

        The lidar transformation has to be altered:
        for some reasons lidar sends already a rotated cloud,
        so herein, we need to ignore pitch and roll

        :return: the filled tf message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(Lidar, self).get_tf_msg()
        rotation = tf_msg.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        dummy_roll, dummy_pitch, yaw = tf.transformations.euler_from_quaternion(
            quat)
        # set roll and pitch to zero
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        tf_msg.transform.rotation = trans.numpy_quaternion_to_ros_quaternion(
            quat)
        return tf_msg

    def sensor_data_updated(self, carla_lidar_measurement):
        """
        Function to transform the a received lidar measurement into a ROS point cloud message

        :param carla_lidar_measurement: carla lidar measurement object
        :type carla_lidar_measurement: carla.LidarMeasurement
        """
        # header = self.get_msg_header(use_parent_frame=False)
        lidar_data = numpy.frombuffer(
            carla_lidar_measurement.raw_data, dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 3), 3))
        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        # we need a copy here, because the data are read only in carla numpy
        # array
        lidar_data = -lidar_data
        # we also need to permute x and y
        lidar_data = lidar_data[..., [1, 0, 2]]
        # point_cloud_msg = create_cloud_xyz32(header, lidar_data)
        # self.publish_ros_message(
        #     self.topic_name() + "/point_cloud", point_cloud_msg)

        if self.msg is None:
            self.msg = PointCloud()
            self.msg.header.CopyFrom(self.parent.get_cyber_header())
            lidar_id = self.carla_actor.attributes['role_name']
            self.msg.header.frame_id = lidar_id
            self.msg.frame_id = lidar_id

        self.msg.measurement_time = cyber_time.Time.now().to_sec()
        for lidar_point in lidar_data:
            cyber_point = PointXYZIT()
            cyber_point.x = lidar_point[0]
            cyber_point.y = lidar_point[1]
            cyber_point.z = lidar_point[2]
            self.msg.point.append(cyber_point)

        dt = self.msg.measurement_time - self.msg.header.timestamp_sec
        if dt >= self.seconds_per_rotation:
            self.write_cyber_message('/apollo/sensor/%s/compensator/PointCloud2' % self.msg.frame_id, self.msg)
            self.msg = None
