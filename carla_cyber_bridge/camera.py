#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to handle Carla camera sensors
"""
import logging
from abc import abstractmethod

import math
import numpy

import cv2
from modules.drivers.proto.sensor_image_pb2 import CompressedImage
from modules.drivers.proto.pointcloud_pb2 import PointXYZIT, PointCloud

import carla
from carla_cyber_bridge.sensor import Sensor
import carla_cyber_bridge.transforms as trans


class Camera(Sensor):

    """
    Sensor implementation details for cameras
    """

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static factory method to create camera actors

        :param carla_actor: carla camera actor object
        :type carla_actor: carla.Camera
        :param parent: the parent of the new traffic actor
        :type parent: carla_ros_bridge.Parent
        :return: the created camera actor
        :rtype: carla_ros_bridge.Camera or derived type
        """
        if carla_actor.type_id.startswith("sensor.camera.rgb"):
            return RgbCamera(carla_actor=carla_actor, parent=parent)
        elif carla_actor.type_id.startswith("sensor.camera.depth"):
            return DepthCamera(carla_actor=carla_actor, parent=parent)
        elif carla_actor.type_id.startswith("sensor.camera.semantic_segmentation"):
            return SemanticSegmentationCamera(carla_actor=carla_actor, parent=parent)
        else:
            return Camera(carla_actor=carla_actor, parent=parent)

    def __init__(self, carla_actor, parent, topic_prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera'
        super(Camera, self).__init__(carla_actor=carla_actor,
                                     parent=parent,
                                     topic_prefix=topic_prefix)

        if self.__class__.__name__ == "Camera":
            logging.warning("Created Unsupported Camera Actor"
                          "(id={}, parent_id={}, type={}, attributes={})".format(
                              self.get_id(), self.get_parent_id(),
                              self.carla_actor.type_id, self.carla_actor.attributes))

    def _build_camera_info(self):
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        # store info without header
        camera_info.header = None
        camera_info.width = int(self.carla_actor.attributes['image_size_x'])
        camera_info.height = int(self.carla_actor.attributes['image_size_y'])
        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (
            2.0 * math.tan(float(self.carla_actor.attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        self._camera_info = camera_info

    def sensor_data_updated(self, carla_image):
        """
        Function (override) to transform the received carla image data
        into a ROS image message

        :param carla_image: carla image object
        :type carla_image: carla.Image
        """
        if ((carla_image.height != int(self.carla_actor.attributes['image_size_y'])) or
                (carla_image.width != int(self.carla_actor.attributes['image_size_x']))):
            logging.error(
                "Camera{} received image not matching configuration".format(self.topic_name()))

        image_data_array, encoding = self.get_carla_image_data_array(
            carla_image=carla_image)

        cyber_img = CompressedImage()
        cyber_img.header.CopyFrom(self.parent.get_cyber_header())
        cyber_img.header.frame_id = self.carla_actor.attributes['role_name']
        cyber_img.frame_id = cyber_img.header.frame_id
        cyber_img.format = 'jpeg'
        cyber_img.measurement_time = cyber_img.header.timestamp_sec
        cyber_img.data = cv2.imencode('.jpg', image_data_array)[1].tostring()
        self.write_cyber_message('/apollo/sensor/camera/%s/image/compressed' % self.carla_actor.attributes['role_name'], cyber_img)

    def get_tf_msg(self):
        """
        Function (override) to modify the tf messages sent by this camera.

        The camera transformation has to be altered to look at the same axis
        as the opencv projection in order to get easy depth cloud for RGBD camera

        :return: the filled tf message
        :rtype: geometry_msgs.msg.TransformStamped
        """
        tf_msg = super(Camera, self).get_tf_msg()
        rotation = tf_msg.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        tf_msg.transform.rotation = trans.numpy_quaternion_to_ros_quaternion(
            quat)
        return tf_msg

    @abstractmethod
    def get_carla_image_data_array(self, carla_image):
        """
        Virtual function to convert the carla image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """
        raise NotImplementedError(
            "This function has to be re-implemented by derived classes")

    @abstractmethod
    def get_image_topic_name(self):
        """
        Virtual function to provide the actual image topic name

        :return image topic name
        :rtype string
        """
        raise NotImplementedError(
            "This function has to be re-implemented by derived classes")


class RgbCamera(Camera):

    """
    Camera implementation details for rgb camera
    """

    def __init__(self, carla_actor, parent, topic_prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera/rgb'
        super(RgbCamera, self).__init__(carla_actor=carla_actor,
                                        parent=parent,
                                        topic_prefix=topic_prefix)

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        The RGB camera provides a 4-channel int8 color format (bgra).

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        return carla_image_data_array, 'bgra8'

    def get_image_topic_name(self):
        """
        virtual function to provide the actual image topic name

        :return image topic name
        :rtype string
        """
        return "image_color"


class DepthCamera(Camera):

    """
    Camera implementation details for depth camera
    """

    def __init__(self, carla_actor, parent, topic_prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera/depth'
        super(DepthCamera, self).__init__(carla_actor=carla_actor,
                                          parent=parent,
                                          topic_prefix=topic_prefix)

    def sensor_data_updated(self, carla_image):
        image_data_array, encoding = self.get_carla_image_data_array(
                carla_image=carla_image)
        x = len(image_data_array[0]) / 2.0
        y = len(image_data_array) / 2.0

        # add noise to the range values from the image
        image_data_array += numpy.random.normal(0, .050, image_data_array.shape)

        msg = PointCloud()
        msg.header.CopyFrom(self.parent.get_cyber_header())
        msg.header.frame_id = self.carla_actor.attributes['role_name']
        msg.frame_id = msg.header.frame_id
        msg.measurement_time = msg.header.timestamp_sec
        w = float(carla_image.width)
        h_fov = float(carla_image.fov) * numpy.pi / 180.0
        f = w / 2.0 / numpy.tan(h_fov / 2.0)
        for u in range(len(image_data_array[0])):
            for v in range(len(image_data_array)):
                point = PointXYZIT()
                point.x = image_data_array[v][u]
                point.y = -(u - x) * point.x / f
                point.z = -(v - y) * point.x / f
                msg.point.append(point)

        self.write_cyber_message('/apollo/sensor/%s/compensator/PointCloud2' % msg.frame_id, msg)

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        The depth camera raw image is converted to a linear depth image
        having 1-channel float32.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        # color conversion within C++ code is broken, when transforming a
        #  4-channel uint8 color pixel into a 1-channel float32 grayscale pixel
        # therefore, we do it on our own here
        #
        # @todo: After fixing https://github.com/carla-simulator/carla/issues/1041
        # the final code in here should look like:
        #
        # carla_image.convert(carla.ColorConverter.Depth)
        #
        # carla_image_data_array = numpy.ndarray(
        #    shape=(carla_image.height, carla_image.width, 1),
        #    dtype=numpy.float32, buffer=carla_image.raw_data)
        #
        bgra_image = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)

        # Apply (R + G * 256 + B * 256 * 256) / (256**3 - 1) * 1000
        # according to the documentation:
        # https://carla.readthedocs.io/en/latest/cameras_and_sensors/#camera-depth-map
        scales = numpy.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
        depth_image = numpy.dot(bgra_image, scales).astype(numpy.float32)

        # actually we want encoding '32FC1'
        # which is automatically selected by cv bridge with passthrough
        return depth_image, 'passthrough'

    def get_image_topic_name(self):
        """
        Function (override) to provide the actual image topic name

        :return image topic name
        :rtype string
        """
        return "image_depth"


class SemanticSegmentationCamera(Camera):

    """
    Camera implementation details for segmentation camera
    """

    def __init__(self, carla_actor, parent, topic_prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'camera/semantic_segmentation'
        super(
            SemanticSegmentationCamera, self).__init__(carla_actor=carla_actor,
                                                       parent=parent,
                                                       topic_prefix=topic_prefix)

    def get_carla_image_data_array(self, carla_image):
        """
        Function (override) to convert the carla image to a numpy data array
        as input for the cv_bridge.cv2_to_imgmsg() function

        The segmentation camera raw image is converted to the city scapes palette image
        having 4-channel uint8.

        :param carla_image: carla image object
        :type carla_image: carla.Image
        :return tuple (numpy data array containing the image information, encoding)
        :rtype tuple(numpy.ndarray, string)
        """

        carla_image.convert(carla.ColorConverter.CityScapesPalette)
        carla_image_data_array = numpy.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=numpy.uint8, buffer=carla_image.raw_data)
        return carla_image_data_array, 'bgra8'

    def get_image_topic_name(self):
        """
        Function (override) to provide the actual image topic name

        :return image topic name
        :rtype string
        """
        return "image_segmentation"
