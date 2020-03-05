#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""
import logging
import math
import pyproj
import numpy as np

from carla import Location, VehicleControl, Vector3D
from cyber_py import cyber, cyber_time
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.localization.proto.gps_pb2 import Gps
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.common.proto.error_code_pb2 import ErrorCode
from modules.routing.proto.routing_pb2 import RoutingResponse
from modules.transform.proto.transform_pb2 import TransformStamped, TransformStampeds

from .object_sensor import ObjectSensor
from .vehicle import Vehicle

# taken from: https://github.com/davheld/tf/blob/master/src/tf/transformations.py#L1100
def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.
    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple
    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
    True
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = np.empty((4, ), dtype=np.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

class EgoVehicle(Vehicle):

    """
    Vehicle implementation details for the ego vehicle
    """

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static factory method to create ego vehicle actors

        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of the new traffic actor
        :type parent: carla_ros_bridge.Parent
        :return: the created vehicle actor
        :rtype: carla_ros_bridge.Vehicle or derived type
        """
        return EgoVehicle(carla_actor=carla_actor, parent=parent)

    def __init__(self, carla_actor, parent):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        """
        super(EgoVehicle, self).__init__(carla_actor=carla_actor,
                                         parent=parent,
                                         topic_prefix=carla_actor.attributes.get('role_name'),
                                         append_role_name_topic_postfix=False)

        self.object_sensor = None
        if parent.use_object_sensor:
            self.object_sensor = ObjectSensor(self)

        self.vehicle_info_published = False
        self.planned_trajectory = None
        self.control_mode = False

        # self.enable_autopilot_subscriber = rospy.Subscriber(
        #     self.topic_name() + "/enable_autopilot",
        #     Bool, self.enable_autopilot_updated)

        cyber.init()
        self.cyber_node = cyber.Node('carla_ego_node')
        self.cyber_node.create_reader('/apollo/control', ControlCommand, self.cyber_control_command_updated)
        self.cyber_node.create_reader('/apollo/planning', ADCTrajectory, self.planning_callback)
        self.cyber_node.create_reader('/apollo/routing_response', RoutingResponse, self.routing_callback)
        self.tf_writer = self.cyber_node.create_writer('/tf', TransformStampeds)

    def get_marker_color(self):
        """
        Function (override) to return the color for marker messages.

        The ego vehicle uses a different marker color than other vehicles.

        :return: the color used by a ego vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0
        color.g = 255
        color.b = 0
        return color

    def get_localization_msg(self):
        transform = self.carla_actor.get_transform()
        linear_vel = self.carla_actor.get_velocity()
        angular_vel = self.carla_actor.get_angular_velocity()
        accel = self.carla_actor.get_acceleration()

        localization_msg = LocalizationEstimate()
        localization_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        localization_msg.header.frame_id = 'novatel'
        localization_msg.pose.position.x = transform.location.x
        localization_msg.pose.position.y = -transform.location.y
        localization_msg.pose.position.z = transform.location.z
        localization_msg.pose.linear_velocity.x = linear_vel.x
        localization_msg.pose.linear_velocity.y = linear_vel.y
        localization_msg.pose.linear_velocity.z = linear_vel.z
        localization_msg.pose.angular_velocity_vrf.x = angular_vel.x
        localization_msg.pose.angular_velocity_vrf.y = angular_vel.y
        localization_msg.pose.angular_velocity_vrf.z = angular_vel.z
        localization_msg.pose.linear_acceleration_vrf.x = accel.x
        localization_msg.pose.linear_acceleration_vrf.y = accel.y
        localization_msg.pose.linear_acceleration_vrf.z = accel.z
        localization_msg.pose.heading = math.radians(-transform.rotation.yaw)
        return localization_msg

    def get_tf(self):
        transform = self.carla_actor.get_transform()

        tf_msg = TransformStamped()
        tf_msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'localization'
        tf_msg.transform.translation.x = transform.location.x
        tf_msg.transform.translation.y = -transform.location.y
        tf_msg.transform.translation.z = transform.location.z
    
        q = quaternion_from_euler(
            -math.radians(transform.rotation.roll),
             math.radians(transform.rotation.pitch),
            -math.radians(transform.rotation.yaw)
             )
        tf_msg.transform.rotation.qx = q[0]
        tf_msg.transform.rotation.qy = q[1]
        tf_msg.transform.rotation.qz = q[2]
        tf_msg.transform.rotation.qw = q[3]
        return tf_msg

    def planning_callback(self, msg):
        self.planned_trajectory = ADCTrajectory()
        self.planned_trajectory.CopyFrom(msg)

    def routing_callback(self, msg):
        if msg.status.error_code == ErrorCode.OK:
            self.planned_trajectory = None
            distance_threshold = 0.5

            ego_location = self.carla_actor.get_transform().location
            x0 = ego_location.x
            y0 = -ego_location.y
            
            apollo_wp = msg.routing_request.waypoint[0]
            x1 = apollo_wp.pose.x
            y1 = apollo_wp.pose.y

            dist = np.linalg.norm([x1 - x0, y1 - y0])
            if dist > distance_threshold:
                carla_wp = self.parent.map.get_waypoint(Location(x1, -y1, 0))
                self.carla_actor.set_transform(carla_wp.transform)

    def send_vehicle_msgs(self):
        """
        Function (override) to send odometry message of the ego vehicle
        instead of an object message.

        The ego vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'

        :return:
        """
        # vehicle_status = CarlaEgoVehicleStatus()
        # vehicle_status.header.stamp = self.get_current_ros_time()
        # vehicle_status.velocity = self.get_vehicle_speed_abs(self.carla_actor)
        # vehicle_status.acceleration = self.get_vehicle_acceleration_abs(self.carla_actor)
        # vehicle_status.orientation = self.get_current_ros_pose().orientation
        # vehicle_status.control.throttle = self.carla_actor.get_control().throttle
        # vehicle_status.control.steer = self.carla_actor.get_control().steer
        # vehicle_status.control.brake = self.carla_actor.get_control().brake
        # vehicle_status.control.hand_brake = self.carla_actor.get_control().hand_brake
        # vehicle_status.control.reverse = self.carla_actor.get_control().reverse
        # vehicle_status.control.gear = self.carla_actor.get_control().gear
        # vehicle_status.control.manual_gear_shift = self.carla_actor.get_control().manual_gear_shift
        # self.publish_ros_message(self.topic_name() + "/vehicle_status", vehicle_status)

        chassis_msg = Chassis()
        chassis_msg.engine_started = True
        chassis_msg.speed_mps = self.get_vehicle_speed_abs(self.carla_actor)
        chassis_msg.throttle_percentage = self.carla_actor.get_control().throttle * 100.0
        chassis_msg.brake_percentage = self.carla_actor.get_control().brake * 100.0
        chassis_msg.steering_percentage = -self.carla_actor.get_control().steer * 100.0
        chassis_msg.parking_brake = self.carla_actor.get_control().hand_brake
        chassis_msg.header.CopyFrom(self.get_cyber_header())
        chassis_msg.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        self.write_cyber_message('/apollo/canbus/chassis', chassis_msg)

        # if not self.vehicle_info_published:
        #     self.vehicle_info_published = True
        #     vehicle_info = CarlaEgoVehicleInfo()
        #     vehicle_info.type = self.carla_actor.type_id
        #     vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
        #     vehicle_physics = self.carla_actor.get_physics_control()

        #     for wheel in vehicle_physics.wheels:
        #         wheel_info = CarlaEgoVehicleInfoWheel()
        #         wheel_info.tire_friction = wheel.tire_friction
        #         wheel_info.damping_rate = wheel.damping_rate
        #         wheel_info.steer_angle = math.radians(wheel.steer_angle)
        #         wheel_info.disable_steering = wheel.disable_steering
        #         vehicle_info.wheels.append(wheel_info)

        #     vehicle_info.max_rpm = vehicle_physics.max_rpm
        #     vehicle_info.max_rpm = vehicle_physics.max_rpm
        #     vehicle_info.moi = vehicle_physics.moi
        #     vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
        #     vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
        #         vehicle_physics.damping_rate_zero_throttle_clutch_engaged
        #     vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
        #         vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
        #     vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
        #     vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
        #     vehicle_info.clutch_strength = vehicle_physics.clutch_strength
        #     vehicle_info.mass = vehicle_physics.mass
        #     vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
        #     vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
        #     vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
        #     vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

        #     self.publish_ros_message(self.topic_name() + "/vehicle_info", vehicle_info, True)

        # @todo: do we still need this?
        if not self.parent.get_param("challenge_mode"):
            localization_msg = self.get_localization_msg()
            self.write_cyber_message('/apollo/localization/pose', localization_msg)

            tfs = TransformStampeds()
            tf_msg = self.get_tf()
            tfs.transforms.append(tf_msg)
            self.tf_writer.write(tfs)

    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        # objects = super(EgoVehicle, self).get_filtered_objectarray(self.carla_actor.id)
        # self.publish_ros_message(self.topic_name() + '/objects', objects)
        self.send_vehicle_msgs()
        if not self.control_mode:
            self.move_along_planned_trajectory()
        super(EgoVehicle, self).update()

    # For use when you want the bridge to explicitly set the position of the vehicle 
    # in simulation rather than have it calculated by the physics engine.
    def move_along_planned_trajectory(self):
        if self.planned_trajectory is None:
            return
        self.carla_actor.set_simulate_physics(False)
        timestamp = cyber_time.Time.now().to_sec()
        transform = self.carla_actor.get_transform()
        wp = self.parent.map.get_waypoint(transform.location)
        dt = timestamp - self.planned_trajectory.header.timestamp_sec
        for tp in self.planned_trajectory.trajectory_point:
            if dt < tp.relative_time:
                # place car a bit above the ground
                # especially needed in Town02 where the car goes under the map
                height_buffer = 0.1
                #TODO: linear interpolation here
                transform.location.x = tp.path_point.x
                transform.location.y = -tp.path_point.y
                transform.location.z = wp.transform.location.z + height_buffer
                transform.rotation.yaw = -math.degrees(tp.path_point.theta)
                self.carla_actor.set_transform(transform)
                return

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on CarlaEgoVehicleControl commands.
        Finally forward call to super class.

        :return:
        """
        logging.debug("Destroy Vehicle(id={})".format(self.get_id()))
        self.cyber_node = None
        cyber.shutdown()
        super(EgoVehicle, self).destroy()

    # TODO: not yet fully supported, ego drives erratically
    def cyber_control_command_updated(self, cyber_vehicle_control):
        self.control_mode = True
        self.carla_actor.set_simulate_physics(True)

        vehicle_control = VehicleControl()
        vehicle_control.hand_brake = cyber_vehicle_control.parking_brake
        vehicle_control.brake = cyber_vehicle_control.brake / 100.0
        vehicle_control.steer = cyber_vehicle_control.steering_target / 100.0
        vehicle_control.throttle = cyber_vehicle_control.throttle / 100.0
        vehicle_control.reverse = cyber_vehicle_control.gear_location == Chassis.GearPosition.GEAR_REVERSE
        self.carla_actor.apply_control(vehicle_control)

    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: std_msgs.Bool
        :return:
        """
        rospy.logdebug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot.data))
        self.carla_actor.set_autopilot(enable_auto_pilot.data)

    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
            carla_vector.y * carla_vector.y + \
            carla_vector.z * carla_vector.z

    @staticmethod
    def get_vehicle_speed_squared(carla_vehicle):
        """
        Get the squared speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: squared speed of a carla vehicle [(m/s)^2]
        :rtype: float64
        """
        return EgoVehicle.get_vector_length_squared(carla_vehicle.get_velocity())

    @staticmethod
    def get_vehicle_speed_abs(carla_vehicle):
        """
        Get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        speed = math.sqrt(EgoVehicle.get_vehicle_speed_squared(carla_vehicle))
        return speed

    @staticmethod
    def get_vehicle_acceleration_abs(carla_vehicle):
        """
        Get the absolute acceleration of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: vehicle acceleration value [m/s^2 >=0]
        :rtype: float64
        """
        return math.sqrt(EgoVehicle.get_vector_length_squared(carla_vehicle.get_acceleration()))
