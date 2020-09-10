#!/usr/bin/env python3
# Copyright (c) 2019-2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Lunar Lander
============

Classic Lunar Lander game implementation for FVP.
See http://moonlander.seb.ly/ for inspiration.
"""

import argparse
import enum
import json
import math
import struct
import tkinter
from game.gamemanager import GameManager, GameException
from game.log import Log

class LunarLander(GameManager.Game):
    """
    :class:`game.gamemanager.GameManager` implementation of the Lunar Lander. This class and its
    subclasses implement the communication layer and the physics of the game.
    """

    class Packet:
        """
        This is a builder class of packets used between the simulator and FVP.

        +------+------+----------+
        | 0    | 1..n | n + 1    |
        +------+------+----------+
        | Type | Data | Checksum |
        +------+------+----------+

        Packet fields:
           * Type: The packet type is defined by enum values of
             :class:`game.lunarlander.lunarlander.LunarLander.PacketType`.
           * Data: Type specific data. The length of the data field is fixed and specific the packet
             type and can be zero.
           * Checksum: Type and Data fields XOR-ed into a byte and gets inverted. The complete
             message XOR-ed should return 0xff.

        Args:
           * data (bytes): Optional parameter for building packet from raw data.
        """
        def __init__(self, data=None):
            self.data = data[:-1] if data else bytes()
            self.checksum = data[-1] if data else 0xff

        def add_packet_type(self, packet_type):
            """
            Adds the packet type to the start of the packet.

            Args:
               * packet_type (PacketType): Packet type enum value
            """
            self.data = bytes([packet_type.value]) + self.data
            self.checksum ^= self.data[0]

        def add_byte(self, value):
            """
            Appens an 8 bit integer value to the packet.

            Args:
               * value (int): Byte
            """
            self.add_data(struct.pack("B", value))

        def add_short(self, value):
            """
            Appens a 16 bit integer value to the packet. Uses little-endian mode.

            Args:
               * value (int): Integer value
            """
            self.add_data(struct.pack("<h", value))

        def add_data(self, data):
            """
            Appends data to end of the packet and updates the checksum. It can be called multiple
            times on the same packet.

            Args:
               * data (bytes): Packet data
            """
            for data_byte in data:
                self.checksum ^= data_byte
            self.data += data

        def get_short(self, offset):
            """
            Gets a 16 bit integer value from the packet. Uses little-endian mode.

            Args:
               * offset (int): Offset in the data field
            """
            offset += 1 # Message type is skipped
            return struct.unpack_from("<h", self.data, offset)[0]

        def get_data(self):
            """ Returns the raw packet data with the checksum included. """
            return self.data + bytes([self.checksum])

        def is_valid(self):
            """ Validates the checksum of the message. """
            checksum_temp = self.checksum
            for data_byte in self.data:
                checksum_temp ^= data_byte
            return checksum_temp == 0xff

    class PacketType(enum.Enum):
        """
        Defines an enum for the packet types.

        Packet types:
           * ``MAP``: Height map data
           * ``LANDING_ZONES``: List of landing zones
           * ``INITIAL_IMU_STATE``: Initial inertial state of the spacecraft
           * ``START_LANDING``: Start landing process by confirming end of handshake
           * ``IMU_UPDATE``: Update inertial measurement unit
           * ``SET_THRUST``: Set the thrust values
           * ``ERROR``: Error message
        """
        MAP = 0x00
        LANDING_ZONES = 0x01
        INITIAL_IMU_STATE = 0x02
        START_LANDING = 0x03
        IMU_UPDATE = 0x04
        SET_THRUST = 0x05
        ERROR = 0xff

    class Map:
        """
        Handles the map by storing the height for each X coordinate. It loads the map from a .png
        file. Its size must be 512x128 pixels. It determines the height in each column by getting
        the Y coordinate of the top pixel having different color than white. Each column must have
        at least one non-white pixel. A non-white pixel in the bottom row means ``0`` value and in
        the top row it means ``127``.

        Args:
           * file (str): Path of the map image
        """
        SPACE_COLOR = (255, 255, 255)
        WIDTH = 512
        HEIGHT = 128

        def __init__(self, file):
            _ = tkinter.Tk()
            map_file = tkinter.PhotoImage(file=file)

            width = map_file.width()
            height = map_file.height()

            if self.WIDTH != width or self.HEIGHT != height:
                raise GameException("The map size should be %dx%d instead of %dx%d" % (
                    self.WIDTH, self.HEIGHT, width, height), GameException.MODE_PRIVATE)

            self.height_map = []
            for map_x in range(width):
                for map_y in range(height):
                    color = map_file.get(map_x, map_y)
                    if color != self.SPACE_COLOR:
                        self.height_map.append(height - map_y - 1)
                        break
                else:
                    raise GameException("No ground at x = %d" % map_x, GameException.MODE_PRIVATE)

        def validate_landing_zones(self, landing_zones):
            """
            Checks if the landing zones has clearances on both sides.

            Args:
               * landing_zones (LandingZone[]): Landing zones
            """
            clearance = LunarLander.LandingZone.LANDING_ZONE_CLEARANCE
            for landing_zone in landing_zones:
                ref_x = int(landing_zone.get_x().get_float_value())
                ref_y = self.height_map[ref_x]
                for x_coord in range(ref_x - clearance, ref_x + clearance + 1):
                    if self.height_map[x_coord] != ref_y:
                        raise GameException("Landing zone has no clearance: %d" % ref_x,
                                            GameException.MODE_PRIVATE)

        def get_height(self, x_coord):
            """
            Queries the height value for an X coordinate.

            Args:
               * x_coord (int): X coordinate (0-511)

            """
            return self.height_map[x_coord]

        def is_ground(self, position):
            """
            Check if the position is ground.

            Args:
               * position (Vector): Position to check
            """
            x_coord = position.x.get_float_value()
            y_coord = position.y.get_float_value()
            height = self.height_map[int(x_coord)]

            return y_coord <= height

        def check_boundaries(self, position):
            """
            Check if the position is within the boundaries of the map.

            Args:
               * position (Vector): Position to check
            """
            if not 0 <= int(position.x.get_float_value()) < self.WIDTH:
                raise GameException("The spacecraft has left the landing zone",
                                    GameException.MODE_PUBLIC, "X")
            if not 0 <= int(position.y.get_float_value()) < self.HEIGHT:
                raise GameException("The spacecraft has left the landing zone",
                                    GameException.MODE_PUBLIC, "Y")

        def check_line(self, position_start, position_end):
            """
            The function checks if the line between two coordinates would hit the ground.
            By converting each coordinate on the path to booleans indicate ground the list should
            contain zero or one raising edge. This means the spacecraft either flew in space during
            the entire tick or it touched the ground. In the second case futher checks should
            validate if the speed, angle, etc. all were in the safe range.

            Args:
               * position_start (Vector): staring position
               * position_end (Vector): ending position
            """
            x_start = position_start.x.get_float_value()
            x_end = position_end.x.get_float_value()
            y_start = position_start.y.get_float_value()
            y_end = position_end.y.get_float_value()

            if int(x_start) != int(x_end):
                # Normal trajectory: Checking both sides of each pixel except the first because it
                # is in the other direction.

                has_reached_ground = False
                slope = (y_end - y_start) / (x_end - x_start)

                if x_start < x_end:
                    # Flying in positive direction
                    # Plus 1 for skipping the back side of the starting pixel
                    x_current = x_start + 1.0

                    while x_current <= x_end:
                        x_left = int(x_current)
                        x_right = x_left + 1

                        y_left = y_start + (slope * (x_left - x_start))
                        y_right = y_start + (slope * (x_right - x_start))

                        # x_left is the "real" coordinate here:
                        if min(y_left, y_right) < self.height_map[x_left]:
                            has_reached_ground = True
                        elif has_reached_ground:
                            raise Exception("The spacecraft has hit the ground between ticks",
                                            GameException.MODE_PUBLIC)

                        x_current += 1.0
                else:
                    # Flying in negative direction
                    # Minus 1 for skipping the back side of the starting pixel
                    x_current = x_start - 1.0

                    while x_current >= x_end:
                        x_left = int(x_current)
                        x_right = x_left + 1

                        y_left = y_end - (slope * (x_end - x_left))
                        y_right = y_end - (slope * (x_end - x_right))

                        # x_left is the "real" coordinate here:
                        if min(y_left, y_right) < self.height_map[x_left]:
                            has_reached_ground = True
                        elif has_reached_ground:
                            raise Exception("The spacecraft has hit the ground between ticks",
                                            GameException.MODE_PUBLIC)

                        x_current -= 1.0
            else:
                # Flying on a vertical line could cause zero devision in slope. In this case no
                # error could happen as the ground cannot have hollow parts.
                pass

        def pack(self, packet):
            """ Adds the map into a Packet. """
            packet.add_data(bytes(self.height_map))

    class LandingZone:
        """
        The class represents a landing zone by its X coordinate and its score multiplier.

        Args:
           * x (int): X coordinate of the landing zone (0-511)
           * value (int): Score multiplier
        """
        LANDING_ZONE_COUNT = 4
        LANDING_ZONE_CLEARANCE = 2

        def __init__(self, x, value):
            self.x = LunarLander.Scalar(x) # pylint: disable=invalid-name
            self.value = LunarLander.Scalar(value)

        def get_value(self):
            """ Queries the value of the landing zone. """
            return self.value

        def get_x(self):
            """ Returns the X coordinate of the landing zone. """
            return self.x

        def pack(self, packet):
            """ Adds the landing zone to the packet. """
            self.x.pack(packet)
            self.value.pack(packet)

    class Scalar:
        """
        The values are represented by fixed point rational numbers. Each number has 10 bits for
        the integer part and 6 bits for the fraction. The values are signed.
        Internally it stores the value as a python integer so it uses more than 16 bits for
        negative numbers. Do not modify the internal value externally.

        +----+-----+---+---+---------+---+
        | 15 | ... | 6 | 5 |   ...   | 0 |
        +----+-----+---+---+---------+---+
        | Integer bits | Fractional bits |
        +--------------+-----------------+

        Examples:
           * ``0x7fff`` = `511.984375`
           * ``0x0001`` = `0.015625`
           * ``0x0000`` = `0.0`
           * ``0xffff`` = `-0.015625`
           * ``0x8000`` = `-512`

        Args:
           * floating_value (float): Floating point initial value
        """
        BASE = (1 << 6)

        def __init__(self, floating_value=0.0):
            self.value = int(floating_value * self.BASE)
            self.check_value("Scalar.__init__")

        def get_float_value(self):
            """ Gets the value as a floating point number. """
            return self.value / self.BASE

        def pack(self, packet):
            """ Packs the scalar into a packet. """
            packet.add_short(self.value)

        def unpack(self, packet, offset):
            """ Unpacks the scalar from a packet at a given offset. """
            self.value = packet.get_short(offset)

        def add_unsafe(self, scalar):
            """ Add another scalar to the object without checking for an overflow. """
            self.value += scalar.value

        def add(self, scalar):
            """ Adds another scalar to the object with overflow checking. """
            self.value += scalar.value
            self.check_value("Scalar.add")

        def multiply(self, scalar):
            """ Multiplies the object with another scalar """
            self.value *= scalar.value
            self.value = int(self.value / self.BASE)
            self.check_value("Scalar.multiply")

        def check_value(self, source):
            """
            Checks if the value store in the class is valid. Otherwire it raises an exception.
            """
            if self.value < -32768 or self.value > 32767:
                raise GameException("Scalar overflow happened",
                                    GameException.MODE_PUBLIC, source)

        def __str__(self):
            return "%.06f" % self.get_float_value()

    class Vector:
        """
        Two dimensional vector

        Args:
           * x (float): X coordinate
           * y (float): Y coordinate
        """

        def __init__(self, x, y):
            self.x = LunarLander.Scalar(x) # pylint: disable=invalid-name
            self.y = LunarLander.Scalar(y) # pylint: disable=invalid-name

        def add_unsafe(self, vector):
            """
            Adds another vector to the object without checking for an overflow.

            Args:
               * vector (Vector): Vector to add
            """
            self.x.add_unsafe(vector.x)
            self.y.add_unsafe(vector.y)

        def add(self, vector):
            """
            Add operator

            Args:
               * vector (Vector): Vector to add
            """
            self.x.add(vector.x)
            self.y.add(vector.y)

        def pack(self, packet):
            """ Packs the vector into a packet. """
            self.x.pack(packet)
            self.y.pack(packet)

        def __str__(self):
            return "(%s, %s)" % (self.x, self.y)

    class Imu:
        """
        This class simulates the kinetic behavior of the spacecraft. The initial inertial state is
        read from the argument and extended by zero as an angular acceleration and angular speed.
        The state of the spacecraft is updated by the tick function.

        Args:
           * initial_imu (dict): Initial position and velocity values. The initial y value of
             the acceleration is considered to be the gravity which is constant during the flight.
        """
        MOMENT_OF_INTERTIA = 1.0
        MASS = 1.0

        # Negative thrust does not exist and the maximal value is 8.0
        MIN_THRUST = 0.0
        MAX_THRUST = 8.0

        # +-1 in radian/tick which is approx. +-57 degress/tick as the MOMENT_OF_INERTIAL is 1.0
        MIN_ROTATION_THRUST = -1
        MAX_ROTATION_THRUST = 1

        # Horizontal and vertical landing velocities
        MIN_LANDING_VELOCITY_X = -0.5
        MAX_LANDING_VELOCITY_X = 0.5

        MIN_LANDING_VELOCITY_Y = -1.0
        MAX_LANDING_VELOCITY_Y = 0.0

        # The spacecraft should point upwards +- 15 degrees
        MIN_LANDING_ANGLE = math.radians(90.0 - 15.0)
        MAX_LANDING_ANGLE = math.radians(90.0 + 15.0)

        def __init__(self, initial_imu):
            self.position = LunarLander.Vector(
                initial_imu["position"]["x"], initial_imu["position"]["y"])
            self.velocity = LunarLander.Vector(
                initial_imu["velocity"]["x"], initial_imu["velocity"]["y"])
            self.acceleration = LunarLander.Vector(0, initial_imu["gravity"])
            self.gravity = LunarLander.Scalar(initial_imu["gravity"])
            self.angle = LunarLander.Scalar(initial_imu["angle"])
            self.angular_speed = LunarLander.Scalar(0.0)
            self.angular_acceleration = LunarLander.Scalar(0.0)

        def limit_thrusts(self, thrust, rotate):
            """
            Limits the thrust and rotational thrust into the valid range and logs invalid values.

            Args:
               * thrust (Scalar): Thrust value
               * rotate (Scalar): Rotational thrust value
            """
            if thrust.get_float_value() < self.MIN_THRUST:
                Log.log("[LunarLander] Invalid thrust value: %s" % str(thrust))
                thrust = LunarLander.Scalar(self.MIN_THRUST)
            elif self.MAX_THRUST < thrust.get_float_value():
                Log.log("[LunarLander] Invalid thrust value: %s" % str(thrust))
                thrust = LunarLander.Scalar(self.MAX_THRUST)

            if rotate.get_float_value() < self.MIN_ROTATION_THRUST:
                Log.log("[LunarLander] Invalid rotational thrust value: %s" % str(rotate))
                rotate = LunarLander.Scalar(self.MIN_ROTATION_THRUST)
            elif self.MAX_ROTATION_THRUST < rotate.get_float_value():
                Log.log("[LunarLander] Invalid rotational thrust value: %s" % str(rotate))
                rotate = LunarLander.Scalar(self.MAX_ROTATION_THRUST)

            return thrust, rotate

        def tick(self, thrust, rotate):
            """
            Periodic update function of the inertial system. First it updates the angle then the
            position using the updated angle and the thrust. The values are limited internally.

            Args:
               * thrust (Scalar): Thrust force
               * rotate (Scalar): Rotating force
            """

            thrust, rotate = self.limit_thrusts(thrust, rotate)

            rotate.multiply(LunarLander.Scalar(self.MOMENT_OF_INTERTIA))
            self.angular_acceleration = rotate
            self.angular_speed.add(self.angular_acceleration)
            self.angle.add(self.angular_speed)

            raw_angle = self.angle.get_float_value()
            raw_acceleration = thrust.get_float_value() * self.MASS
            force_x = raw_acceleration * math.cos(raw_angle)
            force_y = raw_acceleration * math.sin(raw_angle)

            self.acceleration.x = LunarLander.Scalar(force_x)
            self.acceleration.y = LunarLander.Scalar(force_y)
            self.acceleration.y.add(self.gravity)
            self.velocity.add(self.acceleration)
            # Unsafe add is used to be able to raise a more specific error on boundary checks.
            self.position.add_unsafe(self.velocity)

        def validate_landing_velocity(self):
            """ Checks if the landing velocity components are within the boundaries. """
            velocity_x = self.velocity.x.get_float_value()
            velocity_y = self.velocity.y.get_float_value()

            return self.MIN_LANDING_VELOCITY_X <= velocity_x <= self.MAX_LANDING_VELOCITY_X and \
                   self.MIN_LANDING_VELOCITY_Y <= velocity_y <= self.MAX_LANDING_VELOCITY_Y

        def validate_landing_angle(self):
            """ Checks if the landing angle is within the boundaries. """
            angle = self.angle.get_float_value() % (2 * math.pi)
            return self.MIN_LANDING_ANGLE <= angle <= self.MAX_LANDING_ANGLE

        def pack_all(self, packet):
            """
            Packs the position, velocity and acceleration vectors and the angle, angular speed and
            angular acceleration scalars into the packet.
            """
            self.position.pack(packet)
            self.velocity.pack(packet)
            self.acceleration.pack(packet)
            self.angle.pack(packet)
            self.angular_speed.pack(packet)
            self.angular_acceleration.pack(packet)

        def pack(self, packet):
            """ Packs the acceleration vector and the angular acceleration into the packet. """
            self.acceleration.pack(packet)
            self.angular_acceleration.pack(packet)

        def __str__(self):
            return "P%s V%s A%s ANG(%s) ANG_S(%s) ANG_A(%s)" % (
                self.position, self.velocity, self.acceleration,
                self.angle, self.angular_speed, self.angular_acceleration)

    def __init__(self, argv, fvpconnector):
        parser = argparse.ArgumentParser("... --gameargs")
        parser.add_argument("--round")
        args = parser.parse_args(argv)

        try:
            with open(args.round, "r") as map_file:
                self.round = json.load(map_file)
        except json.JSONDecodeError as exception:
            raise GameException("JSON Decode error", GameException.MODE_PRIVATE) from exception

        Log.log("[LunarLander] Round config: " + str(self.round))

        self.map = LunarLander.Map(self.round["map"])
        self.imu = self.Imu(self.round["imu"])

        self.landing_zones = []
        for landing_zone in self.round["landing_zones"]:
            self.landing_zones.append(self.LandingZone(landing_zone["x"], landing_zone["value"]))
        if len(self.landing_zones) != self.LandingZone.LANDING_ZONE_COUNT:
            raise GameException("Invalid landing zone count %d" % (
                len(self.landing_zones)), GameException.MODE_PRIVATE)

        self.map.validate_landing_zones(self.landing_zones)

        self.fvpconnector = fvpconnector
        self.tick_count = 0
        self.points = 0

    def receive_data(self, length):
        """ Receives data with the given length and handles exception propagation. """
        try:
            return self.fvpconnector.recv(length)
        except Exception as exception:
            raise GameException("Communication error", GameException.MODE_PUBLIC) from exception

    def receive_packet(self, length):
        """
        Receives a packet from the FVP with the given length including packet type,
        data and checksum.
        """
        return self.Packet(self.receive_data(length))

    def send_packet(self, packet):
        """ Send a packet to the FVP. """
        try:
            return self.fvpconnector.send(packet.get_data())
        except Exception as exception:
            raise GameException("Communication error", GameException.MODE_PUBLIC) from exception

    def send_error_packet(self):
        """ Sends an error packet to the FVP. """
        packet = self.Packet()
        packet.add_packet_type(self.PacketType.ERROR)
        self.send_packet(packet)

    def handshake(self):
        """
        The handshake function handles the initial packet exchange. It can respond to the
        following requests. The function sends responses on requests. The same request can be
        send multiple times except ``START_LANDING`` which initiates the landing process where
        the initial requests are not available anymore.
        If an invalid packet was received by this function (i.e. it has checksum error) it sends
        and ``ERROR`` packet as a response.

        Packet types:
           * ``MAP``: Requesting height map
           * ``LANDING_ZONES``: Requesting landing zones
           * ``INITIAL_IMU_STATE``: Requesting inertial state
           * ``START_LANDING``: Starting the landing process
        """

        packet = self.receive_packet(2)
        if not packet.is_valid():
            # Invalid checksum, sending an error packet
            Log.log("[LunarLander] Invalid packet was received")
            self.send_error_packet()
            return True

        request = packet.get_data()[0]
        if request == self.PacketType.MAP.value:
            # Map
            Log.log("[LunarLander] Map request received")
            packet = self.Packet()
            packet.add_packet_type(self.PacketType.MAP)
            self.map.pack(packet)
            self.send_packet(packet)
        elif request == self.PacketType.LANDING_ZONES.value:
            # Landing zones
            Log.log("[LunarLander] Landing zones request received")
            packet = self.Packet()
            packet.add_packet_type(self.PacketType.LANDING_ZONES)
            for landing_zone in self.landing_zones:
                landing_zone.pack(packet)
            self.Scalar(self.round["max_ticks"]).pack(packet)
            self.send_packet(packet)
        elif request == self.PacketType.INITIAL_IMU_STATE.value:
            # Initial IMU state
            Log.log("[LunarLander] Initial kinetic state request received")
            packet = self.Packet()
            packet.add_packet_type(self.PacketType.INITIAL_IMU_STATE)
            self.imu.pack_all(packet)
            self.send_packet(packet)
        elif request == self.PacketType.START_LANDING.value:
            # Start landing
            Log.log("[LunarLander] Start landing request received")
            packet = self.Packet()
            packet.add_packet_type(self.PacketType.START_LANDING)
            self.send_packet(packet)

            # Exit handshake phase
            return False
        else:
            Log.log("[LunarLander] Invalid handshake request byte from FVP %02X" % request)
            self.send_error_packet()

        return True

    def tick(self):
        """
        Periodic update function of the game. It handles the ``IMU_UPDATE`` and ``SET_THRUST``
        messages and updates the IMU according to them. It also checks the completion and error
        conditions.
        """
        Log.log("[LunarLander] Tick #%d" % self.tick_count)

        if self.tick_count >= self.round["max_ticks"]:
            raise GameException("The maximum tick count has been exceeded",
                                GameException.MODE_PUBLIC)

        data = self.receive_data(1)
        if data[0] == self.PacketType.IMU_UPDATE.value:
            # Get IMU update
            Log.log("[LunarLander] Get IMU update")
            data += self.receive_data(1)
            packet = self.Packet(data)
            if not packet.is_valid():
                Log.log("[LunarLander] Invalid packet was received")
                self.send_error_packet()
                return True

            packet = self.Packet()
            packet.add_packet_type(self.PacketType.IMU_UPDATE)
            self.imu.pack(packet)
            self.send_packet(packet)
        elif data[0] == self.PacketType.SET_THRUST.value:
            # Set thrust
            data += self.receive_data(5)
            packet = self.Packet(data)
            if not packet.is_valid():
                Log.log("[LunarLander] Invalid packet was received")
                self.send_error_packet()
                return True

            thrust = self.Scalar()
            rotate = self.Scalar()
            thrust.unpack(packet, 0)
            rotate.unpack(packet, 2)

            position_previous = self.imu.position

            # Update IMU, thrust values are limited internally
            Log.log("[LunarLander] Set thrust: %s %s" % (str(thrust), str(rotate)))
            self.imu.tick(thrust, rotate)
            Log.log("[LunarLander] Updated IMU " + str(self.imu))

            position_current = self.imu.position

            # Checking map boundaries
            self.map.check_boundaries(position_current)

            # Check if the line between the starting and ending position would hit the ground
            self.map.check_line(position_previous, position_current)

            # Checking stop condition
            if self.map.is_ground(position_current):
                self.evaluate_landing()
                return False

            packet = self.Packet()
            packet.add_packet_type(self.PacketType.SET_THRUST)
            self.send_packet(packet)

            self.tick_count += 1
        else:
            Log.log("[LunarLander] Invalid tick request byte from FVP %02X" % data[0])
            self.send_error_packet()

        return True

    def evaluate_landing(self):
        """
        The function checks each condition at landing.
           * Velocity component limits
           * Angle limits
           * Distance from landing zones
        """
        # Check landing velocity
        if not self.imu.validate_landing_velocity():
            raise GameException("The spacecraft has crashed into ground", GameException.MODE_PUBLIC)

        # Checking landing angle
        if not self.imu.validate_landing_angle():
            raise GameException("The spacecraft has overturned", GameException.MODE_PUBLIC)

        # Check landing zone distance
        x_coord = self.imu.position.x.get_float_value()

        min_distance = None
        min_landing_zone = None
        for landing_zone in self.landing_zones:
            distance = abs(x_coord - landing_zone.get_x().get_float_value())
            if min_distance is None or distance < min_distance:
                min_distance = distance
                min_landing_zone = landing_zone

        if min_distance > LunarLander.LandingZone.LANDING_ZONE_CLEARANCE:
            raise GameException("Landed too far from landing zone",
                                GameException.MODE_PUBLIC, str(x_coord))

        # Calculating score
        clearance = float(self.LandingZone.LANDING_ZONE_CLEARANCE)
        distance_points = (clearance - min_distance) / clearance

        max_ticks = self.round["max_ticks"]
        tick_points = (max_ticks - self.tick_count) / max_ticks

        multiplier = min_landing_zone.get_value().get_float_value()

        self.points = distance_points * 30 + (tick_points * 70 * multiplier)

    def get_results(self):
        """ Returns the result of the game as a dict. """
        return self.points, "The spacecraft has landed successfully"
