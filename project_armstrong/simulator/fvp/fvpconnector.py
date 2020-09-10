#!/usr/bin/env python3
# Copyright (c) 2019-2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
FVP connector
=============

The module is responsible for connecting to the FVP over TCP for generic data transmission.
On the FVP side of the TCP port the connection is forwarded to the UART modules so basically
this module acts like transparent connection to the UART module of the FVP.
"""

import socket
import time

class FvpConnector:
    """
    Interface for connecting to the FVP over TCP.

    Constants:
       * ``RETRY_COUNT`` - Retry count of connecting to the FVP
       * ``RETRY_PERIOD`` - Delay between connection attempts
       * ``MAX_IDLE_TIME`` - Maximum idle time between messages
    """
    RETRY_COUNT = 30
    RETRY_PERIOD = 1.0
    MAX_IDLE_TIME = 5.0

    def __init__(self):
        self.sock = None
        self.debug_mode = False
        self.connect_count = 0

    def set_debug_mode(self):
        """
        The function sets the debug_mode flag to True. This disables all timeouts in the
        communication.
        """
        self.debug_mode = True

    def has_connect_timed_out(self):
        """
        Determines if the initial connecting phase has timed out.
        In debug mode it always returns False.
        """
        if self.debug_mode:
            return False

        self.connect_count -= 1
        return self.connect_count == 0

    def connect(self, host, port):
        """
        The function tries to connect the FVP multiple times according to the contants of the
        class.

        Args:
           * host (str): Hostname of the FVP running station
           * port (int): TCP port number of the FVP
        """

        self.connect_count = self.RETRY_COUNT
        timeout = None if self.debug_mode else FvpConnector.MAX_IDLE_TIME

        while not self.has_connect_timed_out():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(timeout)
                self.sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
                self.sock.connect((host, port))
                return
            except ConnectionRefusedError:
                time.sleep(FvpConnector.RETRY_PERIOD)

        raise Exception("Failed to connect to FVP")

    def send(self, data):
        """
        Sends data to the FVP.

        Args:
           * data (bytes): Data to send
        """
        self.sock.send(data)

    def recv(self, length):
        """
        Receives data from FVP.

        Args:
           * length (int): The exact amount of data to be received
        """
        data = bytes()
        while len(data) < length:
            fragment = self.sock.recv(length - len(data))
            if not fragment:
                raise Exception("Failed to receive data")
            data += fragment

        return data
