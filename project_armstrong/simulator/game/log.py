#!/usr/bin/env python3
# Copyright (c) 2019-2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Log
===

Module for logging messages from the simulator.
"""

import sys
import time
import threading

class Log:
    """
    System log class. The default target of log messages is the standard error pipe.
    """
    FILE = sys.stderr
    START_TIME = time.time()
    LOCK = threading.Lock()

    @staticmethod
    def set_file(log_file):
        """
        Sets the log file object which will be the output file of the log system.

        Args:
           * log_file (obj): File object

        """
        with Log.LOCK:
            Log.FILE = log_file

    @staticmethod
    def log(message):
        """
        Logs message along a timestamp. The timestamp is the amount of time from the start of the
        software. The function is thread safe.
        """
        with Log.LOCK:
            for line in message.splitlines():
                print("%3.3f %s" % (time.time() - Log.START_TIME, line), file=Log.FILE)
