#!/usr/bin/env python3
# Copyright (c) 2019-2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
FVP runner
==========

This module is responsible for running FVP models.
"""

import subprocess
import threading
from game.log import Log

class FvpRunner(threading.Thread):
    """
    The class is responsible for starting the FVP. It implements ``threading.Thread`` and the FVP
    is started as a subprocess from a separate thread. After starting the subprocess the thread
    handles the logging of the FVP stdout and stderr.

    Args:
       * executable (str): Path of the FVP executable
       * firmware (str): Path of the firmware which will be loaded to the 0x0 address

    """
    def __init__(self, executable, real_time_fvp, firmware):
        threading.Thread.__init__(self, name="FvpRunner")
        if real_time_fvp:
            self.args = ["chrt"]
            self.add_option("-f 99")
            self.add_option(executable)
        else:
            self.args = [executable]
        self.add_option("-C fvp_mps2.telnetterminal0.mode=raw")
        self.add_option("-C fvp_mps2.telnetterminal0.start_telnet=0")
        self.add_option("-C fvp_mps2.telnetterminal0.start_port=5000")
        self.add_option("-C fvp_mps2.telnetterminal1.start_port=5100")
        self.add_option("-C fvp_mps2.telnetterminal2.start_port=5200")
        self.add_option("--data %s@0x00000000" % firmware)
        self.proc = None

    def add_option(self, option):
        """
        Appends an option to FVP command line.

        Args:
           * option (str): Option string

        """
        self.args += option.split(" ")

    def set_debug_mode(self):
        """ Enables CADI debug server for debugging with ARM DS. """
        self.add_option("-S")

    def run(self):
        """
        Main function of the FVP runner thread which starts the FVP process.
        It also forwards the stdout/stderr of the FVP to the log.
        """
        Log.log("[FVP] Starting \"%s\"" % " ".join(self.args))
        self.proc = subprocess.Popen(self.args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        while self.proc.poll() is None:
            for stdout_line in self.proc.stdout.readlines():
                Log.log("[FVP out] %s" % str(stdout_line, "ascii").strip())

            for stderr_line in self.proc.stderr.readlines():
                Log.log("[FVP err] %s" % str(stderr_line, "ascii").strip())

    def kill(self):
        """ Stops the FVP process and waits the thread to join. """
        if self.proc.returncode is None:
            self.proc.kill()
        self.join()
