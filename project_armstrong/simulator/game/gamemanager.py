#!/usr/bin/env python3
# Copyright (c) 2019-2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Game Manager
============

Main module of the project which controls the workflow.
"""

import abc
import argparse
import hashlib
import json
import time
import traceback
from fvp.fvpconnector import FvpConnector
from fvp.fvprunner import FvpRunner
from game.log import Log

class GameException(Exception):
    """
    Game specific exception class. It contains a public and a private message to avoid leaking
    information from the testing environment. The private data can be used to print into the log.
    The class has two modes: private or public. In private mode it only shows an error ID publicly
    and puts the details of the error into the private message. The public mode prints the message
    but it can still have extra private information.

    Args:
       * message (str): Error message
       * mode (int): ``MODE_PRIVATE`` or ``MODE_PUBLIC``
       * private_extra (str): Extra private information
    """
    MODE_PRIVATE = 0
    MODE_PUBLIC = 1

    def __init__(self, message, mode, private_extra=""):
        Exception.__init__(self, message)
        self.message = message

        if mode == self.MODE_PUBLIC:
            self.public_message = message
            self.private_message = message + " " + private_extra
        else:
            timestamp = time.time()
            message_hash = hashlib.sha256(bytes(str(timestamp) + message, "ascii"))
            message_hash = message_hash.hexdigest()[:16]

            self.public_message = "Internal error, please contact organizers: %s" % message_hash
            self.private_message = "Internal error (%s): %s %s" % (
                message_hash, message, private_extra)

    def get_private_message(self):
        """ Queries the private data of the exception which includes a traceback """
        return self.private_message + "\n" + traceback.format_exc()

    def __str__(self):
        return self.public_message

class GameManager:
    """
    This class managed the whole workflow of the FVP based games.

    Args:
       * game (class): Class reference of the current game
    """
    class Game(metaclass=abc.ABCMeta):
        """ Interface for FVP based games """

        @abc.abstractmethod
        def handshake(self):
            """
            Sending initial messages to FVP and accepting handshake. Return True while in handshake
            phase.
            """

        @abc.abstractmethod
        def tick(self):
            """ Periodic tick function of the game. Returns True while the game is running. """

        @abc.abstractmethod
        def get_results(self):
            """ Returns the results in of the game: score, message """

    def __init__(self, game):
        self.game = game
        self.fvpconnector = None
        self.fvprunner = None
        self.args = None
        self.results = []

    def parse_args(self, argv):
        """
        Parsing command line arguments. Run the simulator with ``--help`` to get the actual
        arguments.

        Args:
           * argv (obj): Argument list (i.e. ``sys.argv``)
        """
        parser = argparse.ArgumentParser(description="Arm Competition Environment")
        parser.add_argument("--fvp", help="FVP executable path")
        parser.add_argument("--keep-fvp-open", help="Do not kill the FVP process at the end",
                            action="store_true")
        parser.add_argument("--firmware", help="Firmware file (*.bin)")
        parser.add_argument("--host", help="FVP host address (default: localhost)",
                            default="localhost")
        parser.add_argument("--port", help="FVP TCP port (default: 5000)", type=int, default=5000)
        parser.add_argument("--logfile", help="Log file (default: stderr)")
        parser.add_argument("--timeout", help="Timeout per round in seconds (default: 10)",
                            type=int, default=10)
        parser.add_argument("--debug", help="FVP debug mode with disabled timeouts",
                            action="store_true")
        parser.add_argument("--real-time-fvp", help=
                            "Start FVP with realtime scheduling policy (requires root privileges)",
                            action="store_true")
        parser.add_argument("--gameargs", help="Arguments for the game as a single string",
                            action="append", default=[])

        self.args = parser.parse_args(argv[1:])

    def print_args(self):
        """ Logs the command line arguments """
        args_items = vars(self.args)
        for arg in args_items:
            Log.log("[GAME] %s = %s" % (arg, args_items[arg]))

    def add_round_result(self, score, message):
        """ Adds the result of a round to the list. """
        self.results.append({"score": score, "message": str(message)})

    def print_results(self):
        """ Prints results in a standard JSON format. """
        total_rounds = 0
        passed_rounds = 0
        total_score = 0.0

        for result in self.results:
            if result["score"] > 0.0:
                passed_rounds += 1
                total_score += result["score"]
            total_rounds += 1

        full_result = {
            "total_rounds": total_rounds,
            "passed_rounds": passed_rounds,
            "total_score": total_score,
            "rounds": self.results
        }

        json_result = json.dumps(full_result)

        Log.log("[GAME] Results: " + json_result)
        print(json_result)

    def wait_with_timeout(self, start_time, function):
        """ Calls a function while it returns true and checks the timeout after each call. """
        while function():
            if not self.args.debug and time.time() - start_time > self.args.timeout:
                raise GameException("The round has timed out", GameException.MODE_PUBLIC)

    def main(self):
        """
        Application entry point. The flow of the function looks like as follow:

        * Starting FVP if the FVP and the firmware paths are set
        * Connecting to FVP
        * Calling the ``handshake`` function of the game implementation
        * Calling the ``tick`` function in a loop while it returns True
        * Stopping FVP if it was started by the ``GameManager``
        * Printing game results to the standard output
        """
        if self.args.logfile:
            Log.set_file(open(self.args.logfile, "a"))

        Log.log("[GAME] Started")
        self.print_args()

        for gameargs in self.args.gameargs:
            self.fvpconnector = FvpConnector()
            if self.args.debug:
                self.fvpconnector.set_debug_mode()

            if self.args.fvp and self.args.firmware:
                Log.log("[GAME] Starting FVP")
                self.fvprunner = FvpRunner(self.args.fvp,
                                           self.args.real_time_fvp,
                                           self.args.firmware)

                if self.args.debug:
                    self.fvprunner.set_debug_mode()

                self.fvprunner.start()

            Log.log("[GAME] Connecting to FVP")
            self.fvpconnector.connect(self.args.host, self.args.port)
            Log.log("[GAME] Connected")

            try:
                start_time = time.time()
                game_instance = self.game(gameargs.split(" "), self.fvpconnector)
                Log.log("[GAME] Handshake")

                self.wait_with_timeout(start_time, game_instance.handshake)
                self.wait_with_timeout(start_time, game_instance.tick)

                score, message = game_instance.get_results()
                self.add_round_result(score, message)
            except GameException as exception:
                Log.log("[GAME] Exception: " + exception.get_private_message())
                self.add_round_result(0.0, str(exception))
            except Exception as exception: # pylint: disable=broad-except
                game_exception = GameException(type(exception).__name__ + str(exception),
                                               GameException.MODE_PRIVATE)
                Log.log("[GAME] Unknown exception: " + game_exception.get_private_message())
                self.add_round_result(0.0, str(game_exception))
            finally:
                if self.fvprunner and not self.args.keep_fvp_open:
                    Log.log("[GAME] Stopping FVP")
                    self.fvprunner.kill()

        self.print_results()
        Log.log("[GAME] Exiting")
