#!/usr/bin/env python3
# Copyright (c) 2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Runner script
=============

The script starts the :class:`game.gamemanager.GameManager` with
:class:`game.lunarlander.lunarlander.LunarLander` as the current game.
"""

import sys
from game.gamemanager import GameManager
from game.lunarlander.lunarlander import LunarLander

GAME_MANAGER = GameManager(LunarLander)
GAME_MANAGER.parse_args(sys.argv)
GAME_MANAGER.main()
