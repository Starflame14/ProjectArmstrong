# Copyright (c) 2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

export PYTHONPATH=simulator
python3 simulator/run_lunar_lander.py \
--fvp=FVP_MPS2_Cortex-M0 \
--firmware=./project/rocket.bin \
--gameargs "--round content/example_map.json" \
$@
