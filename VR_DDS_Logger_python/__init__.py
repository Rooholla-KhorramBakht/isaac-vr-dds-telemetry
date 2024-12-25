# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
try:
    from .extension import *
except:
    pass # This is a workaround to avoid import error when telemetry module is outside isaac echosystem
import os
ASSETS_PATH = os.path.join(os.path.dirname(__file__), 'assets/')
