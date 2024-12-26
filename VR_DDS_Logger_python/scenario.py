# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


class ScenarioTemplate:
    def __init__(self):
        pass

    def setup_scenario(self):
        pass

    def teardown_scenario(self):
        pass

    def update_scenario(self):
        pass


import numpy as np
from omni.isaac.core.utils.types import ArticulationAction

"""
This scenario takes in a robot Articulation and makes it move through its joint DOFs.
Additionally, it adds a cuboid prim to the stage that moves in a circle around the robot.

The particular framework under which this scenario operates should not be taken as a direct
recomendation to the user about how to structure their code.  In the simple example put together
in this template, this particular structure served to improve code readability and separate
the logic that runs the example from the UI design.
"""


class FlexivJointMimicScenario(ScenarioTemplate):
    def __init__(self):
        self._object = None
        self._articulation = None

        self._running_scenario = False
        self.flexiv_q0 = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])

    def setup_scenario(self, articulation, object_prim):
        self._articulation = articulation
        self._object = object_prim

        self._initial_object_position = self._object.get_world_pose()[0]
        self._running_scenario = True
        self._lower_joint_limits = articulation.dof_properties["lower"]
        self._upper_joint_limits = articulation.dof_properties["upper"]
        articulation.set_joint_positions(self.flexiv_q0)

    def teardown_scenario(self):
        self._time = 0.0
        self._object = None
        self._articulation = None
        self._running_scenario = False
        self._lower_joint_limits = None
        self._upper_joint_limits = None

    def update_scenario(self, step: float):
        if not self._running_scenario:
            return
        # self._object.set_world_pose(np.array([x, y, z]))
        # joint_position_target = self._calculate_position(self._joint_time, self._path_duration)
        # joint_velocity_target = self._calculate_velocity(self._joint_time, self._path_duration)

        # action = ArticulationAction(
        #     np.array([joint_position_target]),
        #     np.array([joint_velocity_target]),
        #     joint_indices=np.array([self._joint_index]),
        # )
        # self._articulation.apply_action(action)
