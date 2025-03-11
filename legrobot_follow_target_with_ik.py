# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
from isaacsim.core.api import World

from tasks import LegFollowTarget, LegKinematicsSolver

import numpy as np

leg_usd_path = "/home/humanoid/DanielWorkspace/IsaacSimDemo/leg00.usd"
my_world = World(stage_units_in_meters=1.0)
my_task = LegFollowTarget(
    name="leg_follow_target_task",
    target_prim_path=None,
    target_name=None,
    target_position=None,
    target_orientation=None,
    offset=None,
    leg_prim_path="/World/LegRobot",
    leg_robot_name="leg_robot",
    ee_prim_path="/World/LegRobot/L_toe",
    leg_usd_path=leg_usd_path,
)
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("leg_follow_target_task").get_params()
leg_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_leg = my_world.scene.get_object(leg_name)
my_controller = LegKinematicsSolver(my_leg)
articulation_controller = my_leg.get_articulation_controller()
articulation_controller.set_gains(np.array([1000.0] * 10), np.array([10] * 10))
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions, succ = my_controller.compute_inverse_kinematics(
            target_position=observations[target_name]["position"],
            target_orientation=observations[target_name]["orientation"],
        )
        if succ:
            articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

simulation_app.close()
