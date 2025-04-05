import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleXFormPrim
# from isaacsim.core.prims.articulation import Articulation
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.dynamic_control import _dynamic_control
from isaacsim.sensors.physics import IMUSensor
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, euler_angles_to_quats
from isaacsim.core.prims import Articulation, RigidPrim

my_world = World(stage_units_in_meters=1.0, physics_dt = 0.005, rendering_dt = 0.002)
my_world.scene.add_default_ground_plane()

# enable ROS bridge extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

ACTION_DIM = 10
# joint_cmd = np.array([0, 0, -0.2, 0.25, 0, 0, 0, -0.2, 0.25, 0])
joint_cmd = np.array([0, -1.712, 1.712, 0, 0, 0])
kps = np.array([800, 800, 800, 800, 800, 800])
kds = np.array([40, 40, 40, 40, 40, 40])
def joint_cmd_callback(msg):
    # For general, the velocity field is used as pds gains
    global joint_cmd, kps, kds, ACTION_DIM
    joint_cmd = np.array(msg.position)
    kps = np.array(msg.velocity[:ACTION_DIM])
    kds = np.array(msg.velocity[ACTION_DIM:])

# ROS2 node and publishers and subscribers
rclpy.init()
node = rclpy.create_node('UR10_HWITL')
pub_joint_state = node.create_publisher(JointState, '/joint_feedback', 10)
pub_ee_pose = node.create_publisher(Pose, '/ee_pose', 10)
sub_joint_feedback = node.create_subscription(JointState, '/joint_state_cmds', joint_cmd_callback, 10)
rate = node.create_rate(50, node.get_clock())

# Spawn ur10, set initial pose, initialize controller
ur10_asset_path = "ur10.usd"
add_reference_to_stage(usd_path=ur10_asset_path, prim_path="/World/ur10")
ur10: Robot = my_world.scene.add(Robot(prim_path="/World/ur10", name="my_ur10"))
# ur10.set_world_pose(position=np.array([0.0, 0.0, 0.0]) / get_stage_units()
#                         , orientation=euler_angles_to_quats(np.array([0, 0, 0])))

ur10_articulation = Articulation(prim_paths_expr="/World/ur10/root_joint")
# humanoid_articulation.set_joint_positions(np.array([0.0, 0.0, 0.0, 0.0, -0.2, -0.2, 0.25, 0.25, 0.0, 0.0]))
# humanoid_articulation.set_friction_coefficients(np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]))

my_world.reset()
ur10.get_articulation_controller().set_gains(kps, kds)
joint_states = JointState()

# Add end effector
ee_prim_path = "/World/ur10/ee_link"
end_effector:RigidPrim  = my_world.scene.add(RigidPrim(prim_paths_expr=ee_prim_path, name="my_ur10_ee"))

reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False

        current_time = node.get_clock().now().to_msg()
     
        # Apply joint commands
        ur10.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=joint_cmd)
        )
        
        # Joint states feedback
        joint_states.header.stamp = current_time
        joint_states.position = ur10.get_joint_positions().tolist()
        joint_states.velocity = ur10.get_joint_velocities().tolist()
        pub_joint_state.publish(joint_states)

        ee_pos_real = end_effector.get_world_poses()[0].flatten().tolist()
        ee_orn_real = end_effector.get_world_poses()[1].flatten().tolist()

        # publish pose command
        ee_pose_real = Pose()
        ee_pose_real.position.x = ee_pos_real[0]
        ee_pose_real.position.y = ee_pos_real[1]
        ee_pose_real.position.z = ee_pos_real[2]
        ee_pose_real.orientation.x = ee_orn_real[1]
        ee_pose_real.orientation.y = ee_orn_real[2]
        ee_pose_real.orientation.z = ee_orn_real[3]
        ee_pose_real.orientation.w = ee_orn_real[0]
        pub_ee_pose.publish(ee_pose_real)

        # spin ROS2 node
        rclpy.spin_once(node, timeout_sec=0.0)

simulation_app.close()
