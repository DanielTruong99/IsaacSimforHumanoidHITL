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
from isaacsim.core.prims import Articulation

my_world = World(stage_units_in_meters=1.0, physics_dt = 0.005, rendering_dt = 0.002)
my_world.scene.add_default_ground_plane()

# enable ROS bridge extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray

ACTION_DIM = 10
# joint_cmd = np.array([0, 0, -0.2, 0.25, 0, 0, 0, -0.2, 0.25, 0])
joint_cmd = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
kps = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])
kds = np.array([5, 5, 5, 5, 5, 5, 5, 5, 5, 5])
def joint_cmd_callback(msg):
    # For general, the velocity field is used as pds gains
    global joint_cmd, kps, kds, ACTION_DIM
    joint_cmd = np.array(msg.position)
    kps = np.array(msg.velocity[:ACTION_DIM])
    kds = np.array(msg.velocity[ACTION_DIM:])

# ROS2 node and publishers and subscribers
rclpy.init()
node = rclpy.create_node('Humanoid_HWITL')
pub_joint_state = node.create_publisher(JointState, '/joint_feedback', 10)
# pub_body_pose = node.create_publisher(Float32MultiArray, '/body_pose_feedback', 10)
# pub_body_vel = node.create_publisher(Float32MultiArray, '/body_vel_feedback', 10)
sub_joint_feedback = node.create_subscription(JointState, '/joint_cmd', joint_cmd_callback, 10)
pub_imu = node.create_publisher(Imu, '/imu', 10)
rate = node.create_rate(50, node.get_clock())

# Spawn Humanoid, set initial pose, initialize controller
humanoid_asset_path = "usd/leg00/leg00.usd"
add_reference_to_stage(usd_path=humanoid_asset_path, prim_path="/World/Humanoid")
humanoid: Robot = my_world.scene.add(Robot(prim_path="/World/Humanoid", name="my_humanoid"))
humanoid.set_world_pose(position=np.array([0.0, 0.0, 0.91]) / get_stage_units()
                        , orientation=euler_angles_to_quats(np.array([0, 0, 0])))

humanoid_articulation = Articulation(prim_paths_expr="/World/Humanoid/base")
# humanoid_articulation.set_joint_positions(np.array([0.0, 0.0, 0.0, 0.0, -0.2, -0.2, 0.25, 0.25, 0.0, 0.0]))
# humanoid_articulation.set_friction_coefficients(np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]))

imu_sensor: IMUSensor = my_world.scene.add(
    IMUSensor(
        prim_path="/World/Humanoid/base/imu_sensor",
        name="imu",
        # frequency=100,
        translation=np.array([0, 0, 0]),
    )
)


my_world.reset()
humanoid.get_articulation_controller().set_gains(kps, kds)
joint_states = JointState()
# joint_states.name = ["L_hip_joint",
#                      "L_hip2_joint",
#                      "L_thigh_joint", 
#                      "L_calf_joint", 
#                      "L_toe_joint", 
#                      "R_hip_joint", 
#                      "R_hip2_joint", 
#                      "R_thigh_joint", 
#                      "R_calf_joint", 
#                      "R_toe_joint"]

joint_init = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
joint_kneel = np.array([0, 0, 0, 0, -0.5735, -0.5735, 1.147, 1.147, -0.5735, -0.5735])

state = 0
cnt0 = 0
cnt1 = 0
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

        # if state == 0:
        #     joint_cmd = joint_init
        #     cnt0 += 1
        #     if cnt0 > 100:
        #         state = 1
        #         cnt0 = 0
        # elif state == 1:
        #     cnt1 += 1
        #     alpha = cnt1/200
        #     cnt1 = cnt1 if cnt1 < 200 else 200
        #     joint_cmd = joint_init*(1-alpha) + joint_kneel*alpha
        #     if cnt1 == 200:
        #         state = 2


        # Set PD gains
        humanoid.get_articulation_controller().set_gains(kps, kds)
        # print("Joint command: ", joint_cmd)
        # Apply joint commands
        humanoid.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=joint_cmd)
        )
        
        # Joint states feedback
        joint_states.header.stamp = current_time
        joint_states.position = humanoid.get_joint_positions().tolist()
        joint_states.velocity = humanoid.get_joint_velocities().tolist()
        pub_joint_state.publish(joint_states)

        # # Body Pose feedback
        # body_pose_data = Float32MultiArray()
        # body_position = humanoid.get_world_pose()[0].tolist()
        # body_orientation = humanoid.get_world_pose()[1].tolist()
        # body_pose_data.data = body_position + body_orientation

        # # Body Velocity feedback
        # body_vel_data = Float32MultiArray()
        # body_angular_vel = humanoid.get_angular_velocity().tolist()
        # body_vel_data.data = body_linear_vel + body_angular_vel

        # Get IMU data
        imu_data = imu_sensor.get_current_frame()
        # v_w = humanoid.get_linear_velocity()
        # R = quats_to_rot_matrices(humanoid.get_world_pose()[1])
        # v_b = R.T @ v_w; v_b = v_b.tolist()
        w_b = imu_data["ang_vel"].tolist()
        quat = imu_data["orientation"].tolist()
        R = quats_to_rot_matrices(imu_data["orientation"])
        project_gravity = R.T @ np.array([[0], [0], [-1]])
        project_gravity = project_gravity.flatten().tolist()

        # Compose and publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "base"
        imu_msg.angular_velocity.x = w_b[0]
        imu_msg.angular_velocity.y = w_b[1]
        imu_msg.angular_velocity.z = w_b[2]
        # imu_msg.orientation.x = quat[1]
        # imu_msg.orientation.y = quat[2]
        # imu_msg.orientation.z = quat[3]
        # imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = project_gravity[0]
        imu_msg.orientation.y = project_gravity[1]
        imu_msg.orientation.z = project_gravity[2]
        # imu_msg.linear_acceleration.x = v_b[0]
        # imu_msg.linear_acceleration.y = v_b[1]
        # imu_msg.linear_acceleration.z = v_b[2]
        pub_imu.publish(imu_msg)

        rclpy.spin_once(node, timeout_sec=0.0)

simulation_app.close()
