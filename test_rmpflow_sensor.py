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

import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.universal_robots.controllers.rmpflow_controller import RMPFlowController
from isaacsim.robot.manipulators.examples.universal_robots.tasks import FollowTarget
from isaacsim.core.prims import RigidPrim, SingleXFormPrim
from isaacsim.core.api.objects import cuboid
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_stage_units

# enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")

simulation_app.update()
my_world = World(stage_units_in_meters=1.0, physics_dt=1/60)

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# joint_cmd = np.array([0, -1.57, 0, 0, 0, 0])
# def joint_states_callback(msg):
#     # print("Received joint states: ", msg.position)
#     global joint_cmd
#     joint_cmd = np.array(msg.position)

has_sensor_data = False
sensor_data = np.array([])
detected_objects = ([])
def sensor_data_callback(msg):
    # print("Received sensor data: ", msg.data)
    global sensor_data
    sensor_data = np.array(msg.data)
    global has_sensor_data
    has_sensor_data = True
    # print("Received sensor data: ", sensor_data)

def polar_to_cartesian(radius, azimuth_angle, elevation_angle):
    """
    Convert polar coordinates to Cartesian coordinates.
    
    Parameters:
    radius (float): The radial distance from the origin.
    azimuth_angle (float): The azimuthal angle in radians.
    elevation_angle (float): The elevation angle in radians.
    
    Returns:
    tuple: A tuple containing the Cartesian coordinates (x, y, z).
    """
    x = radius * np.cos(np.deg2rad(elevation_angle)) * np.cos(np.deg2rad(azimuth_angle))
    y = radius * np.cos(np.deg2rad(elevation_angle)) * np.sin(np.deg2rad(azimuth_angle))
    z = radius * np.sin(np.deg2rad(elevation_angle))
    return (x, y, z)

def sensor2EE(sensor_data_, ee_pos_):
    """
    Convert sensor data to end effector frame.
    
    Parameters:
    sensor_data_ (np.array): The sensor data in the sensor frame.
    ee_pos_ (np.array): The end effector position in the world frame.
    
    Returns:
    np.array: The sensor data in the end effector frame.
    """
    detected_objects_ = ([])
    for i in range(0, len(sensor_data_), 2):
        # if sensor_data_[i] < 1:
            # print("Object detected at: ", sensor_data_[i], sensor_data_[i + 1])
            # print("Sensor pose: ", sensor_pose_)
            # Calculate position of the object in the sensor frame
            # x, y, z = polar_to_cartesian(sensor_data_[i], sensor_data_[i + 1], sensor_data_[i + 2])
            x, y, z = polar_to_cartesian(sensor_data_[i], sensor_data_[i + 1], 0)
            r_s = np.array([[x], [y], [z]]) 

            # Calculate position of the object in the world frame
            R = np.array([
                [-np.sqrt(2)/2,  -np.sqrt(2)/2,  0.0],
                [np.sqrt(2)/2, -np.sqrt(2)/2,  0.0],
                [         0.0,           0.0, 1.0] 
            ])
            r_w = R @ r_s + ee_pos_.transpose()
            # Append the detected object to the list
            detected_objects_.append((r_w[0][0], r_w[1][0], r_w[2][0]))
        # else: continue # Ignore the sensor data if the object is too far
    return detected_objects_

def CircleTarget(dt):
    r = 0.3
    w = 0.5
    target_pos = init_pos + np.array([r*np.sin(w * dt), r*np.cos(w * dt), 0])
    return target_pos

def PickPlace(_point_num):
    pt1 = np.array([0.0, 0.7, 0.7])
    pt2 = np.array([0.0, 0.7, 0.3])
    pt3 = np.array([0.7, 0.0, 0.7])
    pt4 = np.array([0.7, 0.0, 0.3])
    if _point_num == 0:
        _target_position = pt1
    elif _point_num == 1:
        _target_position = pt2
    elif _point_num == 2:
        _target_position = pt1
    elif _point_num == 3:
        _target_position = pt3
    elif _point_num == 4:
        _target_position = pt4
    elif _point_num == 5:
        _target_position = pt3
    elif _point_num == 6:
        _target_position = pt1
    return _target_position


rospy.init_node('joint_states_listener', anonymous=True)
# rospy.Subscriber("/joint_cmd", JointState, joint_states_callback)
rospy.Subscriber("/sensor_data", Float32MultiArray, sensor_data_callback)
pub_joint = rospy.Publisher("/joint_states", Float32MultiArray, queue_size=10)

# Task Settings
my_world = World(stage_units_in_meters=1.0, physics_dt=1/60)
my_task = FollowTarget(name="follow_target_task", attach_gripper=True)
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("follow_target_task").get_params()
ur10_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_ur10 = my_world.scene.get_object(ur10_name)
my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_ur10, attach_gripper=False)
articulation_controller = my_ur10.get_articulation_controller()

# Add obstacles
obstacle_list = []
for i in range(0, 4):
    obstacle = cuboid.VisualCuboid(
                f"/World/obstacle_{i}", position=np.array([0.8, 0, 0.3+i*0.5]), color=np.array([0, 1.0, 0]), size=0.02
            )
    obstacle_list.append(obstacle)
    my_controller.add_obstacle(obstacle_list[i])

# Add end effector
ee_prim_path = "/World/UR10/ee_link"
end_effector:RigidPrim  = my_world.scene.add(RigidPrim(prim_paths_expr=ee_prim_path, name="my_ur10_ee"))
has_obstacle = False

target_pos = np.array([0.1, 0.4, 0.7])
my_task._target.set_local_pose(translation=target_pos)

reset_needed = False
count = 0
point_num = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False

        # target_pos = CircleTarget(my_world.get_physics_dt() * count)
        target_pos = PickPlace(point_num)
        my_task._target.set_local_pose(translation=target_pos)

        if has_sensor_data:
            ee_pos = end_effector.get_world_poses()[0]
            detected_objects = sensor2EE(sensor_data, ee_pos)
            # print("Detected objects: ", detected_objects)
            for i in range(0, len(detected_objects)):
                obstacle_list[i].set_world_pose(np.array(detected_objects[i]), np.array([0, 0, 0, 1]))
            has_obstacle = True
            
        else:
            if has_obstacle:
                my_controller.remove_obstacle(obstacle)
                has_obstacle = False
            
        count += 1
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)
        # if np.mean(np.abs(np.array(end_effector.get_world_poses()[0]) - target_pos)) < 0.01:
        if my_task.target_reached():
            point_num += 1
            if point_num > 6:
                point_num = 0
        joint_msgs  = Float32MultiArray()
        joint_msgs.data = actions.joint_positions.tolist()
        pub_joint.publish(joint_msgs)
        

simulation_app.close()
