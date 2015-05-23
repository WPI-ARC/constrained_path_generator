#!/usr/bin/python

import math
import rospy
import random
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from constrained_path_generator.msg import *
from constrained_path_generator.srv import *


def make_pose((px, py, pz), (rx, ry, rz, rw)):
    new_pose = Pose()
    new_pose.position.x = px
    new_pose.position.y = py
    new_pose.position.z = pz
    new_pose.orientation.x = rx
    new_pose.orientation.y = ry
    new_pose.orientation.z = rz
    new_pose.orientation.w = rw
    return new_pose


def make_pose_stamped((px, py, pz), (rx, ry, rz, rw), frame):
    pose_stamped = PoseStamped()
    pose_stamped.pose = make_pose((px, py, pz), (rx, ry, rz, rw))
    pose_stamped.header.frame_id = frame
    return pose_stamped


def make_quaternion(w, x, y, z):
    new_quat = Quaternion()
    new_quat.w = w
    new_quat.x = x
    new_quat.y = y
    new_quat.z = z
    return new_quat


def make_vector(x, y, z):
    new_vector = Vector3()
    new_vector.x = x
    new_vector.y = y
    new_vector.z = z
    return new_vector


_joint_state = None


def joint_state_cb(msg):
    global _joint_state
    _joint_state = msg


def test():
    test_node = rospy.init_node("test_planner")
    js_sub = rospy.Subscriber("joint_states", JointState, joint_state_cb)
    planner_client = rospy.ServiceProxy("plan_constrained_path", PlanConstrainedPath)
    # Wait for a joint state
    while _joint_state is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
    print "got robot state"
    # Make the waypoints
    pose_1 = make_pose_stamped((0.585, 0.15, 1.250), (0.0, 0.888, 0.0, -0.460), "base_link")
    waypoints = [pose_1]
    # Make the request
    query = PlanConstrainedPathQuery()
    query.path_type = PlanConstrainedPathQuery.CHECK_ENVIRONMENT_COLLISIONS | PlanConstrainedPathQuery.CARTESIAN_IK | PlanConstrainedPathQuery.PLAN | PlanConstrainedPathQuery.FOLLOW_WAYPOINTS | PlanConstrainedPathQuery.FOLLOW_ORIENTATION_CONSTRAINTS
    query.waypoints = waypoints
    query.group_name = "left_arm"
    query.target_link = "l_wrist_roll_link"
    query.planning_time = 5.0
    query.max_cspace_jump = 0.05
    query.task_space_step_size = 0.025
    query.initial_state.joint_state = _joint_state
    query.path_orientation_constraint = make_quaternion(0.0, 0.888, 0.0, -0.460)
    query.path_angle_tolerance = make_vector(0.01, 0.01, 0.01)
    query.path_position_tolerance = make_vector(0.02, 0.02, 0.02)
    query.goal_angle_tolerance = make_vector(0.01, 0.01, 0.01)
    query.goal_position_tolerance = make_vector(0.01, 0.01, 0.01)
    full_req = PlanConstrainedPathRequest()
    full_req.query = query
    full_res = planner_client.call(full_req)
    print full_res
    # Make some collision_planes
    raw_input("Press ENTER to close...")
    print "Done"


if __name__ == '__main__':
    test()