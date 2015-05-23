#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <urdf_model/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "constrained_path_generator/PlanConstrainedPath.h"

class ConstrainedPathGenerator
{
protected:

    planning_scene::PlanningScenePtr planning_scene_ptr_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_ptr_;
    ros::NodeHandle nh_;
    ros::ServiceClient planning_scene_client_;
    ros::ServiceServer constrained_path_server_;
    double minimum_position_tolerance_;
    double minimum_angle_tolerance_;

public:

    ConstrainedPathGenerator(ros::NodeHandle& nh, ros::NodeHandle& nhp, const std::string& planning_scene_service, const std::string& constrained_path_service, const double minimum_angle_tolerance, const double minimum_position_tolerance) : nh_(nh)
    {
        minimum_angle_tolerance_ = fabs(minimum_angle_tolerance);
        minimum_position_tolerance_ = fabs(minimum_position_tolerance);
        robot_model_loader::RobotModelLoader loader("robot_description", true);
        robot_model::RobotModelPtr robot_model = loader.getModel();
        planning_scene_ptr_ = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));
        planning_pipeline_ptr_ = planning_pipeline::PlanningPipelinePtr(new planning_pipeline::PlanningPipeline(robot_model, nhp, "planning_plugin", "request_adapters"));
        planning_scene_client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_service);
        ROS_INFO("Waiting for planning scene update service to be available...");
        planning_scene_client_.waitForExistence();
        ROS_INFO("...planning scene update service ready");
        constrained_path_server_ = nh.advertiseService(constrained_path_service, &ConstrainedPathGenerator::PlanConstrainedPathCB, this);
    }

    void Loop()
    {
        ros::Rate spin_rate(10.0);
        while (ros::ok())
        {
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    bool UpdateInternalPlanningScene()
    {
        ROS_INFO("Updating planning scene...");
        try
        {
            // Update the planning scene
            moveit_msgs::GetPlanningSceneRequest ps_req;
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES
                                         | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
                                         | moveit_msgs::PlanningSceneComponents::OCTOMAP
                                         | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
                                         | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
            moveit_msgs::GetPlanningSceneResponse ps_res;
            planning_scene_client_.call(ps_req, ps_res);
            moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
            planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
            ROS_INFO("...planning scene updated");
            return true;
        }
        catch (...)
        {
            ROS_ERROR("Failed updating planning scene");
            return false;
        }
    }

    inline Eigen::Affine3d Interpolate(const Eigen::Affine3d& t1, const Eigen::Affine3d& t2, const double ratio) const
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Interpolate
        Eigen::Vector3d v1 = t1.translation();
        Eigen::Quaterniond q1(t1.rotation());
        Eigen::Vector3d v2 = t2.translation();
        Eigen::Quaterniond q2(t2.rotation());
        Eigen::Vector3d vint = Interpolate(v1, v2, real_ratio);
        Eigen::Quaterniond qint = Interpolate(q1, q2, real_ratio);
        Eigen::Affine3d tint = (Eigen::Translation3d)vint * qint;
        return tint;
    }

    inline Eigen::Quaterniond Interpolate(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const double ratio) const
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Interpolate
        return q1.slerp(real_ratio, q2);
    }

    inline Eigen::Vector3d Interpolate(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const double ratio) const
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Interpolate
        Eigen::Vector3d interp_vector = v2 - v1;
        return v1 + (interp_vector * real_ratio);
    }

    inline double Distance(const Eigen::Affine3d& t1, const Eigen::Affine3d& t2) const
    {
        Eigen::Vector3d v1 = t1.translation();
        Eigen::Quaterniond q1(t1.rotation());
        Eigen::Vector3d v2 = t2.translation();
        Eigen::Quaterniond q2(t2.rotation());
        double vdist = Distance(v1, v2);
        double qdist = Distance(q1, q2);
        return vdist + qdist;
    }

    inline double Distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const
    {
        return (v2 - v1).norm();
    }

    inline double Distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) const
    {
        double dq = fabs((q1.w() * q2.w()) + (q1.x() * q2.x()) + (q1.y() * q2.y()) + (q1.z() * q2.z()));
        if (dq < (1.0 - std::numeric_limits<double>::epsilon()))
        {
            return acos(2.0 * (dq * dq) - 1.0);
        }
        else
        {
            return 0.0;
        }
    }

    inline double Distance(const std::vector<double>& p1, const std::vector<double>& p2) const
    {
        if (p1.size() == p2.size())
        {
            double distance = 0.0;
            for (size_t idx = 0; idx < p1.size(); idx++)
            {
                distance += (p2[idx] - p1[idx]) * (p2[idx] - p1[idx]);
            }
            return sqrt(distance);
        }
        else
        {
            return INFINITY;
        }
    }

    inline double GetLength(const std::vector<Eigen::Affine3d>& waypoints) const
    {
        if (waypoints.size() < 2)
        {
            return 0.0;
        }
        else
        {
            double distance = 0.0;
            for (size_t idx = 1; idx < waypoints.size(); idx++)
            {
                const Eigen::Affine3d& prev = waypoints[idx - 1];
                const Eigen::Affine3d& cur = waypoints[idx];
                double dist = Distance(prev, cur);
                distance += dist;
            }
            return distance;
        }
    }

    inline std::pair<double, std::vector<double>> GetLengthAndDistances(const std::vector<Eigen::Affine3d>& waypoints) const
    {
        if (waypoints.size() < 2)
        {
            return std::pair<double, std::vector<double>>(0.0, {});
        }
        else
        {
            double distance = 0.0;
            std::vector<double> distances(waypoints.size());
            distances[0] = 0.0;
            for (size_t idx = 1; idx < waypoints.size(); idx++)
            {
                const Eigen::Affine3d& prev = waypoints[idx - 1];
                const Eigen::Affine3d& cur = waypoints[idx];
                double dist = Distance(prev, cur);
                distance += dist;
                distances[idx] = distance;
            }
            return std::pair<double, std::vector<double>>(distance, distances);
        }
    }

    std::vector<Eigen::Affine3d> InterpolatePosePath(const std::vector<Eigen::Affine3d>& waypoints, const double step) const
    {
        if (waypoints.size() == 0)
        {
            std::cerr << "Can't interpolate with no waypoints provided" << std::endl;
            return waypoints;
        }
        else if (waypoints.size() == 1)
        {
            std::cerr << "Can't interpolate with one waypoints provided" << std::endl;
            return waypoints;
        }
        else
        {
            // First, compute the real length of the path
            std::pair<double, std::vector<double>> length_query = GetLengthAndDistances(waypoints);
            double path_length = length_query.first;
            const std::vector<double>& distances = length_query.second;
            if (path_length < std::numeric_limits<double>::epsilon())
            {
                std::cerr << "Path has zero length, cannot meaningfully interpolate" << std::endl;
                return waypoints;
            }
            u_int32_t num_steps = (u_int32_t)ceil(path_length / step);
            double step_size = path_length / (double)num_steps;
            // Step along the path
            std::vector<Eigen::Affine3d> interpolated_path;
            u_int32_t step_num = 0;
            size_t current_path_index = 0;
            while (step_num < num_steps)
            {
                double current_elapsed_distance = distances[current_path_index];
                double target_elapsed_distance = (double)step_num * step_size;
                // We need to interpolate between the previous state and the current state
                if (target_elapsed_distance < current_elapsed_distance)
                {
                    const Eigen::Affine3d& previous = waypoints[current_path_index - 1];
                    const Eigen::Affine3d& current = waypoints[current_path_index];
                    double previous_elapsed_distance = distances[current_path_index - 1];
                    double current_window_distance = current_elapsed_distance - previous_elapsed_distance;
                    double overlap_distance = target_elapsed_distance - previous_elapsed_distance;
                    double percent_overlap = overlap_distance / current_window_distance;
                    Eigen::Affine3d interpolated = Interpolate(previous, current, percent_overlap);
                    interpolated_path.push_back(interpolated);
                    step_num++;
                }
                // We need to skip ahead
                else if (target_elapsed_distance > current_elapsed_distance)
                {
                    current_path_index++;
                }
                // Exactly where we want to be
                else
                {
                    interpolated_path.push_back(waypoints[current_path_index]);
                    step_num++;
                }
            }
            return interpolated_path;
        }
    }

    inline Eigen::Affine3d GeometryPoseToEigenAffine3d(const geometry_msgs::Pose& pose) const
    {
        Eigen::Translation3d trans(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Affine3d transform = trans * quat;
        return transform;
    }

    inline geometry_msgs::Pose EigenAffine3dToGeometryPose(const Eigen::Affine3d& transform) const
    {
        Eigen::Vector3d trans = transform.translation();
        Eigen::Quaterniond quat(transform.rotation());
        geometry_msgs::Pose geom_pose;
        geom_pose.position.x = trans.x();
        geom_pose.position.y = trans.y();
        geom_pose.position.z = trans.z();
        geom_pose.orientation.w = quat.w();
        geom_pose.orientation.x = quat.x();
        geom_pose.orientation.y = quat.y();
        geom_pose.orientation.z = quat.z();
        return geom_pose;
    }

    std::vector<Eigen::Affine3d> InterpolatePosePath(const std::vector<geometry_msgs::PoseStamped>& pose_waypoints, const double step) const
    {
        std::vector<Eigen::Affine3d> waypoints(pose_waypoints.size());
        for (size_t idx = 0; idx < pose_waypoints.size(); idx++)
        {
            waypoints[idx] = GeometryPoseToEigenAffine3d(pose_waypoints[idx].pose);
        }
        return InterpolatePosePath(waypoints, step);
    }

    inline bool CheckForCspaceJump(const trajectory_msgs::JointTrajectory& traj, const double jump_threshold) const
    {
        if (traj.points.size() < 2)
        {
            return false;
        }
        else
        {
            for (size_t idx = 1; idx < traj.points.size(); idx++)
            {
                const trajectory_msgs::JointTrajectoryPoint& previous = traj.points[idx - 1];
                const trajectory_msgs::JointTrajectoryPoint& current = traj.points[idx];
                double distance = Distance(previous.positions, current.positions);
                if (distance > jump_threshold)
                {
                    std::cerr << "Configuration jump detected at state " << idx + 1 << " of " << traj.points.size() << std::endl;
                    return true;
                }
            }
            return false;
        }
    }

    std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes> AttemptIKCartesianPath(const moveit_msgs::RobotState& initial_state, const std::vector<geometry_msgs::PoseStamped>& waypoints, const std::string& group_name, const std::string& link_name, const double task_space_step, const double max_cspace_jump, const bool check_environment_collisions)
    {
        // First, some error checking
        moveit_msgs::MoveItErrorCodes error_code;
        error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        // Now, interpolate a pose path
        std::string reference_frame = waypoints[waypoints.size() - 1].header.frame_id;
        // First, get the current pose
        planning_scene_ptr_->setCurrentState(initial_state);
        geometry_msgs::PoseStamped initial_waypoint;
        initial_waypoint.pose = EigenAffine3dToGeometryPose(planning_scene_ptr_->getFrameTransform(link_name));
        initial_waypoint.header.frame_id = reference_frame;
        // Prepend it to the list of waypoints
        std::vector<geometry_msgs::PoseStamped> real_waypoints;
        real_waypoints.push_back(initial_waypoint);
        real_waypoints.insert(real_waypoints.end(), waypoints.begin(), waypoints.end());
        std::vector<Eigen::Affine3d> pose_path = InterpolatePosePath(real_waypoints, task_space_step);
        // Now, perform IK on each state
        robot_state::RobotState& robot_state = planning_scene_ptr_->getCurrentStateNonConst();
        const moveit::core::JointModelGroup* group = robot_state.getJointModelGroup(group_name);
        collision_detection::CollisionRequest col_req;
        collision_detection::CollisionResult col_res;
        robot_trajectory::RobotTrajectory robot_traj(planning_scene_ptr_->getRobotModel(), group_name);
        for (size_t idx = 0; idx < pose_path.size(); idx++)
        {
            bool success = robot_state.setFromIK(group, pose_path[idx], link_name, 10, 0.01);
            if (success)
            {
                col_res.clear();
                if (check_environment_collisions)
                {
                    planning_scene_ptr_->checkCollision(col_req, col_res);
                    if (col_res.collision)
                    {
                        ROS_ERROR("IK solution for pose %zu of %zu in collision", idx + 1, pose_path.size());
                        error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
                        break;
                    }
                    else
                    {
                        robot_traj.addSuffixWayPoint(robot_state, 0.0);
                    }
                }
                else
                {
                    planning_scene_ptr_->checkSelfCollision(col_req, col_res);
                    if (col_res.collision)
                    {
                        ROS_ERROR("IK solution for pose %zu of %zu in self collision", idx + 1, pose_path.size());
                        error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
                        break;
                    }
                    else
                    {
                        robot_traj.addSuffixWayPoint(robot_state, 0.0);
                    }
                }
            }
            else
            {
                ROS_ERROR("Failed to compute IK for pose %zu of %zu", idx + 1, pose_path.size());
                error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
                break;
            }
        }
        // Check if we've made it here successfully
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            // Retime it
            trajectory_processing::IterativeParabolicTimeParameterization retimer;
            bool retimed = retimer.computeTimeStamps(robot_traj);
            if (!retimed)
            {
                ROS_ERROR("Retiming generated cartesian path failed");
                error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                return std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes>(trajectory_msgs::JointTrajectory(), error_code);
            }
            trajectory_msgs::JointTrajectory trajectory;
            trajectory.joint_names = group->getJointModelNames();
            size_t num_waypoints = robot_traj.getWayPointCount();
            for (size_t idx = 0; idx < num_waypoints; idx++)
            {
                const robot_state::RobotState& traj_state = robot_traj.getWayPoint(idx);
                trajectory_msgs::JointTrajectoryPoint traj_point;
                traj_point.positions.resize(trajectory.joint_names.size());
                for (size_t jdx = 0; jdx < trajectory.joint_names.size(); jdx++)
                {
                    traj_point.positions[jdx] = *traj_state.getJointPositions(trajectory.joint_names[jdx]);
                }
                if (traj_state.hasVelocities())
                {
                    traj_point.velocities.resize(trajectory.joint_names.size());
                    for (size_t jdx = 0; jdx < trajectory.joint_names.size(); jdx++)
                    {
                        traj_point.velocities[jdx] = *traj_state.getJointVelocities(trajectory.joint_names[jdx]);
                    }
                }
                traj_point.time_from_start = ros::Duration(robot_traj.getWaypointDurationFromStart(idx));
                trajectory.points.push_back(traj_point);
            }
            // Check for c-space jumps
            if (CheckForCspaceJump(trajectory, max_cspace_jump))
            {
                ROS_ERROR("Generating cartesian path via IK failed due to configuration jump");
                error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS;
                return std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes>(trajectory_msgs::JointTrajectory(), error_code);
            }
            ROS_INFO("Generating cartesian path via IK succeeded");
            return std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes>(trajectory, error_code);
        }
        else
        {
            ROS_ERROR("Generating cartesian path via IK failed");
            return std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes>(trajectory_msgs::JointTrajectory(), error_code);
        }
    }

    moveit_msgs::PositionConstraint GenerateWaypointConstraints(const moveit_msgs::RobotState& initial_state, const std::vector<geometry_msgs::PoseStamped>& waypoints, const std::string& link_name, const geometry_msgs::Vector3& path_position_tolerance)
    {
        std::string reference_frame = waypoints[waypoints.size() - 1].header.frame_id;
        // First, get the current pose
        planning_scene_ptr_->setCurrentState(initial_state);
        geometry_msgs::PoseStamped initial_waypoint;
        initial_waypoint.pose = EigenAffine3dToGeometryPose(planning_scene_ptr_->getFrameTransform(link_name));
        initial_waypoint.header.frame_id = reference_frame;
        // Prepend it to the list of waypoints
        std::vector<geometry_msgs::PoseStamped> real_waypoints;
        real_waypoints.push_back(initial_waypoint);
        real_waypoints.insert(real_waypoints.end(), waypoints.begin(), waypoints.end());
        // Interpolate a full pose path between the waypoints
        double step_size = (std::min({path_position_tolerance.x, path_position_tolerance.y, path_position_tolerance.z}) * 0.5);
        std::vector<Eigen::Affine3d> interpolated_pose_path = InterpolatePosePath(real_waypoints, step_size);
        // Extract the waypoint constraints
        geometry_msgs::Vector3 target_point_offset;
        target_point_offset.x = 0.0;
        target_point_offset.y = 0.0;
        target_point_offset.z = 0.0;
        moveit_msgs::PositionConstraint waypoint_constraint;
        waypoint_constraint.header.frame_id = reference_frame;
        waypoint_constraint.link_name = link_name;
        waypoint_constraint.target_point_offset = target_point_offset;
        waypoint_constraint.weight = 1.0;
        // Make the bounding volume
        shape_msgs::SolidPrimitive waypoint_sphere;
        waypoint_sphere.type = shape_msgs::SolidPrimitive::BOX;
        waypoint_sphere.dimensions.push_back(path_position_tolerance.x);
        waypoint_sphere.dimensions.push_back(path_position_tolerance.y);
        waypoint_sphere.dimensions.push_back(path_position_tolerance.z);
        moveit_msgs::BoundingVolume waypoint_constraint_region;
        for (size_t idx = 0; idx < interpolated_pose_path.size(); idx++)
        {
            waypoint_constraint_region.primitive_poses.push_back(EigenAffine3dToGeometryPose(interpolated_pose_path[idx]));
            waypoint_constraint_region.primitives.push_back(waypoint_sphere);
        }
        waypoint_constraint.constraint_region = waypoint_constraint_region;
        return waypoint_constraint;
    }

    std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes> AttemptPlannedCartesianPath(const moveit_msgs::RobotState& initial_state, const std::vector<geometry_msgs::PoseStamped>& waypoints, const std::string& group_name, const std::string& link_name, const geometry_msgs::Vector3& goal_position_tolerance, const geometry_msgs::Vector3& goal_angle_tolerance, const geometry_msgs::Vector3& path_position_tolerance, const geometry_msgs::Vector3& path_angle_tolerance, const geometry_msgs::Quaternion& path_orientation_constraint, const double planning_time, const bool follow_waypoints, const bool follow_orientation)
    {
        // Initial setup
        planning_scene_ptr_->setCurrentState(initial_state);
        robot_state::RobotState& robot_state = planning_scene_ptr_->getCurrentStateNonConst();
        const moveit::core::JointModelGroup* group = robot_state.getJointModelGroup(group_name);
        if (group == nullptr)
        {
            ROS_ERROR("Invalid group name");
            moveit_msgs::MoveItErrorCodes error_code;
            error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            return std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes>(trajectory_msgs::JointTrajectory(), error_code);
        }
        // Prepare the planning request
        planning_interface::MotionPlanRequest planning_req;
        planning_req.group_name = group_name;
        std::vector<double> position_tolerance = {goal_position_tolerance.x, goal_position_tolerance.y, goal_position_tolerance.z};
        std::vector<double> rotation_tolerance = {goal_angle_tolerance.x, goal_angle_tolerance.y, goal_angle_tolerance.z};
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(link_name, waypoints[waypoints.size() - 1], position_tolerance, rotation_tolerance);
        planning_req.goal_constraints.push_back(pose_goal);
        if (follow_waypoints)
        {
            planning_req.path_constraints.position_constraints.push_back(GenerateWaypointConstraints(initial_state, waypoints, link_name, path_position_tolerance));
        }
        if (follow_orientation)
        {
            moveit_msgs::OrientationConstraint orientation_constraint;
            orientation_constraint.orientation = path_orientation_constraint;
            orientation_constraint.header.frame_id = waypoints[waypoints.size() - 1].header.frame_id;
            orientation_constraint.link_name = link_name;
            orientation_constraint.weight = 1.0;
            orientation_constraint.absolute_x_axis_tolerance = fabs(path_angle_tolerance.x);
            orientation_constraint.absolute_y_axis_tolerance = fabs(path_angle_tolerance.y);
            orientation_constraint.absolute_z_axis_tolerance = fabs(path_angle_tolerance.z);
            planning_req.path_constraints.orientation_constraints.push_back(orientation_constraint);
        }
        planning_req.allowed_planning_time = planning_time;
        planning_req.num_planning_attempts = 1;
        planning_req.planner_id = "RRTConnectkConfigDefault";
        // Plan
        planning_interface::MotionPlanResponse planning_res;
        planning_pipeline_ptr_->generatePlan(planning_scene_ptr_, planning_req, planning_res);
        // Extract the results
        if (planning_res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_ERROR("Generating cartesian path via planning failed with code %d", planning_res.error_code_.val);
            return std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes>(trajectory_msgs::JointTrajectory(), planning_res.error_code_);
        }
        else
        {
            trajectory_msgs::JointTrajectory trajectory;
            trajectory.joint_names = group->getJointModelNames();
            size_t num_waypoints = planning_res.trajectory_->getWayPointCount();
            for (size_t idx = 0; idx < num_waypoints; idx++)
            {
                const robot_state::RobotState& traj_state = planning_res.trajectory_->getWayPoint(idx);
                trajectory_msgs::JointTrajectoryPoint traj_point;
                traj_point.positions.resize(trajectory.joint_names.size());
                for (size_t jdx = 0; jdx < trajectory.joint_names.size(); jdx++)
                {
                    traj_point.positions[jdx] = *traj_state.getJointPositions(trajectory.joint_names[jdx]);
                }
                if (traj_state.hasVelocities())
                {
                    traj_point.velocities.resize(trajectory.joint_names.size());
                    for (size_t jdx = 0; jdx < trajectory.joint_names.size(); jdx++)
                    {
                        traj_point.velocities[jdx] = *traj_state.getJointVelocities(trajectory.joint_names[jdx]);
                    }
                }
                traj_point.time_from_start = ros::Duration(planning_res.trajectory_->getWaypointDurationFromStart(idx));
                trajectory.points.push_back(traj_point);
            }
            ROS_INFO("Generating cartesian path via planning succeded");
            return std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes>(trajectory_msgs::JointTrajectory(), planning_res.error_code_);
        }
    }

    bool PlanConstrainedPathCB(constrained_path_generator::PlanConstrainedPath::Request& req, constrained_path_generator::PlanConstrainedPath::Response& res)
    {
        ROS_INFO("Processing plan constrained path service...");
        bool got_planning_scene = UpdateInternalPlanningScene();
        if (!got_planning_scene)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::PLANNING_SCENE_UPDATE_FAILED;
            return true;
        }
        // Check the request
        if (req.query.waypoints.size() < 1)
        {
            ROS_ERROR("At least one waypoint must be provided");
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::INVALID_PARAMETERS;
            return true;
        }
        bool use_ik = false;
        if ((req.query.path_type & constrained_path_generator::PlanConstrainedPathQuery::CARTESIAN_IK) > 0)
        {
            use_ik = true;
            ROS_INFO("Will try to produce path using using IK");
        }
        bool use_planning = false;
        if ((req.query.path_type & constrained_path_generator::PlanConstrainedPathQuery::PLAN) > 0)
        {
            use_planning = true;
            ROS_INFO("Will try to produce path using planning");
        }
        bool check_env_collisions = false;
        if ((req.query.path_type & constrained_path_generator::PlanConstrainedPathQuery::CHECK_ENVIRONMENT_COLLISIONS) > 0)
        {
            check_env_collisions = true;
            ROS_INFO("Checking environment collisions to produce path with IK");
        }
        else
        {
            ROS_WARN("Ignoring environment collisions to produce path with IK");
        }
        bool use_waypoint_constraints = false;
        if ((req.query.path_type & constrained_path_generator::PlanConstrainedPathQuery::FOLLOW_WAYPOINTS) > 0)
        {
            use_waypoint_constraints = true;
            ROS_INFO("Will plan using waypoint constraints");
        }
        bool use_orientation_constraints = false;
        if ((req.query.path_type & constrained_path_generator::PlanConstrainedPathQuery::FOLLOW_ORIENTATION_CONSTRAINTS) > 0)
        {
            use_orientation_constraints = true;
            ROS_INFO("Will plan using orientation constraints");
        }
        // Make sure the request is sensible
        if (!(use_ik || use_planning))
        {
            ROS_ERROR("Either IK OR planning must be selected");
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::INVALID_PARAMETERS;
            return true;
        }
        if (use_planning && use_orientation_constraints)
        {
            if (req.query.path_orientation_constraint.x == 0.0 && req.query.path_orientation_constraint.y == 0.0 && req.query.path_orientation_constraint.z == 0.0 && req.query.path_orientation_constraint.w == 0.0)
            {
                ROS_ERROR("Orientation constraints specified, but invalid quaternion provided");
                res.result.status = constrained_path_generator::PlanConstrainedPathResult::INVALID_PARAMETERS;
                return true;
            }
        }
        // Clean the tolerances
        if (req.query.path_angle_tolerance.x < minimum_angle_tolerance_)
        {
            ROS_WARN("Path angle tolerance (x) below minimum threshold");
            req.query.path_angle_tolerance.x = minimum_angle_tolerance_;
        }
        if (req.query.path_angle_tolerance.y < minimum_angle_tolerance_)
        {
            ROS_WARN("Path angle tolerance (y) below minimum threshold");
            req.query.path_angle_tolerance.y = minimum_angle_tolerance_;
        }
        if (req.query.path_angle_tolerance.z < minimum_angle_tolerance_)
        {
            ROS_WARN("Path angle tolerance (z) below minimum threshold");
            req.query.path_angle_tolerance.z = minimum_angle_tolerance_;
        }
        if (req.query.goal_angle_tolerance.x < minimum_angle_tolerance_)
        {
            ROS_WARN("Goal angle tolerance (x) below minimum threshold");
            req.query.goal_angle_tolerance.x = minimum_angle_tolerance_;
        }
        if (req.query.goal_angle_tolerance.y < minimum_angle_tolerance_)
        {
            ROS_WARN("Goal angle tolerance (y) below minimum threshold");
            req.query.goal_angle_tolerance.y = minimum_angle_tolerance_;
        }
        if (req.query.goal_angle_tolerance.z < minimum_angle_tolerance_)
        {
            ROS_WARN("Goal angle tolerance (z) below minimum threshold");
            req.query.goal_angle_tolerance.z = minimum_angle_tolerance_;
        }
        if (req.query.path_position_tolerance.x < minimum_position_tolerance_)
        {
            ROS_WARN("Path position tolerance (x) below minimum threshold");
            req.query.path_position_tolerance.x = minimum_position_tolerance_;
        }
        if (req.query.path_position_tolerance.y < minimum_position_tolerance_)
        {
            ROS_WARN("Path position tolerance (y) below minimum threshold");
            req.query.path_position_tolerance.y = minimum_position_tolerance_;
        }
        if (req.query.path_position_tolerance.z < minimum_position_tolerance_)
        {
            ROS_WARN("Path position tolerance (z) below minimum threshold");
            req.query.path_position_tolerance.z = minimum_position_tolerance_;
        }
        if (req.query.goal_position_tolerance.x < minimum_position_tolerance_)
        {
            ROS_WARN("Goal position tolerance (x) below minimum threshold");
            req.query.goal_position_tolerance.x = minimum_position_tolerance_;
        }
        if (req.query.goal_position_tolerance.y < minimum_position_tolerance_)
        {
            ROS_WARN("Goal position tolerance (y) below minimum threshold");
            req.query.goal_position_tolerance.y = minimum_position_tolerance_;
        }
        if (req.query.goal_position_tolerance.z < minimum_position_tolerance_)
        {
            ROS_WARN("Goal position tolerance (z) below minimum threshold");
            req.query.goal_position_tolerance.z = minimum_position_tolerance_;
        }
        // Attempt the request
        moveit_msgs::MoveItErrorCodes error_code;
        if (use_ik)
        {
            ROS_INFO("Attempting to produce path using IK...");
            std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes> path_attempt = AttemptIKCartesianPath(req.query.initial_state, req.query.waypoints, req.query.group_name, req.query.target_link, req.query.task_space_step_size, req.query.max_cspace_jump, check_env_collisions);
            if (path_attempt.second.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                ROS_INFO("Path produced using IK, returning path");
                res.result.path = path_attempt.first;
                res.result.status = constrained_path_generator::PlanConstrainedPathResult::SUCCESS;
                return true;
            }
            else
            {
                ROS_ERROR("Producing path using IK failed");
                error_code = path_attempt.second;
            }
        }
        if (use_planning)
        {
            ROS_INFO("Attempting to produce path using planning...");
            std::pair<trajectory_msgs::JointTrajectory, moveit_msgs::MoveItErrorCodes> path_attempt = AttemptPlannedCartesianPath(req.query.initial_state, req.query.waypoints, req.query.group_name, req.query.target_link, req.query.goal_position_tolerance, req.query.goal_angle_tolerance, req.query.path_position_tolerance, req.query.path_angle_tolerance, req.query.path_orientation_constraint, req.query.planning_time, use_waypoint_constraints, use_orientation_constraints);
            if (path_attempt.second.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                ROS_INFO("Path produced using planning, returning path");
                res.result.path = path_attempt.first;
                res.result.status = constrained_path_generator::PlanConstrainedPathResult::SUCCESS;
                return true;
            }
            else
            {
                ROS_ERROR("Producing path using planning failed");
                error_code = path_attempt.second;
            }
        }
        // If we got here, things have failed - time to convert the moveit error code
        if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::INVALID_PARAMETERS;
        }
        else if (error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::CARTESIAN_PATH_FAILED;
        }
        else if (error_code.val == moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::GOAL_INFEASIBLE;
        }
        else if (error_code.val == moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::GOAL_INFEASIBLE;
        }
        else if (error_code.val == moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::GOAL_INFEASIBLE;
        }
        else if (error_code.val == moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::START_INFEASIBLE;
        }
        else if (error_code.val == moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::START_INFEASIBLE;
        }
        else if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::PLANNING_TIME_EXCEEDED;
        }
        else
        {
            res.result.status = constrained_path_generator::PlanConstrainedPathResult::MOVEIT_ERROR;
        }
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "constrained_path_generator");
    ROS_INFO("Starting constrained path generator...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string planning_scene_service;
    std::string constrained_path_service;
    double minimum_angle_tolerance = 0.01;
    double minimum_position_tolerance = 0.01;
    nhp.param(std::string("planning_scene_service"), planning_scene_service, std::string("get_planning_scene"));
    nhp.param(std::string("constrained_path_service"), constrained_path_service, std::string("plan_constrained_path"));
    nhp.param(std::string("minimum_angle_tolerance"), minimum_angle_tolerance, minimum_angle_tolerance);
    nhp.param(std::string("minimum_position_tolerance"), minimum_position_tolerance, minimum_position_tolerance);
    ConstrainedPathGenerator generator(nh, nhp, planning_scene_service, constrained_path_service, minimum_angle_tolerance, minimum_position_tolerance);
    ROS_INFO("...startup complete");
    generator.Loop();
    return 0;
}
