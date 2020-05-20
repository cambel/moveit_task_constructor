/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein, Artur Istvan Karoly
   Desc:   Task definition for serving it as a ROS action
*/

#include <moveit_task_constructor_demo/pick_place_task_for_action.h>

namespace pick_place {
constexpr char LOGNAME[] = "pick_place_task";
PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true) {}

void PickPlaceTask::init(const moveit_task_constructor_msgs::PickPlacePlanningGoalConstPtr& goal) {
	ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");

	/************************
	 Initialize parameters:
	************************/

	// Constants:
	hand_open_pose_ = "open";
	hand_close_pose_ = "close";
	arm_home_pose_ = "home";

	place_surface_offset_ = 0.0001;

	// From ROS action goal:
    arm_group_name_ = goal->robot;
    object_name_ = goal->object_name;
	assembly_name_ = goal->assembly_name;
    place_pose_ = goal->object_target_pose;
    object_subframe_to_place_ = goal->object_frame_to_place;

	surface_link_ = goal->surface_link;
	support_surfaces_ = { surface_link_ };

	approach_object_min_dist_ = goal->approach_object_min_dist;
	approach_object_max_dist_ = goal->approach_object_max_dist;
	lift_object_min_dist_ = goal->lift_object_min_dist;
	lift_object_max_dist_ = goal->lift_object_max_dist;

    // Derived parameters:
    moveit::planning_interface::MoveGroupInterface group(arm_group_name_);
    hand_group_name_ = arm_group_name_ + "_robotiq_85";
    eef_name_ = arm_group_name_ + "_tip";
    hand_frame_ = hand_group_name_ + "_tip_link";
    world_frame_ = "world";

	const std::string object = object_name_;

	/************************
	 Create MTC task:
	************************/

	// Reset ROS introspection before constructing the new object
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel();

	// Sampling planner
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(1.0);
	cartesian_planner->setMaxAccelerationScaling(1.0);
	cartesian_planner->setStepSize(.01);

	// Set task properties
	t.setProperty("group", arm_group_name_);
	t.setProperty("eef", eef_name_);
	t.setProperty("hand", hand_group_name_);
	t.setProperty("hand_grasping_frame", hand_frame_);
	t.setProperty("ik_frame", hand_frame_);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	Stage* current_state = nullptr;  // Forward current_state on to grasp pose generator
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});

		current_state = applicability_filter.get();
		t.add(std::move(applicability_filter));
	}

	/****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup(hand_group_name_);
		stage->setGoal(hand_open_pose_);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/
	Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
	{
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

		/****************************************************
  ---- *               Approach Object                    *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", hand_frame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

			// Set hand forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_frame_;
			vec.vector.x = 0.3;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
			// Sample grasp pose
			auto stage = std::make_unique<stages::LoadGraspPose>("load grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->setPreGraspPose(hand_open_pose_);
			stage->setAssembly(assembly_name_);
			stage->setObject(object);
			stage->setMonitoredStage(current_state);  // Hook into current state

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(hand_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			grasp->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Close Hand                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
			// stage->properties().property("group").configureInitFrom(Stage::PARENT, hand_group_name_);
            stage->properties().set("group", hand_group_name_);
			stage->setGoal(hand_close_pose_);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, hand_frame_);
			attach_object_stage = stage.get();
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Allow collision (object support)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, support_surfaces_, true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Lift object                        *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
			stage->setIKFrame(hand_frame_);
			stage->properties().set("marker_ns", "lift_object");

			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Forbid collision (object support)  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ object }, support_surfaces_, false);
			grasp->insert(std::move(stage));
		}

		// Add grasp container to task
		t.add(std::move(grasp));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Place Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto place = std::make_unique<SerialContainer>("place object");
		t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
		place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

		/******************************************************
  ---- *          Lower Object                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
			stage->properties().set("marker_ns", "lower_object");
			stage->properties().set("link", hand_frame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.03, .13);

			// Set downward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Generate Place Pose                       *
		 *****************************************************/
		{
			// auto stage = std::make_unique<stages::GeneratePlacePoseSubframe>("generate place pose");
			auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
			stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
			stage->properties().set("marker_ns", "place_pose");
			// stage->properties().set("subframe", object_subframe_to_place_);
			stage->setObject(object);
			stage->setSubframe(object_subframe_to_place_);


			// geometry_msgs::PoseStamped p;
			// p.header.frame_id = "move_group/base/screw_hole_panel2_1";
			// p.header.frame_id = "workspace_center";   // example
			// p.pose = place_pose_;
			// p.pose.position.x -= place_surface_offset_;
            place_pose_.pose.position.x -= place_surface_offset_;
			stage->setPose(place_pose_);
			stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(2);
			wrapper->setIKFrame(hand_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		/******************************************************
  ---- *          Open Hand                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
			// stage->properties().property("group").configureInitFrom(Stage::PARENT, hand_group_name_);
            stage->properties().set("group", hand_group_name_);
			stage->setGoal(hand_open_pose_);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Forbid collision (hand, object)        *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
			stage->allowCollisions(
			    object_name_,
			    t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), false);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Detach Object                             *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object_name_, hand_frame_);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Retreat Motion                            *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.05, .1);
			stage->setIKFrame(hand_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_frame_;
			vec.vector.x = -0.3;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		// Add place container to task
		t.add(std::move(place));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Home                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal(arm_home_pose_);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}
}

bool PickPlaceTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	ros::NodeHandle pnh("~");
	int planning_attempts = pnh.param<int>("planning_attempts", 10);

	try {
		task_->plan(planning_attempts);
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}
	if (task_->numSolutions() == 0) {
		ROS_ERROR_NAMED(LOGNAME, "Planning failed");
		return false;
	}
	return true;
}

bool PickPlaceTask::execute() {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	task_->solutions().front()->fillMessage(execute_goal.solution);
	execute_.sendGoal(execute_goal);
	execute_.waitForResult();
	moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
		return false;
	}

	return true;
}

moveit_task_constructor_msgs::Solution PickPlaceTask::getSolution(){
	moveit_task_constructor_msgs::Solution solution;
	if (task_->numSolutions() != 0) {
		task_->solutions().front()->fillMessage(solution);
	}
	return solution;
}

}
