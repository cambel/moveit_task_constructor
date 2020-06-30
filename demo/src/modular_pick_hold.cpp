/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
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

/* Author: Artur Istvan Karoly
   Desc:   Class for creating pick-place/pick-hold tasks and their building blocks (pick object, lift object, place object, release object) in a modular fashion and 
	provide these planning capabilities as ros actions.
*/

#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/load_grasp_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/dummy.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/generate_handover_pose.h>
#include <moveit/task_constructor/container.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_task_constructor_msgs/PickObjectAction.h>
#include <moveit_task_constructor_msgs/PlaceObjectAction.h>
#include <moveit_task_constructor_msgs/PickPlaceWithRegraspAction.h>

#include <actionlib/server/simple_action_server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace mtc_modules {
constexpr char LOGNAME[] = "mtc_modules";
using namespace moveit::task_constructor;

enum RobotStatus{free, holding_object, holding_tool};

struct RobotInfo{
	std::string arm_group_name;
	std::string hand_group_name;

	std::string eef_name;

	std::string hand_frame;

	RobotStatus robot_status;
};

class Modules_Planner{
	public:

		Modules_Planner(): pick_planning_server(Modules_Planner::nh, "pick_planning", boost::bind(&Modules_Planner::pick_planning_server_cb, this, _1), false),
		place_planning_server(Modules_Planner::nh, "place_planning", boost::bind(&Modules_Planner::place_planning_server_cb, this, _1), false),
		pick_place_planning_server(Modules_Planner::nh, "pick_place_planning", boost::bind(&Modules_Planner::pick_place_planning_server_cb, this, _1), false),
		fastening_planning_server(Modules_Planner::nh, "fastening_planning", boost::bind(&Modules_Planner::fastening_planning_server_cb, this, _1), false)
    	{
			pick_place_planning_server.start();
        	pick_planning_server.start();
			place_planning_server.start();
			fastening_planning_server.start();
    	}

		void init();

		std::unique_ptr<SerialContainer> Pick_Object(const std::string& object);
		std::unique_ptr<SerialContainer> Lift_Object(const std::string& object);
		std::unique_ptr<SerialContainer> Pick_and_Lift(const std::string& object, const std::string& arm_group_name);
		std::unique_ptr<SerialContainer> Place_Object(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Release_Object_and_Retreat(const std::string& object);
		std::unique_ptr<SerialContainer> Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object=true, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Fasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Pick_Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object=true, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Pick_Place_with_Regrasp(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& pick_arm_group_name, const std::string& place_arm_group_name, bool release_object=true, const std::string& object_subframe_to_place="");
		std::unique_ptr<Alternatives> Pick_and_Lift_Alternatives(const std::string& object);
		std::unique_ptr<Alternatives> Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");
		std::unique_ptr<Alternatives> Fasten_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place="");
		std::unique_ptr<Alternatives> Pick_Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");
		std::unique_ptr<Fallbacks> Pick_Place_Fallback(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");

		void createPickPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");
		void createPick(const std::string& object);
		void createPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");
		void createFasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place="");

		void pick_planning_server_cb(const moveit_task_constructor_msgs::PickObjectGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_grasp_parameter_location = grasp_parameter_location;
			std::string temp_lift_direction_reference_frame = lift_direction_reference_frame;
			std::vector<double> temp_lift_direction = lift_direction;

			if (goal->grasp_parameter_location != ""){
				grasp_parameter_location = goal->grasp_parameter_location;
			}

			if (goal->lift_direction_reference_frame != "" && !goal->lift_direction.empty()){
				lift_direction_reference_frame = goal->lift_direction_reference_frame;
				lift_direction = goal->lift_direction;
			}

			createPick(goal->object_name);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
					// task.introspection().publishSolution(*task.solutions().front());
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			pick_result.success = success;
			pick_result.solution = sol;

			grasp_parameter_location = temp_grasp_parameter_location;
			lift_direction_reference_frame = temp_lift_direction_reference_frame;
			lift_direction = temp_lift_direction;
			pick_planning_server.setSucceeded(pick_result);
		}

		void place_planning_server_cb(const moveit_task_constructor_msgs::PlaceObjectGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			createPlace(goal->object_name, goal->object_target_pose, goal->release_object_after_place, goal->object_subframe_to_place);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
					// task.introspection().publishSolution(*task.solutions().front());
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			place_result.success = success;
			place_result.solution = sol;

			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			place_planning_server.setSucceeded(place_result);
		}

		void fastening_planning_server_cb(const moveit_task_constructor_msgs::PlaceObjectGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			createFasten(goal->object_name, goal->object_target_pose, goal->object_subframe_to_place);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
					// task.introspection().publishSolution(*task.solutions().front());
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			place_result.success = success;
			place_result.solution = sol;

			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			fastening_planning_server.setSucceeded(place_result);
		}

		void pick_place_planning_server_cb(const moveit_task_constructor_msgs::PickPlaceWithRegraspGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_grasp_parameter_location = grasp_parameter_location;
			std::string temp_lift_direction_reference_frame = lift_direction_reference_frame;
			std::vector<double> temp_lift_direction = lift_direction;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;

			if (goal->grasp_parameter_location != ""){
				grasp_parameter_location = goal->grasp_parameter_location;
			}

			if (goal->lift_direction_reference_frame != "" && !goal->lift_direction.empty()){
				lift_direction_reference_frame = goal->lift_direction_reference_frame;
				lift_direction = goal->lift_direction;
			}

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			createPickPlace(goal->object_name, goal->object_target_pose, goal->release_object_after_place, goal->object_subframe_to_place);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
					// task.introspection().publishSolution(*task.solutions().front());
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			pick_place_result.success = success;
			pick_place_result.solution = sol;

			grasp_parameter_location = temp_grasp_parameter_location;
			lift_direction_reference_frame = temp_lift_direction_reference_frame;
			lift_direction = temp_lift_direction;
			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			pick_place_planning_server.setSucceeded(pick_place_result);
		}

	private:
		ros::NodeHandle nh;
		actionlib::SimpleActionServer<moveit_task_constructor_msgs::PickPlaceWithRegraspAction> pick_place_planning_server;
		actionlib::SimpleActionServer<moveit_task_constructor_msgs::PickObjectAction> pick_planning_server;
		actionlib::SimpleActionServer<moveit_task_constructor_msgs::PlaceObjectAction> place_planning_server;
		actionlib::SimpleActionServer<moveit_task_constructor_msgs::PlaceObjectAction> fastening_planning_server;
		moveit_task_constructor_msgs::PickPlaceWithRegraspResult pick_place_result;
		moveit_task_constructor_msgs::PickObjectResult pick_result;
		moveit_task_constructor_msgs::PlaceObjectResult place_result;

		moveit::task_constructor::TaskPtr task_;

		// Planners
		moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner;
		moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner;

		// Planning Groups
		std::vector<std::string> arm_group_names;
		std::vector<std::string> hand_group_names;

		std::string group;
		std::string hand_group_name;

		// Frames
		std::vector<std::string> hand_frames;

		std::string hand_frame;

		// Robot Model
		moveit::core::RobotModelConstPtr robot_model_;

		// Internal stage pointer for hooks
		Stage* current_state_stage;
		Stage* attach_object_stage;
		Stage* lift_object_stage;

		// Robots
		std::vector<RobotInfo> robots_info;

		// Grasp parameter namespace
		std::string grasp_parameter_location;

		// Lifting object
		std::string lift_direction_reference_frame;
		std::vector<double> lift_direction;

		// Placing object
		std::string approach_place_direction_reference_frame;
		std::vector<double> approach_place_direction;
};

void Modules_Planner::init(){

	// Load requred params from the param server
	ROS_INFO_NAMED(LOGNAME, "Initializing Modules Planner");
	ros::NodeHandle pnh("~");

	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_names", arm_group_names);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_names", hand_group_names);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_parameter_location", grasp_parameter_location);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_direction_reference_frame", lift_direction_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_direction", lift_direction);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_place_direction_reference_frame", approach_place_direction_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_place_direction", approach_place_direction);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	// Init Cartesian planner
	cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(1.0);
	cartesian_planner->setMaxAccelerationScaling(1.0);
	cartesian_planner->setStepSize(.01);

	// Init sampling planner
	sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	for (std::size_t n = 0; n < std::min( arm_group_names.size(), hand_group_names.size() ); n++)
	{
		RobotInfo robotinfo;
		robotinfo.arm_group_name = arm_group_names[n];
		robotinfo.hand_group_name = hand_group_names[n];

		robotinfo.eef_name = arm_group_names[n] + "_tip";

		robotinfo.hand_frame = hand_group_names[n] + "_tip_link";

		robotinfo.robot_status = free;
	}

	for (std::string group : arm_group_names){
		std::string hand_group_name = group + "_robotiq_85";
		std::string hand_frame = hand_group_name + "_tip_link";

		hand_group_names.push_back(hand_group_name);
		hand_frames.push_back(hand_frame);
	}

	group = arm_group_names[0];
	hand_group_name = hand_group_names[0];
	hand_frame = hand_frames[0];

	// Internal stage pointer for hooks
	attach_object_stage = nullptr;
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_Object(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Pick Object " + group);

	/****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup(hand_group_name);
		stage->setGoal("open");
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Approach object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
		stage->properties().set("marker_ns", "approach_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(0.1, 0.15);
		// stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);     !!!!!!!!!!!!!!!!!!!!!!!!!!!! PARAMS

		// Set hand forward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = hand_frame;
		vec.vector.x = 0.3;
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Generate Grasp Pose                *
	 *                                                  *
	 ***************************************************/
	{
		// Load grasp pose
		auto stage = std::make_unique<stages::LoadGraspPose>("load grasp pose");
		// stage->properties().configureInitFrom(Stage::PARENT);
		stage->properties().set("marker_ns", "grasp_pose");
		stage->setPreGraspPose("open");
		stage->setAssembly(grasp_parameter_location);
		stage->setObject(object);
		stage->setMonitoredStage(current_state_stage);  // Hook into current state
		stage->setEndEffector(group + "_tip");  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Define above

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(8);
		wrapper->setMinSolutionDistance(1.0);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		// wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}

	/****************************************************
	 *                                                  *
	 *          Allow Collision (hand object)           *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
		stage->allowCollisions(
			object, robot_model_->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
			true);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *                  Close Hand                      *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
        stage->properties().set("group", hand_group_name);
		stage->setGoal("close");
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Lift_Object(const std::string& object){
	lift_object_stage = nullptr;
	auto c = std::make_unique<SerialContainer>("Lift Object " + group);

	/****************************************************
	 *                                                  *
	 *                 Attach Object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject(object, hand_frame);
		attach_object_stage = stage.get();
		c->insert(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *       Allow collision (object support)           *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
		stage->allowCollisions({ object }, { "tray_center" }, true);                       // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PARAM
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *                  Lift Object                     *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		//stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->properties().set("group", group);
		stage->setMinMaxDistance(0.1, 0.15);           // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PARAMS
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "lift_object");

		// Set upward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = lift_direction_reference_frame;
		vec.vector.x = lift_direction[0];
		vec.vector.y = lift_direction[1];
		vec.vector.z = lift_direction[2];
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *        Forbid collision (object support)         *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
		stage->allowCollisions({ object }, { "tray_center" }, false);                   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!PARAM
		lift_object_stage = stage.get();
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_and_Lift(const std::string& object, const std::string& arm_group_name){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";
	
	auto c = std::make_unique<SerialContainer>("Pick Object " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	current_state_stage = nullptr;  // Forward current_state on to grasp pose generator
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

		current_state_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	// Pick
	c->insert(std::move(Modules_Planner::Pick_Object(object)));


	// Lift
	c->insert(std::move(Modules_Planner::Lift_Object(object)));

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Place_Object(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place){
	auto c = std::make_unique<SerialContainer>("Place Object " + group);

	/******************************************************
	 *                                                    *
	 *          Lower Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
		stage->properties().set("marker_ns", "lower_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(.03, .13);              //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		// Set downward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = approach_place_direction_reference_frame;
		vec.vector.x = approach_place_direction[0];
		vec.vector.y = approach_place_direction[1];
		vec.vector.z = approach_place_direction[2];
		stage->setDirection(vec);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Generate Place Pose                       *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
		stage->properties().set("marker_ns", "place_pose");
		stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setObject(object);
		// stage->setSubframe("panel_bearing/bottom_screw_hole_aligner_1");   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		stage->setSubframe(object_subframe_to_place);

        //place_pose_.pose.position.x -= place_surface_offset_;            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		stage->setPose(target_pose);
		stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(2);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		// wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}
	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Release_Object_and_Retreat(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Release Object and Retreat " + group);

	// /******************************************************
	//  *                                                    *
	//  *                  Open Hand                         *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->properties().set("group", hand_group_name);
		stage->setGoal("open");
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *         Forbid collision (hand, object)            *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
		stage->allowCollisions(
			object,
			robot_model_->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(), false);
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *                 Detach Object                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object, hand_frame);
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *                Retreat Motion                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
		stage->setMinMaxDistance(.05, .1);                          //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "retreat");
		stage->properties().set("group", group);
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = hand_frame;
		vec.vector.x = -1.0;
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object, const std::string& object_subframe_to_place){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	std::string tem = hand_frame;
	
	auto c = std::make_unique<SerialContainer>("Place Object " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	current_state_stage = nullptr;  // Forward current_state on to grasp pose generator
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached and cannot be placed";
				return false;
			}
		});

		attach_object_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

		if (release_object){
		// Release and retreat
		c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Fasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, const std::string& object_subframe_to_place){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	std::string tem = hand_frame;

	auto c = std::make_unique<SerialContainer>("Fastening " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	current_state_stage = nullptr;  // Forward current_state on to grasp pose generator
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached so fastening cannot be planned";
				return false;
			}
		});

		attach_object_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

	c->insert(std::move(std::make_unique<stages::Dummy>("Fastening")));

	// /******************************************************
	//  *                                                    *
	//  *                Retreat Motion                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat after fastening", cartesian_planner);
		stage->setMinMaxDistance(.05, .1);                          //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "retreat");
		stage->properties().set("group", group);
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = approach_place_direction_reference_frame;
		vec.vector.x = -1*approach_place_direction[0];
		vec.vector.y = -1*approach_place_direction[1];
		vec.vector.z = -1*approach_place_direction[2];
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object, const std::string& object_subframe_to_place){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	auto c = std::make_unique<SerialContainer>("Pick-Place " + group);

	// Pick and Lift
	c->insert(std::move(Modules_Planner::Pick_and_Lift(object, group)));

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

	if (release_object){
		// Release and retreat
		c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_Place_with_Regrasp(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& pick_arm_group_name, const std::string& place_arm_group_name, bool release_object, const std::string& object_subframe_to_place){
	auto c = std::make_unique<SerialContainer>("Pick-Place with Regrasp");

	group = pick_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	// Pick and Lift
	c->insert(std::move(Modules_Planner::Pick_and_Lift(object, group)));


	geometry_msgs::PoseStamped handover_pose;                          // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DATA MEMBER?
	handover_pose.header.frame_id = "workspace_center";
	handover_pose.pose.orientation.w = 1.0;
	handover_pose.pose.position.z = 0.25;


	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to handover", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GenerateHandoverPose>("handover pose");
		stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setPose(handover_pose);
		stage->setObject(object);
		stage->setMonitoredStage(lift_object_stage);

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("handover pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(2);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		// wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}

	group = place_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	// Pick
	c->insert(std::move(Modules_Planner::Pick_Object(object)));

	group = pick_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));

	group = place_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	/****************************************************
	 *                                                  *
	 *                 Attach Object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject(object, hand_frame);
		attach_object_stage = stage.get();
		c->insert(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

	if (release_object){
		// Release and retreat
		c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));
	}

	return c;
}

std::unique_ptr<Alternatives> Modules_Planner::Pick_and_Lift_Alternatives(const std::string& object){
	auto parallel = std::make_unique<Alternatives>("Pick and Lift");

	parallel->insert(std::move(Modules_Planner::Pick_and_Lift(object, "a_bot")));
	parallel->insert(std::move(Modules_Planner::Pick_and_Lift(object, "b_bot")));

	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place){
	auto parallel = std::make_unique<Alternatives>("Place");

	parallel->insert(std::move(Modules_Planner::Place(object, target_pose, "a_bot", release_object, object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Place(object, target_pose, "b_bot", release_object, object_subframe_to_place)));

	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Fasten_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place){
	auto parallel = std::make_unique<Alternatives>("Fasten");

	parallel->insert(std::move(Modules_Planner::Fasten(object, target_pose, "a_bot", object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Fasten(object, target_pose, "b_bot", object_subframe_to_place)));

	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Pick_Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place){
	auto parallel = std::make_unique<Alternatives>("Pick-Place");

	parallel->insert(std::move(Modules_Planner::Pick_Place(object, target_pose, "a_bot", release_object, object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Pick_Place(object, target_pose, "b_bot", release_object, object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, "a_bot", "b_bot", release_object, object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, "b_bot", "a_bot", release_object, object_subframe_to_place)));

	return parallel;
}

std::unique_ptr<Fallbacks> Modules_Planner::Pick_Place_Fallback(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place){
	auto parallel = std::make_unique<Fallbacks>("Pick-Place");

	auto single_robot_task_solutions = std::make_unique<Alternatives>("Pick-Place single robot");
	single_robot_task_solutions->insert(std::move(Modules_Planner::Pick_Place(object, target_pose, "a_bot", release_object, object_subframe_to_place)));
	single_robot_task_solutions->insert(std::move(Modules_Planner::Pick_Place(object, target_pose, "b_bot", release_object, object_subframe_to_place)));
	
	auto regrasp_task_solutions = std::make_unique<Alternatives>("Pick-Place with regrasp");
	regrasp_task_solutions->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, "a_bot", "b_bot", release_object, object_subframe_to_place)));
	regrasp_task_solutions->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, "b_bot", "a_bot", release_object, object_subframe_to_place)));

	parallel->insert(std::move(single_robot_task_solutions));
	parallel->insert(std::move(regrasp_task_solutions));

	return parallel;
}

void Modules_Planner::createPickPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	// t.add(Modules_Planner::Pick_Place_Alternatives(object, target_pose, false, object_subframe_to_place));
	t.add(Modules_Planner::Pick_Place_Fallback(object, target_pose, release_object, object_subframe_to_place));
}

void Modules_Planner::createPick(const std::string& object) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Pick_and_Lift_Alternatives(object));
}

void Modules_Planner::createPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Place_Alternatives(object, target_pose, release_object, object_subframe_to_place));
}

void Modules_Planner::createFasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Fasten_Alternatives(object, target_pose, object_subframe_to_place));
}

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_modules");

	mtc_modules::Modules_Planner Modules;
	Modules.init();

	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
