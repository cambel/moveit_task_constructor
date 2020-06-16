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

/* Author: Robert Haschke
   Desc:   Planning a simple sequence of Cartesian motions
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
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/container.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>

namespace pick_hold_module {
using namespace moveit::task_constructor;

class MTC_Modules{
	public:
		MTC_Modules();
		~MTC_Modules() = default;

		void init(moveit::core::RobotModelConstPtr robot_model, const std::vector<std::string>& arm_group_names);

		std::unique_ptr<SerialContainer> Pick_Object(const std::string& object);
		std::unique_ptr<SerialContainer> Lift_Object(const std::string& object);
		std::unique_ptr<SerialContainer> Place_Object(const std::string& object);
		std::unique_ptr<SerialContainer> Release_Object_and_Retreat(const std::string& object);
		std::unique_ptr<SerialContainer> Pick_Place(const std::string& object, const std::string& arm_group_name, bool release_object=true);
		std::unique_ptr<SerialContainer> Pick_Place_with_Regrasp(const std::string& object, const std::string& pick_arm_group_name, const std::string& place_arm_group_name, bool release_object=true);
		std::unique_ptr<Alternatives> Pick_Place_Alternatives(const std::string& object, bool release_object=true);

	private:
		// Planners
		moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner;
		moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner;

		// Planning Groups
		std::vector<std::string> groups;
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
};

MTC_Modules::MTC_Modules(){}

void MTC_Modules::init(moveit::core::RobotModelConstPtr robot_model, const std::vector<std::string>& arm_group_names){
	// Init Cartesian planner
	cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(1.0);
	cartesian_planner->setMaxAccelerationScaling(1.0);
	cartesian_planner->setStepSize(.01);

	// Init sampling planner
	sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Robot setup
	groups = arm_group_names;

	for (std::string group : groups){
		std::string hand_group_name = group + "_robotiq_85";
		std::string hand_frame = hand_group_name + "_tip_link";

		hand_group_names.push_back(hand_group_name);
		hand_frames.push_back(hand_frame);
	}

	group = groups[0];
	hand_group_name = hand_group_names[0];
	hand_frame = hand_frames[0];

	robot_model_ = robot_model;

	// Internal stage pointer for hooks
	attach_object_stage = nullptr;
}

std::unique_ptr<SerialContainer> MTC_Modules::Pick_Object(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Pick Object");

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
		// stage->properties().configureInitFrom(Stage::PARENT, { "group" });
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
		stage->setAssembly("wrs_assembly_1");  //  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PARAM
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

std::unique_ptr<SerialContainer> MTC_Modules::Lift_Object(const std::string& object){
	lift_object_stage = nullptr;
	auto c = std::make_unique<SerialContainer>("Lift Object");

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
		vec.header.frame_id = "world";                 // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!DEFINE
		vec.vector.z = 1.0;
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

std::unique_ptr<SerialContainer> MTC_Modules::Place_Object(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Place Object");

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
		vec.header.frame_id = "world";                    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		vec.vector.z = -1.0;
		stage->setDirection(vec);
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
		stage->setSubframe("panel_bearing/bottom_screw_hole_aligner_1");   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// stage->setSubframe(object_subframe_to_place_);

		geometry_msgs::PoseStamped target_pose;
		target_pose.header.frame_id = "move_group/base/screw_hole_panel2_1";
		target_pose.pose.orientation.w = 1.0;
		target_pose.pose.position.x = -0.0001;

        //place_pose_.pose.position.x -= place_surface_offset_;            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// stage->setPose(place_pose_);                                //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

std::unique_ptr<SerialContainer> MTC_Modules::Release_Object_and_Retreat(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Release Object and Retreat");

	// /******************************************************
	//  *                                                    *
	//  *                  Open Hand                         *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->properties().set("group", hand_group_name);
		stage->setGoal("open");                                        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

std::unique_ptr<SerialContainer> MTC_Modules::Pick_Place(const std::string& object, const std::string& arm_group_name, bool release_object){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	auto c = std::make_unique<SerialContainer>("Pick-Place " + group);

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
	c->insert(std::move(MTC_Modules::Pick_Object(object)));

	// Lift
	c->insert(std::move(MTC_Modules::Lift_Object(object)));

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
	c->insert(std::move(MTC_Modules::Place_Object(object)));

	if (release_object){
		// Release and retreat
		c->insert(std::move(MTC_Modules::Release_Object_and_Retreat(object)));
	}

	return c;

}

std::unique_ptr<SerialContainer> MTC_Modules::Pick_Place_with_Regrasp(const std::string& object, const std::string& pick_arm_group_name, const std::string& place_arm_group_name, bool release_object){
	auto c = std::make_unique<SerialContainer>("Pick-Place with Regrasp");

	group = pick_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

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
	c->insert(std::move(MTC_Modules::Pick_Object(object)));

	// Lift
	c->insert(std::move(MTC_Modules::Lift_Object(object)));



	geometry_msgs::PoseStamped handover_pose;                          // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DATA MEMBER?
	handover_pose.header.frame_id = "workspace_center";
	handover_pose.pose.orientation.w = 1.0;
	handover_pose.pose.position.z = 0.5;


	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to handover", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("handover pose");
		//stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setPose(handover_pose);
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

	// /****************************************************
	//  *                                                  *
	//  *           Move to Handover Pose                  *
	//  *                                                  *
	//  ***************************************************/
	// {
	// 	auto stage = std::make_unique<stages::MoveTo>("move to handover", sampling_planner);
	// 	stage->setGroup(pick_arm_group_name);
	// 	stage->setGoal(handover_pose);
	// 	stage->setIKFrame(hand_frame);
	// 	current_state_stage = stage.get();
	// 	c->insert(std::move(stage));
	// }

	group = place_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	// Pick
	c->insert(std::move(MTC_Modules::Pick_Object(object)));

	group = pick_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	c->insert(std::move(MTC_Modules::Release_Object_and_Retreat(object)));

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

	c->insert(std::move(MTC_Modules::Place_Object(object)));

	// if (release_object){
	// 	// Release and retreat
	// 	c->insert(std::move(MTC_Modules::Release_Object_and_Retreat(object)));
	// }

	return c;
}

std::unique_ptr<Alternatives> MTC_Modules::Pick_Place_Alternatives(const std::string& object, bool release_object){
	auto parallel = std::make_unique<Alternatives>("Pick-Place");



	parallel->insert(std::move(MTC_Modules::Pick_Place(object, "a_bot", release_object)));
	parallel->insert(std::move(MTC_Modules::Pick_Place(object, "b_bot", release_object)));

	return parallel;
}

}

moveit::task_constructor::Task createTask() {
	moveit::task_constructor::Task t;
	t.stages()->setName("Task");
	t.loadRobotModel();

	const std::string object = "panel_bearing";

	std::vector<std::string> groups;
	groups.push_back("b_bot");
	groups.push_back("a_bot");

	auto robot_model = t.getRobotModel();

	pick_hold_module::MTC_Modules Modules;
	Modules.init(robot_model, groups);


	// t.add(Modules.Pick_Place_Alternatives(object, false));
	t.add(Modules.Pick_Place_with_Regrasp(object, "a_bot", "b_bot", false));

	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	auto task = createTask();
	try {
		if (task.plan())
			task.introspection().publishSolution(*task.solutions().front());
	} catch (const moveit::task_constructor::InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
