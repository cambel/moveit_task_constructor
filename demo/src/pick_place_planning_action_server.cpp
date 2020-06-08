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


// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task_for_action.h>

#include <geometry_msgs/Pose.h>
#include <moveit_task_constructor_msgs/PickPlacePlanningAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>

constexpr char LOGNAME[] = "pick_place_planning";

class Pick_Place_Planning_Server
{
    public:

    ros::NodeHandle nh;
    ros::NodeHandle snh;
    actionlib::SimpleActionServer<moveit_task_constructor_msgs::PickPlacePlanningAction> server;
    pick_place::PickPlaceTask pick_place_task;
    moveit_task_constructor_msgs::PickPlacePlanningResult result;

    Pick_Place_Planning_Server(): server(Pick_Place_Planning_Server::nh, "pick_place_planning", boost::bind(&Pick_Place_Planning_Server::plan_pick_place_task, this, _1), false), 
    pick_place_task("pick_place_task", Pick_Place_Planning_Server::snh)
    {
        server.start();
    }

    void plan_pick_place_task(const moveit_task_constructor_msgs::PickPlacePlanningGoalConstPtr& goal)
    {
        pick_place_task.init(goal);

        bool success = false;
        moveit_task_constructor_msgs::Solution sol;

        success = pick_place_task.plan();
        if (success) {
			ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
			sol = pick_place_task.getSolution();
        } else {
			ROS_INFO_NAMED(LOGNAME, "Planning failed");
        }

        result.success = success;
        result.solution = sol;

        server.setSucceeded(result);
    }

};

int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init pick_place_planner");
	ros::init(argc, argv, "pick_place_planner");
    Pick_Place_Planning_Server planning_server;


	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::waitForShutdown();
	return 0;
}
