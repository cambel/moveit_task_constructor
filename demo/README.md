# moveit_task_constructor_demo

Description: A simple pick & place demo using MoveIt Task Constructor. This uses the Panda from Franka Emika

Developed by Henning Kayser & Simon Goldstein at [PickNik Consulting](http://picknik.ai/)

## Run

Run demo

    roslaunch moveit_task_constructor_demo demo.launch

# o2ac_demo

A demo pick-place planning using the o2ac scene

## Run

Start the o2ac demo scene in a terminal:

`roslaunch o2ac_moveit_config demo.launch`

In an other terminal use the following command to spawn an object:

`roslaunch moveit_task_constructor_demo o2ac_demo.launch spawn:=true`

This node uses the information from `config/o2ac_config.yaml` Specify the `assembly_name`, the `object_name`, the `object_reference_frame` and the `object pose` (x,y,z,r,p,y) to spawn the object.
(First spawn the base plate somwhere in the workspace, then spawn the L plate in the tray).

If both the base and the L plate are in the scene, and the `o2ac_config.yaml` is filled with the info of the object to be grasped call the o2ac_demo launch file again without the spawn argument to plan:

`roslaunch moveit_task_constructor_demo o2ac_demo.launch`


To visualize the plans add the Motion Planning Tasks plugin in RViz and for the Task Solution Topic select /o2ac_demo/solution

If the planning was successful, you should be able to visualize the plans by selecting `pick_place_task` in the Motion Planning Tasks window and then clicking on the individual solutions on the right.