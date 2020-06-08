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

```
roslaunch o2ac_moveit_config demo.launch
```

Make sure the objects necessary for the pick-place operation are available in the scene in the form of collision objects, because in order for the planning to be successful, these objects are expected to be already available before the planner is called. The planner does not spawn the objects!

If the objects are available in the planning scene, call

```
roslaunch moveit_task_constructor_demo o2ac_demo.launch
```

to run the demo. This node uses the information from `config/o2ac_config.yaml` Look at the contents of this file to see the use of the parameters of the demo.

To visualize the plans add the Motion Planning Tasks plugin in RViz and for the Task Solution Topic select /o2ac_demo/solution

If the planning was successful, you should be able to visualize the plans by selecting `pick_place_task` in the Motion Planning Tasks window and then clicking on the individual solutions on the right.

# Pick-place planning action server

An action server providing an action for the motion planning of a pick-place task. The action definition can be found in `msgs/action/PickPlacePlanning.action`

## Usage

First start the o2ac demo scene in a terminal:

```
roslaunch o2ac_moveit_config demo.launch
```

In an other terminal use the spawn_object node from o2ac_routines module to spawn objects:

```
roslaunch o2ac_routines spawn_object.launch
```

After the objects are loaded in the scene, start the action server with:

```
rosrun moveit_task_constructor_demo pick_place_planning_action_server 
```

When the server started, send goals to the `/pick_place_planning` action topic, e.g. with `rosrun actionlib axclient.py /pick_place_planning`. The explanation of the fields can be found in the action definition in `moveit_task_constructor/msgs/action/PickPlacePlanning.action` An example of how to use the action with an action client is in `o2ac_routines/scripts/assembly.py`.

To visualize the plans add the Motion Planning Tasks plugin in RViz and for the Task Solution Topic select /o2ac_demo/solution

If the planning was successful, you should be able to visualize the plans by selecting `pick_place_task` in the Motion Planning Tasks window and then clicking on the individual solutions on the right.

# Grasps

Both the `o2ac_demo` and the `pick-place planning action server` rely on a stage that looks for and loads the possible grasps from the ROS parameter server instead of the GenerateGraspPose stage. The grasps on the parameter server are expected to be in the following format:

```
/assembly_level_namespace/object_level_namespace/grasp_level_namespace/position  # A list of three values ([x,y,z])
/assembly_level_namespace/object_level_namespace/grasp_level_namespace/orientation  # A list of four values (quaternion, [x,y,z,w])
```

Getting the `/assembly_level_namespace/object_level_namespace/grasp_level_namespace` parameter enables the retieval of a single grasp of a single object. The returned value is a dictionary of the form: `{"position":[], "orientation":[]}`

To get all the grasps of a single object the parent parameter can be retrieved, such as `/assembly_level_namespace/object_level_namespace`. This returns a dictionary of dictionaries that contains all the grasps of a single object, like: `{"grasp_1":{"position":[], "orientation":[]}, "grasp_2":{"position":[], "orientation":[]}, ...}`

Retrieving `/assembly_level_namespace` parameter only, returns all of the grasps of all of the objects in this collection.

The `assembly_name` parameter of `o2ac_demo` in `config/o2ac_config.yaml` and the `grasp_parameter_location` field of the `msgs/action/PickPlacePlanning.action` action definition both refer to the `assembly_level_namespace` and are used to retrieve the correct grasps. The `object_name` parameter in `config/o2ac_config.yaml` and in `msgs/action/PickPlacePlanning.action` refer to the `object_level_namespace`. Please note, that the planners expect the grasps to be loaded to the param server and the planning will not be successful if the grasps are not available.