#include <moveit/task_constructor/stages/generate_place_pose_subframe.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/attached_body.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit {
namespace task_constructor {
namespace stages {

GeneratePlacePoseSubframe::GeneratePlacePoseSubframe(const std::string& name) :  GeneratePlacePose(name) {}

void GeneratePlacePoseSubframe::onNewSolution(const SolutionBase& s) {
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	std::string msg;
	if (!scene->getCurrentState().hasAttachedBody(object))
		msg = "'" + object + "' is not an attached object";
	if (scene->getCurrentState().getAttachedBody(object)->getFixedTransforms().empty())
		msg = "'" + object + "' has no associated shapes";
	if (!msg.empty()) {
		if (storeFailures()) {
			InterfaceState state(scene);
			SubTrajectory solution;
			solution.markAsFailure();
			solution.setComment(msg);
			spawn(std::move(state), std::move(solution));
		} else
			ROS_WARN_STREAM_NAMED("GeneratePlacePose", msg);
		return;
	}

	upstream_solutions_.push(&s);
}

void GeneratePlacePoseSubframe::compute(){
	if (upstream_solutions_.empty())
		return;

	const SolutionBase& s = *upstream_solutions_.pop();
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene()->diff();
	const moveit::core::RobotState& robot_state = scene->getCurrentState();
	const auto& props = properties();

	const moveit::core::AttachedBody* object = robot_state.getAttachedBody(props.get<std::string>("object"));
	// current object_pose w.r.t. planning frame
	const Eigen::Isometry3d& orig_object_pose = object->getGlobalCollisionBodyTransforms()[0];

	std::string attach_link_name = object->getAttachedLink()->getName();
	const Eigen::Isometry3d& subframe_pose_relative_to_attach_link = object->getSubframeTransform(props.get<std::string>("subframe")); // CHECK IF SUBFRAME EXISTS!!!!!!!!!!
	const Eigen::Isometry3d& subframe_pose_global = robot_state.getGlobalLinkTransform(attach_link_name) * subframe_pose_relative_to_attach_link;
	// std::cout << orig_object_pose.matrix() << std::endl;

	const geometry_msgs::PoseStamped& pose_msg = props.get<geometry_msgs::PoseStamped>("pose");
	Eigen::Isometry3d target_pose;
	tf::poseMsgToEigen(pose_msg.pose, target_pose);
	// target pose w.r.t. planning frame
	scene->getTransforms().transformPose(pose_msg.header.frame_id, target_pose, target_pose);

	const geometry_msgs::PoseStamped& ik_frame_msg = props.get<geometry_msgs::PoseStamped>("ik_frame");
	Eigen::Isometry3d ik_frame;
	tf::poseMsgToEigen(ik_frame_msg.pose, ik_frame);
	ik_frame = robot_state.getGlobalLinkTransform(ik_frame_msg.header.frame_id) * ik_frame;
	Eigen::Isometry3d object_to_ik = subframe_pose_global.inverse() * ik_frame;
	//Eigen::Isometry3d object_to_ik = orig_object_pose.inverse() * ik_frame;

	// spawn the nominal target object pose, considering flip about z and rotations about z-axis
	auto spawner = [&s, &scene, &object_to_ik, this](const Eigen::Isometry3d& nominal, uint z_flips,
	                                                 uint z_rotations = 10) {
		for (uint flip = 0; flip < z_flips; ++flip) {
			// flip about object's x-axis
			Eigen::Isometry3d object = nominal * Eigen::AngleAxisd(flip * M_PI, Eigen::Vector3d::UnitX());
			for (uint i = 0; i < z_rotations; ++i) {
				// rotate object at target pose about world's z-axis
				Eigen::Vector3d pos = object.translation();
				object.pretranslate(-pos)
				    .prerotate(Eigen::AngleAxisd(i * 2. * M_PI / z_rotations, Eigen::Vector3d::UnitZ()))
				    .pretranslate(pos);

				// target ik_frame's pose w.r.t. planning frame
				geometry_msgs::PoseStamped target_pose_msg;
				target_pose_msg.header.frame_id = scene->getPlanningFrame();
				tf::poseEigenToMsg(object * object_to_ik, target_pose_msg.pose);

				InterfaceState state(scene);
				forwardProperties(*s.end(), state);  // forward properties from inner solutions
				state.properties().set("target_pose", target_pose_msg);

				SubTrajectory trajectory;
				trajectory.setCost(0.0);
				rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "place frame");

				spawn(std::move(state), std::move(trajectory));
			}
		}
	};

	if (object->getShapes().size() == 1) {
		switch (object->getShapes()[0]->type) {
			case shapes::CYLINDER:
				spawner(target_pose, 2);
				return;

			case shapes::BOX: {  // consider 180/90 degree rotations about z axis
				const double* dims = static_cast<const shapes::Box&>(*object->getShapes()[0]).size;
				spawner(target_pose, 2, (std::abs(dims[0] - dims[1]) < 1e-5) ? 4 : 2);
				return;
			}
			case shapes::SPHERE:  // keep original orientation and rotate about world's z
				target_pose.linear() = orig_object_pose.linear();
				spawner(target_pose, 1);
				return;
			default:
				break;
		}
	}

	// any other case: only try given target pose
	spawner(target_pose, 1, 1);
}

}
}
}