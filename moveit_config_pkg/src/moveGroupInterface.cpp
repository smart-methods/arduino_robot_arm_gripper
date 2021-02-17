#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>
//MOVE IT
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


float positionX = -0.0023734011670250126;
float positionY = 0.026825106858739834;
float positionZ = 0.32402280551708873;

float orientationX = 0.4966690471541269;
float orientationY = -0.4966708043538031;
float orientationZ = -0.5033089303160498;
float orientationW = -0.5033071531040186;

int count = 0;


void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	float new_positionX = msg->position.x;
	float new_positionY = msg->position.y;
	float new_positionZ = msg->position.z;


	if (new_positionX!=positionX )
	{

	ROS_INFO("new pose");

	if (count == 0)
	{
		static const std::string PLANNING_GROUP = "gripper";
		moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
		const moveit::core::JointModelGroup *joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		joint_group_positions[0] = 1.0472; // radians
		move_group.setJointValueTarget(joint_group_positions);

		moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
		bool success = (move_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		move_group.move();
		count++;
	}

	//planning group that we would like to control
	moveit::planning_interface::MoveGroupInterface group("arm");
	
	//we can add or remove collision objects in our virtual world scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	//raw pointers are used to refer to the planning group for improved performance
	//const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");
	group.setEndEffectorLink("arm3");
	group.setPoseReferenceFrame("base");
	group.setPlannerId("base");
	group.setNumPlanningAttempts(30);
	group.setPlanningTime(30.0);
	group.allowReplanning(true);
	group.setGoalJointTolerance(0.0001);
	group.setGoalPositionTolerance(0.0001);
	group.setGoalOrientationTolerance(0.001);
	//group.setNamedTarget("random");
	group.setRandomTarget();
	group.move();  // WORKS FINE :)

	//  CUSTOM PLANNING
	geometry_msgs::Pose target_pose1;
	//NOTE: THIS IS THE VALID POSE FROM RANDOM NODE

	   
	target_pose1.orientation.w = msg->orientation.w;
	target_pose1.orientation.x = msg->orientation.x;
	target_pose1.orientation.y = msg->orientation.y;
	target_pose1.orientation.z = msg->orientation.z;
	
	target_pose1.position.x =  msg->position.x;
	target_pose1.position.y =  msg->position.y;
	target_pose1.position.z =  msg->position.z;

	group.setStartStateToCurrentState();
	group.setPoseTarget(target_pose1,"arm3");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit_msgs::MotionPlanRequest response;

	group.plan(my_plan);
	group.execute(my_plan);
	
	ROS_INFO("pose updated");



	static const std::string PLANNING_GROUP = "gripper";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const moveit::core::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	if (count % 2 == 1)
	{
	joint_group_positions[0] = 0.0; // radians
	joint_group_positions[1] = 0.0; // radians
	}
	else if (count % 2 == 0)
	{
	joint_group_positions[0] = 1.0472; // radians
	joint_group_positions[1] = 1.0472; // radians
	}
	count++;

	move_group.setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
	bool success = (move_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	move_group.move();


	positionX = msg->position.x;
	positionY = msg->position.y;
	positionZ = msg->position.z;
	orientationX = msg->orientation.x;
	orientationY = msg->orientation.y;
	orientationZ = msg->orientation.z;
	orientationW = msg->orientation.w;
	
	}
}



int main(int argc, char **argv)
{

ros::init(argc, argv, "move_group_interface");
ros::NodeHandle nh;

ROS_INFO("Ready to get direction .. use the camera frame");

// define user callback queue
  ros::CallbackQueue string_queue;
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
      "/direction", // topic name
      1000, // queue length
      chatterCallback, // callback
      ros::VoidPtr(), // tracked object, we don't need one thus NULL
      &string_queue // pointer to callback queue object
    );
  // subscribe
  ros::Subscriber sub2 = nh.subscribe(ops);

ros::AsyncSpinner spinner1(1);
spinner1.start();

ros::AsyncSpinner spinner(1, &string_queue);
spinner.start();


ros::Rate r(10); // 10 hz
while (ros::ok())
  {
    // process one message from ROS internal queue
    ros::spinOnce();
    r.sleep();
    
  }

return 0;

}
