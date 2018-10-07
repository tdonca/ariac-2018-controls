#include <ros/ros.h>

#include <controller_server/MoveJoints.h>
#include <controller_server/MovePose.h>
#include <controller_server/MoveCartesian.h>
#include <controller_server/ActivateGripper.h>

int main( int argc, char* argv[] ){

	ros::init( argc, argv, "test_node" );
	ros::NodeHandle node;
	ros::AsyncSpinner spinner{2};
	spinner.start();

	ROS_INFO("Test Node is ready.");



	//Test
	ros::ServiceClient cartesian_srv = node.serviceClient<controller_server::MoveCartesian>( "controller_server/move_cartesian" );
	ros::ServiceClient joints_srv = node.serviceClient<controller_server::MoveJoints>( "controller_server/move_joints" );
	ros::ServiceClient pose_srv = node.serviceClient<controller_server::MovePose>( "controller_server/move_pose" );
	ros::ServiceClient gripper_srv = node.serviceClient<controller_server::ActivateGripper>( "controller_server/activate_gripper" );
	controller_server::MoveCartesian c_srv;
	controller_server::MoveJoints j_srv;
	controller_server::MovePose p_srv;
	controller_server::ActivateGripper g_srv;
	std::vector<std::string> jn = { "linear_arm_actuator_joint", "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
	std::vector<double> box = {-0.07,           0.00,            0.91,            0.00,         -1.81,           0.00,           0.21,             0.00};
	std::vector<double> facebin4 = {0.60, -2.11, -1.40, -1.52, -2.06, -1.58, 1.44, 0.06};
	geometry_msgs::Pose box_p;
	box_p.position.x = 0.465;
	box_p.position.y = 0.930;
	box_p.position.z = 0.827; 
	box_p.orientation.x = 0.000;
	box_p.orientation.y = 0.995;
	box_p.orientation.z = 0.000; 
	box_p.orientation.w = 0.098;


	while(getchar()) {
		
		ROS_INFO("Tasks \n1: joints \n2: motion \n3: cartesian \n4: gripper");
		int i;
		std::string task_name;
		std::cin>>i;



		if( i == 1){
			j_srv.request.joint_names = jn;
			j_srv.request.joint_values = box;
			if( joints_srv.call(j_srv) ){
				if( j_srv.response.success ){
					ROS_INFO("Joints path executed successfully.");
				}
				else{
					ROS_ERROR("Fail: %s", j_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call MoveJoints.");
			}
		}



		else if( i == 2){
			p_srv.request.goal_pose = box_p;
			if( pose_srv.call(p_srv) ){
				if( p_srv.response.success ){
					ROS_INFO("Motion path executed successfully.");
				}
				else{
					ROS_ERROR("Fail: %s", p_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call MovePose.");
			}

		}



		else if( i == 3){
			geometry_msgs::Pose cp = box_p;
			cp.position.z += 0.2;
			cp.position.x -= 0.1;
			c_srv.request.waypoints = std::vector<geometry_msgs::Pose>{cp};
			if( cartesian_srv.call(c_srv) ){
				if( c_srv.response.success ){
					ROS_INFO("Cartesian path executed successfully.");
				}
				else{
					ROS_ERROR("Fail: %s", c_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call MoveCartesian.");
			}
		}



		else if( i == 4){
			g_srv.request.enable = true;
			if( gripper_srv.call(g_srv) ){
				if( g_srv.response.success ){
					ROS_INFO("Gripper action executed successfully.");
				}
				else{
					ROS_ERROR("Fail: %s", g_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call ActivateGripper.");
			}
		}

	}

	


	ros::waitForShutdown();
	return 0;
}
