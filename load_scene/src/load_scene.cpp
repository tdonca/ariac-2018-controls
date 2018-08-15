
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>


int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "load_scene");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	//Setup planning scene 
	moveit::planning_interface::PlanningSceneInterface scene;
	
	//Broadcast static TF frames for each bin
	//Pose coordinates obtained from the gear URDF file
	static tf2_ros::StaticTransformBroadcaster static_broadcast;
	geometry_msgs::TransformStamped bin_tf;
	tf2::Quaternion q;
	tf2_ros::Buffer tfBuf;
	tf2_ros::TransformListener tfListener(tfBuf);
	std::vector<moveit_msgs::CollisionObject> c_objs;
	
	
	/***************************
	 * 
	 * 
	 * 		BIN COLLISIONS
	 * 
	 * 
	 * 
	 * *************************/
	
	/* Bin 1 */
	
	//~ bin_tf.header.stamp = ros::Time::now();
	//~ bin_tf.header.frame_id = "world";
	//~ bin_tf.child_frame_id = "bin1";
	//~ q.setRPY(0, -0.25, 3.14159);
	//~ bin_tf.transform.rotation.x = q.x();
	//~ bin_tf.transform.rotation.y = q.y();
	//~ bin_tf.transform.rotation.z = q.z();
	//~ bin_tf.transform.rotation.w = q.w();
	//~ bin_tf.transform.translation.x = -0.85;
	//~ bin_tf.transform.translation.y = -0.49;
	//~ bin_tf.transform.translation.z = 0.96;
	//~ static_broadcast.sendTransform(bin_tf);
	
	// bottom
	geometry_msgs::TransformStamped wall_bottom_tf;
	wall_bottom_tf.header.stamp = ros::Time::now();
	wall_bottom_tf.header.frame_id = "bin1_frame";
	wall_bottom_tf.child_frame_id = "bin1_bottom";
	q.setRPY(0.0, 0.0, 0.0);
	wall_bottom_tf.transform.rotation.x = q.x();
	wall_bottom_tf.transform.rotation.y = q.y();
	wall_bottom_tf.transform.rotation.z = q.z();
	wall_bottom_tf.transform.rotation.w = q.w();
	wall_bottom_tf.transform.translation.x = 0.0;
	wall_bottom_tf.transform.translation.y = -0.007;
	wall_bottom_tf.transform.translation.z = 0.001;
	static_broadcast.sendTransform(wall_bottom_tf);
	
	// front wall
	geometry_msgs::TransformStamped wall_front_tf;
	wall_front_tf.header.stamp = ros::Time::now();
	wall_front_tf.header.frame_id = "bin1_frame";
	wall_front_tf.child_frame_id = "bin1_front";
	q.setRPY(0.0, 0.0, 0.01);
	wall_front_tf.transform.rotation.x = q.x();
	wall_front_tf.transform.rotation.y = q.y();
	wall_front_tf.transform.rotation.z = q.z();
	wall_front_tf.transform.rotation.w = q.w();
	wall_front_tf.transform.translation.x = -0.24;
	wall_front_tf.transform.translation.y = -0.007;
	wall_front_tf.transform.translation.z = 0.042;
	static_broadcast.sendTransform(wall_front_tf);
	
	// back wall
	geometry_msgs::TransformStamped wall_back_tf;
	wall_back_tf.header.stamp = ros::Time::now();
	wall_back_tf.header.frame_id = "bin1_frame";
	wall_back_tf.child_frame_id = "bin1_back";
	q.setRPY(0.0, 0.0, 0.0);
	wall_back_tf.transform.rotation.x = q.x();
	wall_back_tf.transform.rotation.y = q.y();
	wall_back_tf.transform.rotation.z = q.z();
	wall_back_tf.transform.rotation.w = q.w();
	wall_back_tf.transform.translation.x = 0.24;
	wall_back_tf.transform.translation.y = -0.014;
	wall_back_tf.transform.translation.z = 0.075;
	static_broadcast.sendTransform(wall_back_tf);
	
	// left wall
	geometry_msgs::TransformStamped wall_left_tf;
	wall_left_tf.header.stamp = ros::Time::now();
	wall_left_tf.header.frame_id = "bin1_frame";
	wall_left_tf.child_frame_id = "bin1_left";
	q.setRPY(0.0, 0.0, -0.02);
	wall_left_tf.transform.rotation.x = q.x();
	wall_left_tf.transform.rotation.y = q.y();
	wall_left_tf.transform.rotation.z = q.z();
	wall_left_tf.transform.rotation.w = q.w();
	wall_left_tf.transform.translation.x = 0.0;
	wall_left_tf.transform.translation.y = 0.273;
	wall_left_tf.transform.translation.z = 0.075;
	static_broadcast.sendTransform(wall_left_tf);
	
	// right wall
	geometry_msgs::TransformStamped wall_right_tf;
	wall_right_tf.header.stamp = ros::Time::now();
	wall_right_tf.header.frame_id = "bin1_frame";
	wall_right_tf.child_frame_id = "bin1_right";
	q.setRPY(0.0, 0.0, -0.04);
	wall_right_tf.transform.rotation.x = q.x();
	wall_right_tf.transform.rotation.y = q.y();
	wall_right_tf.transform.rotation.z = q.z();
	wall_right_tf.transform.rotation.w = q.w();
	wall_right_tf.transform.translation.x = 0.0;
	wall_right_tf.transform.translation.y = -0.294;
	wall_right_tf.transform.translation.z = 0.075;
	static_broadcast.sendTransform(wall_right_tf);
	
	
	
	/* Bin 2 */
	
	//~ bin_tf.header.stamp = ros::Time::now();
	//~ bin_tf.child_frame_id = "bin2";
	//~ bin_tf.transform.translation.y = 0.32;
	//~ static_broadcast.sendTransform(bin_tf);
	
	//bottom
	wall_bottom_tf.header.stamp = ros::Time::now();
	wall_bottom_tf.header.frame_id = "bin2_frame";
	wall_bottom_tf.child_frame_id = "bin2_bottom";
	static_broadcast.sendTransform(wall_bottom_tf);
	
	//front wall
	wall_front_tf.header.stamp = ros::Time::now();
	wall_front_tf.header.frame_id = "bin2_frame";
	wall_front_tf.child_frame_id = "bin2_front";
	static_broadcast.sendTransform(wall_front_tf);
	
	//back wall
	wall_back_tf.header.stamp = ros::Time::now();
	wall_back_tf.header.frame_id = "bin2_frame";
	wall_back_tf.child_frame_id = "bin2_back";
	static_broadcast.sendTransform(wall_back_tf);
	
	//left wall
	wall_left_tf.header.stamp = ros::Time::now();
	wall_left_tf.header.frame_id = "bin2_frame";
	wall_left_tf.child_frame_id = "bin2_left";
	static_broadcast.sendTransform(wall_left_tf);
	
	//right wall
	wall_right_tf.header.stamp = ros::Time::now();
	wall_right_tf.header.frame_id = "bin2_frame";
	wall_right_tf.child_frame_id = "bin2_right";
	static_broadcast.sendTransform(wall_right_tf);
	
	
	
	/* Bin 3 */
	
	//~ bin_tf.header.stamp = ros::Time::now();
	//~ bin_tf.child_frame_id = "bin3";
	//~ bin_tf.transform.translation.y = 1.13;
	//~ static_broadcast.sendTransform(bin_tf);
	
	//bottom
	wall_bottom_tf.header.stamp = ros::Time::now();
	wall_bottom_tf.header.frame_id = "bin3_frame";
	wall_bottom_tf.child_frame_id = "bin3_bottom";
	static_broadcast.sendTransform(wall_bottom_tf);
	
	//front wall
	wall_front_tf.header.stamp = ros::Time::now();
	wall_front_tf.header.frame_id = "bin3_frame";
	wall_front_tf.child_frame_id = "bin3_front";
	static_broadcast.sendTransform(wall_front_tf);
	
	//back wall
	wall_back_tf.header.stamp = ros::Time::now();
	wall_back_tf.header.frame_id = "bin3_frame";
	wall_back_tf.child_frame_id = "bin3_back";
	static_broadcast.sendTransform(wall_back_tf);
	
	//left wall
	wall_left_tf.header.stamp = ros::Time::now();
	wall_left_tf.header.frame_id = "bin3_frame";
	wall_left_tf.child_frame_id = "bin3_left";
	static_broadcast.sendTransform(wall_left_tf);
	
	//right wall
	wall_right_tf.header.stamp = ros::Time::now();
	wall_right_tf.header.frame_id = "bin3_frame";
	wall_right_tf.child_frame_id = "bin3_right";
	static_broadcast.sendTransform(wall_right_tf);
	
	
	
	/* Bin 4 */
	
	//~ bin_tf.header.stamp = ros::Time::now();
	//~ bin_tf.child_frame_id = "bin4";
	//~ bin_tf.transform.translation.y = 1.94;
	//~ static_broadcast.sendTransform(bin_tf);
	
	//bottom
	wall_bottom_tf.header.stamp = ros::Time::now();
	wall_bottom_tf.header.frame_id = "bin4_frame";
	wall_bottom_tf.child_frame_id = "bin4_bottom";
	static_broadcast.sendTransform(wall_bottom_tf);
	
	//front wall
	wall_front_tf.header.stamp = ros::Time::now();
	wall_front_tf.header.frame_id = "bin4_frame";
	wall_front_tf.child_frame_id = "bin4_front";
	static_broadcast.sendTransform(wall_front_tf);
	
	//back wall
	wall_back_tf.header.stamp = ros::Time::now();
	wall_back_tf.header.frame_id = "bin4_frame";
	wall_back_tf.child_frame_id = "bin4_back";
	static_broadcast.sendTransform(wall_back_tf);
	
	//left wall
	wall_left_tf.header.stamp = ros::Time::now();
	wall_left_tf.header.frame_id = "bin4_frame";
	wall_left_tf.child_frame_id = "bin4_left";
	static_broadcast.sendTransform(wall_left_tf);
	
	//right wall
	wall_right_tf.header.stamp = ros::Time::now();
	wall_right_tf.header.frame_id = "bin4_frame";
	wall_right_tf.child_frame_id = "bin4_right";
	static_broadcast.sendTransform(wall_right_tf);
	
	
	
	
	/* Bin 5 */
	
	//~ bin_tf.header.stamp = ros::Time::now();
	//~ bin_tf.child_frame_id = "bin5";
	//~ bin_tf.transform.translation.y = 2.75;
	//~ static_broadcast.sendTransform(bin_tf);
	
	//bottom
	wall_bottom_tf.header.stamp = ros::Time::now();
	wall_bottom_tf.header.frame_id = "bin5_frame";
	wall_bottom_tf.child_frame_id = "bin5_bottom";
	static_broadcast.sendTransform(wall_bottom_tf);
	
	//front wall
	wall_front_tf.header.stamp = ros::Time::now();
	wall_front_tf.header.frame_id = "bin5_frame";
	wall_front_tf.child_frame_id = "bin5_front";
	static_broadcast.sendTransform(wall_front_tf);
	
	//back wall
	wall_back_tf.header.stamp = ros::Time::now();
	wall_back_tf.header.frame_id = "bin5_frame";
	wall_back_tf.child_frame_id = "bin5_back";
	static_broadcast.sendTransform(wall_back_tf);
	
	//left wall
	wall_left_tf.header.stamp = ros::Time::now();
	wall_left_tf.header.frame_id = "bin5_frame";
	wall_left_tf.child_frame_id = "bin5_left";
	static_broadcast.sendTransform(wall_left_tf);
	
	//right wall
	wall_right_tf.header.stamp = ros::Time::now();
	wall_right_tf.header.frame_id = "bin5_frame";
	wall_right_tf.child_frame_id = "bin5_right";
	static_broadcast.sendTransform(wall_right_tf);
	
	
	
	
	/* TF Frames for Collision */
	
	//Use TF to transform collision box poses for collision msgs
	
	/* Bin 1 collisions */
	
	//world frame transform of bin1 parts
	geometry_msgs::TransformStamped tf_bottom, tf_back, tf_front, tf_left, tf_right; 
	tf_bottom = tfBuf.lookupTransform("world", "bin1_bottom", ros::Time(0), ros::Duration(1.0));
	tf_back   = tfBuf.lookupTransform("world", "bin1_back",   ros::Time(0), ros::Duration(1.0));
	tf_front  = tfBuf.lookupTransform("world", "bin1_front",  ros::Time(0), ros::Duration(1.0));
	tf_left   = tfBuf.lookupTransform("world", "bin1_left",   ros::Time(0), ros::Duration(1.0));
	tf_right  = tfBuf.lookupTransform("world", "bin1_right",  ros::Time(0), ros::Duration(1.0));	
	moveit_msgs::CollisionObject obj;
	
	
	obj.header.frame_id = "world";
	obj.id = "bin1";
	obj.operation = obj.ADD;
	// bottom
	shape_msgs::SolidPrimitive pr;
	pr.type = pr.BOX;
	pr.dimensions.resize(3);
	pr.dimensions[0] = 0.5;
	pr.dimensions[1] = 0.623;
	pr.dimensions[2] = 0.025;
	obj.primitives.push_back(pr);
	geometry_msgs::Pose po;
	po.position.x = tf_bottom.transform.translation.x;
	po.position.y = tf_bottom.transform.translation.y;
	po.position.z = tf_bottom.transform.translation.z;
	po.orientation.x = tf_bottom.transform.rotation.x;
	po.orientation.y = tf_bottom.transform.rotation.y;
	po.orientation.z = tf_bottom.transform.rotation.z;
	po.orientation.w = tf_bottom.transform.rotation.w;
	obj.primitive_poses.push_back(po);
	//back
	pr.dimensions[0] = 0.02;
	pr.dimensions[1] = 0.532;
	pr.dimensions[2] = 0.145;
	obj.primitives.push_back(pr);
	po.position.x = tf_back.transform.translation.x;
	po.position.y = tf_back.transform.translation.y;
	po.position.z = tf_back.transform.translation.z;
	po.orientation.x = tf_back.transform.rotation.x;
	po.orientation.y = tf_back.transform.rotation.y;
	po.orientation.z = tf_back.transform.rotation.z;
	po.orientation.w = tf_back.transform.rotation.w;
	obj.primitive_poses.push_back(po);
	//front
	pr.dimensions[0] = 0.015;
	pr.dimensions[1] = 0.525;
	pr.dimensions[2] = 0.084;
	obj.primitives.push_back(pr);
	po.position.x = tf_front.transform.translation.x;
	po.position.y = tf_front.transform.translation.y;
	po.position.z = tf_front.transform.translation.z;
	po.orientation.x = tf_front.transform.rotation.x;
	po.orientation.y = tf_front.transform.rotation.y;
	po.orientation.z = tf_front.transform.rotation.z;
	po.orientation.w = tf_front.transform.rotation.w;
	obj.primitive_poses.push_back(po);
	//left
	pr.dimensions[0] = 0.5;
	pr.dimensions[1] = 0.05;
	pr.dimensions[2] = 0.145;
	obj.primitives.push_back(pr);
	po.position.x = tf_left.transform.translation.x;
	po.position.y = tf_left.transform.translation.y;
	po.position.z = tf_left.transform.translation.z;
	po.orientation.x = tf_left.transform.rotation.x;
	po.orientation.y = tf_left.transform.rotation.y;
	po.orientation.z = tf_left.transform.rotation.z;
	po.orientation.w = tf_left.transform.rotation.w;
	obj.primitive_poses.push_back(po);
	//right
	pr.dimensions[0] = 0.5;
	pr.dimensions[1] = 0.05;
	pr.dimensions[2] = 0.145;
	obj.primitives.push_back(pr);
	po.position.x = tf_right.transform.translation.x;
	po.position.y = tf_right.transform.translation.y;
	po.position.z = tf_right.transform.translation.z;
	po.orientation.x = tf_right.transform.rotation.x;
	po.orientation.y = tf_right.transform.rotation.y;
	po.orientation.z = tf_right.transform.rotation.z;
	po.orientation.w = tf_right.transform.rotation.w;
	obj.primitive_poses.push_back(po);
	
	c_objs.push_back(obj);
	
	
	
	/* Bin 2 collisions */
	
	tf_bottom = tfBuf.lookupTransform("world", "bin2_bottom", ros::Time(0), ros::Duration(1.0));
	tf_back   = tfBuf.lookupTransform("world", "bin2_back",   ros::Time(0), ros::Duration(1.0));
	tf_front  = tfBuf.lookupTransform("world", "bin2_front",  ros::Time(0), ros::Duration(1.0));
	tf_left   = tfBuf.lookupTransform("world", "bin2_left",   ros::Time(0), ros::Duration(1.0));
	tf_right  = tfBuf.lookupTransform("world", "bin2_right",  ros::Time(0), ros::Duration(1.0));
	
	obj.id = "bin2";
	//bottom
	obj.primitive_poses[0].position.x = tf_bottom.transform.translation.x;
	obj.primitive_poses[0].position.y = tf_bottom.transform.translation.y;
	obj.primitive_poses[0].position.z = tf_bottom.transform.translation.z;
	//back
	obj.primitive_poses[1].position.x = tf_back.transform.translation.x;
	obj.primitive_poses[1].position.y = tf_back.transform.translation.y;
	obj.primitive_poses[1].position.z = tf_back.transform.translation.z;
	//front
	obj.primitive_poses[2].position.x = tf_front.transform.translation.x;
	obj.primitive_poses[2].position.y = tf_front.transform.translation.y;
	obj.primitive_poses[2].position.z = tf_front.transform.translation.z;
	//left
	obj.primitive_poses[3].position.x = tf_left.transform.translation.x;
	obj.primitive_poses[3].position.y = tf_left.transform.translation.y;
	obj.primitive_poses[3].position.z = tf_left.transform.translation.z;
	//right
	obj.primitive_poses[4].position.x = tf_right.transform.translation.x;
	obj.primitive_poses[4].position.y = tf_right.transform.translation.y;
	obj.primitive_poses[4].position.z = tf_right.transform.translation.z;
	
	c_objs.push_back(obj);
	
	
	
	/* Bin 3 collisions */
	
	tf_bottom = tfBuf.lookupTransform("world", "bin3_bottom", ros::Time(0), ros::Duration(1.0));
	tf_back   = tfBuf.lookupTransform("world", "bin3_back",   ros::Time(0), ros::Duration(1.0));
	tf_front  = tfBuf.lookupTransform("world", "bin3_front",  ros::Time(0), ros::Duration(1.0));
	tf_left   = tfBuf.lookupTransform("world", "bin3_left",   ros::Time(0), ros::Duration(1.0));
	tf_right  = tfBuf.lookupTransform("world", "bin3_right",  ros::Time(0), ros::Duration(1.0));
	
	obj.id = "bin3";
	//bottom
	obj.primitive_poses[0].position.x = tf_bottom.transform.translation.x;
	obj.primitive_poses[0].position.y = tf_bottom.transform.translation.y;
	obj.primitive_poses[0].position.z = tf_bottom.transform.translation.z;
	//back
	obj.primitive_poses[1].position.x = tf_back.transform.translation.x;
	obj.primitive_poses[1].position.y = tf_back.transform.translation.y;
	obj.primitive_poses[1].position.z = tf_back.transform.translation.z;
	//front
	obj.primitive_poses[2].position.x = tf_front.transform.translation.x;
	obj.primitive_poses[2].position.y = tf_front.transform.translation.y;
	obj.primitive_poses[2].position.z = tf_front.transform.translation.z;
	//left
	obj.primitive_poses[3].position.x = tf_left.transform.translation.x;
	obj.primitive_poses[3].position.y = tf_left.transform.translation.y;
	obj.primitive_poses[3].position.z = tf_left.transform.translation.z;
	//right
	obj.primitive_poses[4].position.x = tf_right.transform.translation.x;
	obj.primitive_poses[4].position.y = tf_right.transform.translation.y;
	obj.primitive_poses[4].position.z = tf_right.transform.translation.z;
	
	c_objs.push_back(obj);
	
	
	/* Bin 4 collisions*/
	
	tf_bottom = tfBuf.lookupTransform("world", "bin4_bottom", ros::Time(0), ros::Duration(1.0));
	tf_back   = tfBuf.lookupTransform("world", "bin4_back",   ros::Time(0), ros::Duration(1.0));
	tf_front  = tfBuf.lookupTransform("world", "bin4_front",  ros::Time(0), ros::Duration(1.0));
	tf_left   = tfBuf.lookupTransform("world", "bin4_left",   ros::Time(0), ros::Duration(1.0));
	tf_right  = tfBuf.lookupTransform("world", "bin4_right",  ros::Time(0), ros::Duration(1.0));
	
	obj.id = "bin4";
	//bottom
	obj.primitive_poses[0].position.x = tf_bottom.transform.translation.x;
	obj.primitive_poses[0].position.y = tf_bottom.transform.translation.y;
	obj.primitive_poses[0].position.z = tf_bottom.transform.translation.z;
	//back
	obj.primitive_poses[1].position.x = tf_back.transform.translation.x;
	obj.primitive_poses[1].position.y = tf_back.transform.translation.y;
	obj.primitive_poses[1].position.z = tf_back.transform.translation.z;
	//front
	obj.primitive_poses[2].position.x = tf_front.transform.translation.x;
	obj.primitive_poses[2].position.y = tf_front.transform.translation.y;
	obj.primitive_poses[2].position.z = tf_front.transform.translation.z;
	//left
	obj.primitive_poses[3].position.x = tf_left.transform.translation.x;
	obj.primitive_poses[3].position.y = tf_left.transform.translation.y;
	obj.primitive_poses[3].position.z = tf_left.transform.translation.z;
	//right
	obj.primitive_poses[4].position.x = tf_right.transform.translation.x;
	obj.primitive_poses[4].position.y = tf_right.transform.translation.y;
	obj.primitive_poses[4].position.z = tf_right.transform.translation.z;
	
	c_objs.push_back(obj);
	
	
	/*Bin 5 collisions */
	
	tf_bottom = tfBuf.lookupTransform("world", "bin5_bottom", ros::Time(0), ros::Duration(1.0));
	tf_back   = tfBuf.lookupTransform("world", "bin5_back",   ros::Time(0), ros::Duration(1.0));
	tf_front  = tfBuf.lookupTransform("world", "bin5_front",  ros::Time(0), ros::Duration(1.0));
	tf_left   = tfBuf.lookupTransform("world", "bin5_left",   ros::Time(0), ros::Duration(1.0));
	tf_right  = tfBuf.lookupTransform("world", "bin5_right",  ros::Time(0), ros::Duration(1.0));
	
	obj.id = "bin5";
	//bottom
	obj.primitive_poses[0].position.x = tf_bottom.transform.translation.x;
	obj.primitive_poses[0].position.y = tf_bottom.transform.translation.y;
	obj.primitive_poses[0].position.z = tf_bottom.transform.translation.z;
	//back
	obj.primitive_poses[1].position.x = tf_back.transform.translation.x;
	obj.primitive_poses[1].position.y = tf_back.transform.translation.y;
	obj.primitive_poses[1].position.z = tf_back.transform.translation.z;
	//front
	obj.primitive_poses[2].position.x = tf_front.transform.translation.x;
	obj.primitive_poses[2].position.y = tf_front.transform.translation.y;
	obj.primitive_poses[2].position.z = tf_front.transform.translation.z;
	//left
	obj.primitive_poses[3].position.x = tf_left.transform.translation.x;
	obj.primitive_poses[3].position.y = tf_left.transform.translation.y;
	obj.primitive_poses[3].position.z = tf_left.transform.translation.z;
	//right
	obj.primitive_poses[4].position.x = tf_right.transform.translation.x;
	obj.primitive_poses[4].position.y = tf_right.transform.translation.y;
	obj.primitive_poses[4].position.z = tf_right.transform.translation.z;
	
	c_objs.push_back(obj);
	
	
	
	/***************************
	 * 
	 * 
	 * 		FLOOR COLLISIONS
	 * 
	 * 
	 * 
	 * *************************/
	 
	 // object
	 moveit_msgs::CollisionObject floor;
	 floor.header.frame_id = "world";
	 floor.id = "floor";
	 floor.operation = floor.ADD;
	 
	 // primitive shape
	 shape_msgs::SolidPrimitive pr_floor;
	 pr_floor.type = pr_floor.BOX;
	 pr_floor.dimensions.resize(3);
	 pr_floor.dimensions[0] = 2.75;
	 pr_floor.dimensions[1] = 8.61;
	 pr_floor.dimensions[2] = 0.104;
	 floor.primitives.push_back(pr_floor);
	 
	 //primitive pose
	 geometry_msgs::Pose po_floor;
	 po_floor.position.x = 0.095;
	 po_floor.position.y = 0.715;
	 po_floor.position.z = 0.1;
	 q.setRPY(0, 0, 0);
	 po_floor.orientation.x = q.x();
	 po_floor.orientation.y = q.y();
	 po_floor.orientation.z = q.z();
	 po_floor.orientation.w = q.w();
	 floor.primitive_poses.push_back(po_floor);
	 
	 // add object to world
	 c_objs.push_back(floor);
	 
	 
	 /***************************
	 * 
	 * 
	 * 		SHELF COLLISIONS
	 * 
	 * 
	 * 
	 * *************************/
	 
	 // object
	 moveit_msgs::CollisionObject shelf;
	 shelf.header.frame_id = "world";
	 shelf.id = "shelf";
	 shelf.operation = shelf.ADD;
	 
	 
	 // 1.
	 
	 // shelf-below primitive shape
	 shape_msgs::SolidPrimitive pr_shelf;
	 pr_shelf.type = pr_shelf.BOX;
	 pr_shelf.dimensions.resize(3);
	 pr_shelf.dimensions[0] = 0.600; // OLD: 0.538
	 pr_shelf.dimensions[1] = 8.1;
	 pr_shelf.dimensions[2] = 0.01;
	 shelf.primitives.push_back(pr_shelf);
	 
	 // shelf-below primitive pose
	 geometry_msgs::Pose po_shelf;
	 po_shelf.position.x = -0.741;
	 po_shelf.position.y = 0.72;
	 po_shelf.position.z = 0.74;
	 q.setRPY(0, 0.17, 0);
	 po_shelf.orientation.x = q.x();
	 po_shelf.orientation.y = q.y();
	 po_shelf.orientation.z = q.z();
	 po_shelf.orientation.w = q.w();
	 shelf.primitive_poses.push_back(po_shelf);
	 
	  // shelf-below front part shape
	 shape_msgs::SolidPrimitive pr_front;
	 pr_front.type = pr_shelf.BOX;
	 pr_front.dimensions.resize(3);
	 pr_front.dimensions[0] = 0.10; // OLD: 0.07
	 pr_front.dimensions[1] = 8.1;
	 pr_front.dimensions[2] = 0.03;
	 shelf.primitives.push_back(pr_front);
	 
	 // shelf-below front part pose
	 geometry_msgs::Pose po_front;
	 po_front.position.x = -0.497;
	 po_front.position.y = 0.72;
	 po_front.position.z = 0.712;
	 q.setRPY(0, 0.17, 0);
	 po_front.orientation.x = q.x();
	 po_front.orientation.y = q.y();
	 po_front.orientation.z = q.z();
	 po_front.orientation.w = q.w();
	 shelf.primitive_poses.push_back(po_front);
	 
	 
	 // 2.
	 
	 // shelf-above primitive shape
	 shelf.primitives.push_back(pr_shelf);
	 
	 // shelf-above primitive pose
	 po_shelf.position.z = 1.292;
	 shelf.primitive_poses.push_back(po_shelf);
	 
	 
	 // shelf-above front part shape
	 shelf.primitives.push_back(pr_front);
	 
	 // shelf-above front part pose
	 po_front.position.z = 1.264;
	 shelf.primitive_poses.push_back(po_front);
	 
	 
	 // 3.
	 
	 // shelf bars primitive shapes
	 shape_msgs::SolidPrimitive pr_bar;
	 pr_bar.type = pr_bar.BOX;
	 pr_bar.dimensions.resize(3);
	 pr_bar.dimensions[0] = 0.02;
	 pr_bar.dimensions[1] = 0.015;
	 pr_bar.dimensions[2] = 1.68;
	 shelf.primitives.push_back(pr_bar);
	 shelf.primitives.push_back(pr_bar);
	 shelf.primitives.push_back(pr_bar);
	 shelf.primitives.push_back(pr_bar);
	 shelf.primitives.push_back(pr_bar);
	 shelf.primitives.push_back(pr_bar);
	 
	 // shelf bars primitive poses
	 geometry_msgs::Pose po_bar;
	 po_bar.position.x = -0.475;
	 po_bar.position.y = -0.895;
	 po_bar.position.z = 0.97;
	 q.setRPY(0, 0, 0);
	 po_bar.orientation.x = q.x();
	 po_bar.orientation.y = q.y();
	 po_bar.orientation.z = q.z();
	 po_bar.orientation.w = q.w();
	 shelf.primitive_poses.push_back(po_bar);
	 po_bar.position.y = -0.08;
	 shelf.primitive_poses.push_back(po_bar);
	 po_bar.position.y = 0.73;
	 shelf.primitive_poses.push_back(po_bar);
	 po_bar.position.y = 1.54;
	 shelf.primitive_poses.push_back(po_bar);
	 po_bar.position.y = 2.35;
	 shelf.primitive_poses.push_back(po_bar);
	 po_bar.position.y = 3.165;
	 shelf.primitive_poses.push_back(po_bar);
	 
	 // add object to world
	 c_objs.push_back(shelf);
	 
	 
	/***************************
	 * 
	 * 
	 * 		BELT COLLISIONS
	 * 
	 * 
	 * 
	 * *************************/
	
	 // object
	 moveit_msgs::CollisionObject belt;
	 belt.header.frame_id = "world";
	 belt.id = "belt";
	 belt.operation = belt.ADD;
	 
	 // 1. 
	 
	 // belt top primitive shape
	 shape_msgs::SolidPrimitive pr_belt_top;
	 pr_belt_top.type = pr_belt_top.BOX;
	 pr_belt_top.dimensions.resize(3);
	 pr_belt_top.dimensions[0] = 0.58;
	 pr_belt_top.dimensions[1] = 7.5;
	 pr_belt_top.dimensions[2] = 0.001;
	 belt.primitives.push_back(pr_belt_top);
	 
	 
	 // belt top primitive pose
	 geometry_msgs::Pose po_belt_top;
	 po_belt_top.position.x = 0.5475; 
	 po_belt_top.position.y = 0.6275;
	 po_belt_top.position.z = 0.58;
	 q.setRPY(0, 0, 0);
	 po_belt_top.orientation.x = q.x();
	 po_belt_top.orientation.y = q.y();
	 po_belt_top.orientation.z = q.z();
	 po_belt_top.orientation.w = q.w();
	 belt.primitive_poses.push_back(po_belt_top);
	 
	 // 2.
	 
	 // belt front side primitive shape
	 shape_msgs::SolidPrimitive pr_belt_front;
	 pr_belt_front.type = pr_belt_front.BOX;
	 pr_belt_front.dimensions.resize(3);
	 pr_belt_front.dimensions[0] = 0.105;
	 pr_belt_front.dimensions[1] = 7.98;
	 pr_belt_front.dimensions[2] = 0.4;
	 belt.primitives.push_back(pr_belt_front);
	 
	 // belt front side primitive pose
	 geometry_msgs::Pose po_belt_front;
	 po_belt_front.position.x = 0.90;
	 po_belt_front.position.y = 0.6275;
	 po_belt_front.position.z = 0.41;
	 q.setRPY(0, 0, 0);
	 po_belt_front.orientation.x = q.x();
	 po_belt_front.orientation.y = q.y();
	 po_belt_front.orientation.z = q.z();
	 po_belt_front.orientation.w = q.w();
	 belt.primitive_poses.push_back(po_belt_front);
	 
	 // 3.
	 
	 // belt back side primitive shape
	 shape_msgs::SolidPrimitive pr_belt_back;
	 pr_belt_back.type = pr_belt_back.BOX;
	 pr_belt_back.dimensions.resize(3);
	 pr_belt_back.dimensions[0] = 0.105;
	 pr_belt_back.dimensions[1] = 7.98;
	 pr_belt_back.dimensions[2] = 0.4;
	 belt.primitives.push_back(pr_belt_back);
	 
	 // belt back side primitive pose
	 geometry_msgs::Pose po_belt_back;
	 po_belt_back.position.x = 0.225;
	 po_belt_back.position.y = 0.6275;
	 po_belt_back.position.z = 0.41;
	 q.setRPY(0, 0, 0);
	 po_belt_back.orientation.x = q.x();
	 po_belt_back.orientation.y = q.y();
	 po_belt_back.orientation.z = q.z();
	 po_belt_back.orientation.w = q.w();
	 belt.primitive_poses.push_back(po_belt_back);
	
	 
	 
	 // add object to world
	 c_objs.push_back(belt);	
	
	
	
	/***************************
	 * 	
	 * 
	 * 		BOX COLLISIONS
	 * 
	 * 
	 * 
	 * *************************/
	 
	 // object
	 moveit_msgs::CollisionObject box;
	 box.header.frame_id = "world";
	 box.id = "box";
	 box.operation = box.ADD;
	 
	 // tf frame of object
	 geometry_msgs::TransformStamped tf_box;
	 tf_box = tfBuf.lookupTransform( "world", "logical_camera_6_shipping_box_0_frame", ros::Time(0), ros::Duration(1.0) );
	 ROS_INFO("%f %f %f ", tf_box.transform.translation.x, tf_box.transform.translation.y, tf_box.transform.translation.z);
	 
	 // 1.
	 
	 // box mesh 
	 box.meshes.resize(1);
	 shapes::Mesh* m = shapes::createMeshFromResource( "package://osrf_gear/models/open_box_ariac/meshes/open_box.obj", Eigen::Vector3d(1.1, 1.8, 0.25) );
	 shape_msgs::Mesh mesh;
     shapes::ShapeMsg mesh_msg;  
	 shapes::constructMsgFromShape(m, mesh_msg);
	 mesh = boost::get<shape_msgs::Mesh>(mesh_msg); 
	 box.meshes[0] = mesh;
	 
	 // box pose
	 box.mesh_poses.resize(1);
	 box.mesh_poses[0].position.x = tf_box.transform.translation.x;
	 box.mesh_poses[0].position.y = tf_box.transform.translation.y;
	 box.mesh_poses[0].position.z = tf_box.transform.translation.z;
	 box.mesh_poses[0].orientation = tf_box.transform.rotation;
	 
	 
	 /** Update box positions as they move on the belt **/ 
	 
	 // add object to world
	 c_objs.push_back(box);
	 
	 /***************************
	 * 
	 * 
	 * 		SENSOR COLLISIONS
	 * 
	 * 
	 * 
	 * *************************/
	
	moveit_msgs::CollisionObject sensors;
	sensors.header.frame_id = "world";
	sensors.id = "sensors";
	sensors.operation = sensors.ADD;
	
	// 1.
	
	// Box sensor
	shape_msgs::SolidPrimitive sensor;
	sensor.type = sensor.BOX;
	sensor.dimensions.resize(3);
	sensor.dimensions[0] = 0.08;
	sensor.dimensions[1] = 0.06;
	sensor.dimensions[2] = 0.13;
	sensors.primitives.push_back(sensor);
	
	geometry_msgs::Pose po_sensor;
	po_sensor.position.x = 0.767;
	po_sensor.position.y = 0.907;
	po_sensor.position.z = 1.042;
	q.setRPY(0, 1.2587, 3.14159);
	po_sensor.orientation.x = q.x();
	po_sensor.orientation.y = q.y();
	po_sensor.orientation.z = q.z();
	po_sensor.orientation.w = q.w();
	sensors.primitive_poses.push_back(po_sensor);
	
	// 2.
	
	// Bin 1 - 5 sensors
	sensors.primitives.push_back(sensor);
	sensors.primitives.push_back(sensor);
	sensors.primitives.push_back(sensor);
	sensors.primitives.push_back(sensor);
	sensors.primitives.push_back(sensor);
	po_sensor.position.x = -0.78;
	po_sensor.position.y = -0.49;
	po_sensor.position.z = 1.23;
	q.setRPY(0, 1.5707, 3.14159);  
	po_sensor.orientation.x = q.x();
	po_sensor.orientation.y = q.y();
	po_sensor.orientation.z = q.z();
	po_sensor.orientation.w = q.w();
	sensors.primitive_poses.push_back(po_sensor);
	po_sensor.position.y = 0.32;
	sensors.primitive_poses.push_back(po_sensor);
	po_sensor.position.y = 1.14;
	sensors.primitive_poses.push_back(po_sensor);
	po_sensor.position.y = 1.94;
	sensors.primitive_poses.push_back(po_sensor);
	po_sensor.position.y = 2.75;
	sensors.primitive_poses.push_back(po_sensor);
	
	
	// 3.
	
	// Quality sensor 1 - 2
	sensors.primitives.push_back(sensor);
	sensors.primitives.push_back(sensor);
	po_sensor.position.x = 0.55;
	po_sensor.position.y = 1.1;
	po_sensor.position.z = 1.37;
	q.setRPY(-1.5707, 1.5707, -3.1416);
	po_sensor.orientation.x = q.x();
	po_sensor.orientation.y = q.y();
	po_sensor.orientation.z = q.z();
	po_sensor.orientation.w = q.w();
	sensors.primitive_poses.push_back(po_sensor);
	po_sensor.position.y = -0.7;
	sensors.primitive_poses.push_back(po_sensor);
	
	c_objs.push_back(sensors);
	
	
	//Add all collision objects to the planning scene
	scene.addCollisionObjects(c_objs);
	ros::waitForShutdown();	
	
}

