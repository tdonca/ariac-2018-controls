#include <world_state/WorldState.hpp>
#include <algorithm>


namespace world {
	
	double partOffset( std::string type );

		
	bool WorldState::initializeWorld(){
		
		// topics
		m_tf_sub = m_node.subscribe( "tf", 100, &WorldState::cb_tfList, this);
		ros::Duration(0.5).sleep();
		
		// timers
		m_update_t = m_node.createTimer( ros::Duration(2.0), &WorldState::cb_updateParts, this );
		
		// Boxes
		Box b1("BOX0");
		addBox(b1);
		
		
		//~ // DEBUG -- REMOVE!!
			//~ Part part1("gasket_part_63", geometry_msgs::Pose(), &m_removed);
			//~ addNewPartToGripper(part1);
			//~ Part part2("gear_part_5", geometry_msgs::Pose(), &m_removed);
			//~ addNewPartToBox(part2);
			//~ Part part3("pulley_part_60", geometry_msgs::Pose(), &m_removed);
			//~ addNewPartToBox(part3);
			
		//~ // DEBUG -- REMOVE!!
		
		
		// Sensors
		addCameraSensor("logical_camera_1");
		addCameraSensor("logical_camera_2");
		addCameraSensor("logical_camera_3");
		addCameraSensor("logical_camera_4");
		addCameraSensor("logical_camera_5");
		addCameraSensor("logical_camera_6");
		addQualitySensor("quality_control_sensor_1");
		m_bins[0].connectSensor( m_sensors["logical_camera_1"].get() );
		m_bins[1].connectSensor( m_sensors["logical_camera_2"].get() );
		m_bins[2].connectSensor( m_sensors["logical_camera_3"].get() );
		m_bins[3].connectSensor( m_sensors["logical_camera_4"].get() );
		m_bins[4].connectSensor( m_sensors["logical_camera_5"].get() );
		m_boxes[0]->connectSensor( m_sensors["logical_camera_6"].get() );
		m_sensors["logical_camera_1"]->start();
		m_sensors["logical_camera_2"]->start();
		m_sensors["logical_camera_3"]->start();
		m_sensors["logical_camera_4"]->start();
		m_sensors["logical_camera_5"]->start();
		m_sensors["logical_camera_6"]->start();
		m_sensors["quality_control_sensor_1"]->start();
		
		// Robot
		m_graph.initializeGraph();
		std::unique_ptr<Robot> r1 = std::unique_ptr<Robot>( new IIWA14Robot("iiwa14", m_move_group) );
		addRobot( std::move(r1) );
		m_robot->initialize(m_graph);
		
		// services
		m_find_part_type_srv = 		m_node.advertiseService( "find_part_type", &WorldState::sv_findPartType, this );
		m_mark_part_used_srv = 		m_node.advertiseService( "mark_part_used", &WorldState::sv_markPartUsed, this );
		m_release_part_srv = 		m_node.advertiseService( "release_part", &WorldState::sv_releasePart, this );
		m_gripper_part_srv = 		m_node.advertiseService( "gripper_part", &WorldState::sv_getGripperPart, this );
		m_box_parts_srv = 			m_node.advertiseService( "box_parts", &WorldState::sv_getBoxParts, this );
		m_move_part_to_box_srv = 	m_node.advertiseService( "move_part_to_box", &WorldState::sv_movePartToBox, this );
		m_remove_part_srv = 		m_node.advertiseService( "remove_part", &WorldState::sv_removePart, this );
		m_part_pose_srv =			m_node.advertiseService( "get_part_pose", &WorldState::sv_getPartPose, this );
		m_bin_location_srv =		m_node.advertiseService( "get_bin_location", &WorldState::sv_getBinLocation, this );
		m_box_location_srv =		m_node.advertiseService( "get_box_location", &WorldState::sv_getBoxLocation, this );
		m_goal_pose_srv =			m_node.advertiseService( "compute_goal_pose", &WorldState::sv_computeGoalPose, this );
			
		return true;
	}
	
	
	void WorldState::cb_updateParts( const ros::TimerEvent & t ){
		ROS_INFO("+++++World Update+++++");
		// bins
		for( int i = 0; i < m_bins.size(); ++i ){
			
			std::vector<SensorPart> s_parts = m_bins[i].getSensor()->getVisibleParts();
			std::vector<Part> c_parts = m_bins[i].getParts();
			
			// every sensor part
			for( int k = 0; k < s_parts.size(); ++k ){
				
				//find the part in the container
				bool found = false;
				for( int p = 0; p < c_parts.size(); ++p ){
					if( c_parts[p].getName() == s_parts[k].name ){
						
						// update part pose
						if( m_bins[i].updatePartPose( c_parts[p].getName(), s_parts[k].pose ) ){
							//~ ROS_INFO("Update %s pose", c_parts[p].getName().c_str());
						}
						else{
							ROS_ERROR("Error trying to update %s pose!", c_parts[p].getName().c_str());
						}
						
						found = true;
					}
				} 
				
				// add the part to the world and bin only if it does not exist in the world yet
				if( !found && m_parts[s_parts[k].name][s_parts[k].type].expired() ){
					
					Part new_part( s_parts[k].name, s_parts[k].pose, &m_removed );
					if( addNewPart( new_part, i+1 ) ){
						ROS_INFO("Added the new part %s to the world in %s", s_parts[k].name.c_str(), m_bins[i].getName().c_str());
					}
					else{
						ROS_ERROR("Error adding the new part %s to the world!", s_parts[k].name.c_str());
					}
				}
			}
		}
		
		
		// box
		if(m_boxes.size() > 0){
			std::vector<SensorPart> s_parts = m_boxes[0]->getSensor()->getVisibleParts();
			std::vector<Part> c_parts = m_boxes[0]->getParts();
			
			// every sensor part
			for( int k = 0; k < s_parts.size(); ++k ){
				
				//find the part in the container
				bool found = false;
				for( int p = 0; p < c_parts.size(); ++p ){
					if( c_parts[p].getName() == s_parts[k].name ){
						
						// update part pose
						if( m_boxes[0]->updatePartPose( c_parts[p].getName(), s_parts[k].pose ) ){
							//~ ROS_INFO("Update %s pose", c_parts[p].getName().c_str());
						}
						else{
							ROS_ERROR("Error trying to update %s pose!", c_parts[p].getName().c_str());
						}
						
						found = true;
					}
				} 
		
				if( !found ){
					ROS_ERROR("Error updating %s from %s sensor, the part does not exist in the container!", s_parts[k].name.c_str(), m_boxes[0]->getName().c_str());
				}
			}
			
			
			
			// check for faulty parts
			std::vector<SensorPart> faulty_parts = m_sensors["quality_control_sensor_1"]->getVisibleParts();
			
			
			
			for( int i = 0; i < faulty_parts.size(); ++i ){
				ROS_ERROR("Faulty Part Detected: %s!", faulty_parts[i].name.c_str());
				bool found = false;
				
				// only mark the fauly part once
				if( std::find(m_faulty_parts.begin(), m_faulty_parts.end(), faulty_parts[i].id) != m_faulty_parts.end() ){
					continue;
				}
				
				// v MOVE TO A SEPARATE FUNCTION --------------------
				// find matching part id
				for( WorldPartsMap::iterator it = m_parts.begin(); it != m_parts.end(); ++it ){
					for( std::unordered_map<std::string, std::weak_ptr<Part> >::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2 ){
						
						std::string tmp_name = it2->first;
						if( getIDFromName(tmp_name) == faulty_parts[i].id ){
							// mark faulty
							it2->second.lock()->markFaulty();
							ROS_ERROR("Marked %s as faulty", it2->first.c_str());
							m_faulty_parts.push_back(getIDFromName(tmp_name)); 
							found = true;
							break;
						}
					}
					if(found){
						break;
					}
				}
				// ^ MOVE TO A SEPARATE FUNCTION ________________
				
				if(!found){
					ROS_ERROR("Could not mark %s faulty !!!", faulty_parts[i].name.c_str());
				}
			}	
		}
	}
	
	bool WorldState::addNewPart( Part part, int bin ){
		
		ROS_INFO("Adding %s to bin %d.", part.getName().c_str(), bin);
		int binnum = bin-1;
		
		// only add part if it doesn't already exist
		if( m_parts[part.getType()][part.getName()].expired() ){
			
			// add part shared_ptr to bin
			std::shared_ptr<Part> part_ptr = std::make_shared<Part>(part);
			m_bins[binnum].addPart(part_ptr);
			// add part weak_ptr to world
			m_parts[part.getType()][part.getName()] = part_ptr;
		}
		else{
			ROS_ERROR("Could not add %s to the world, it already exists!", part.getName().c_str());
			return false;
		}
		
		return true;
	}
	
			
	
	bool WorldState::addNewPartToBox( Part part ){
		
		if(m_boxes.size() > 0){
			// only add part if it doesn't already exist
			if( m_parts[part.getType()][part.getName()].expired() ){
				
				// add part shared_ptr to bin
				std::shared_ptr<Part> part_ptr = std::make_shared<Part>(part);
				m_boxes[0]->addPart(part_ptr);
				// add part weak_ptr to world
				m_parts[part.getType()][part.getName()] = part_ptr;
			}
			else{
				ROS_ERROR("Could not add %s to the world, it already exists!", part.getName().c_str());
				return false;
			}
			
		}
		else{
			ROS_ERROR("Cannot add part to box, there are no boxes in the world!");
		}
	}		
	
	
	
	bool WorldState::addNewPartToGripper( Part part ){
		
		if( m_parts[part.getType()][part.getName()].expired() ){
			// add part shared_ptr to bin
			std::shared_ptr<Part> part_ptr = std::make_shared<Part>(part);
			m_gripper.addPart(part_ptr);
			// add part weak_ptr to world
			m_parts[part.getType()][part.getName()] = part_ptr;
		}
		else{
			ROS_ERROR("Could not add %s to the world, it already exists!", part.getName().c_str());
			return false;
		}
	}
	
	
	
	
	bool WorldState::removePart( std::string name ){
		
		std::string type = getTypeFromName(name);
		
		// release pointer if it is still active
		if( m_parts[type][name].expired() ){
			ROS_ERROR("Could not remove %s from the world, it does not exist!", name.c_str());
		}
		else{
			ROS_ERROR("Could not remove %s from the world, it still exists somewhere!", name.c_str());
		}
		
		return false;
	}
	
	
	bool WorldState::addBox(Box box ){
		
		ROS_INFO("Adding box %s to the world.", box.getName().c_str());
		std::unique_ptr<Box> b = std::unique_ptr<Box>( new Box(box) );
		m_boxes.push_back( std::move(b) );
		ROS_INFO("There are now %lu boxes.", m_boxes.size());
		return true;
	}
			
	
	
	bool WorldState::removeBox(){
		
		if( m_boxes.size() > 0 ){
			ROS_INFO("Removing the farthest box %s from the world.", m_boxes.front()->getName().c_str());
			m_boxes.pop_front();
		}
		else{
			ROS_ERROR("There are no boxes in the world to remove!");
			return false;
		}
		
		return true;
	}
		
		
	
	bool WorldState::addCameraSensor( std::string sensor_name ){
		
		ROS_INFO("Adding sensor %s to the world.", sensor_name.c_str());
		std::unique_ptr<Sensor> s( new LogicalCameraSensor(sensor_name, &m_tfBuf, m_tf_list) );
		m_sensors[ sensor_name ] = std::move(s);
		ROS_INFO("There are now %lu sensors.", m_sensors.size());
		
		return true;
	}
	
	bool WorldState::addQualitySensor( std::string sensor_name ){
		
		ROS_INFO("Adding sensor %s to the world.", sensor_name.c_str());
		std::unique_ptr<Sensor> s( new QualityControlSensor(sensor_name, &m_tfBuf, m_tf_list) );
		m_sensors[ sensor_name ] = std::move(s);
		ROS_INFO("There are now %lu sensors.", m_sensors.size());
		
		return true;
	}
	
	
	
	bool WorldState::removeSensor( std::string sensor_name ){
	
		if( m_sensors[sensor_name] != nullptr ){
			ROS_INFO("Removing %s from the world.", sensor_name.c_str());
			m_sensors[sensor_name] = nullptr;
		}
		else{
			ROS_ERROR("Could not remove %s, it does not exist!", sensor_name.c_str());
			return false;
		}

		return true;
	}	
	
	
	bool WorldState::addRobot( std::unique_ptr<Robot> robot ){
		
		if( m_robot == nullptr ){
			ROS_INFO("Adding robot %s to the world.", robot->getName().c_str());
			m_robot = std::move(robot);
		}
		else{
			ROS_ERROR("Cannot add the robot, one already exists!");
			return false;
		}
		
		return true;
	}
			
			
	bool WorldState::removeRobot( std::string robot_name ){
		
		if( robot_name == m_robot->getName() ){
			m_robot = nullptr;
		}
		else{
			ROS_ERROR("Cannot remove the robot %s, it does not exist.", robot_name.c_str());
			return false;
		}
		
		return true;
	}
			
			
	bool WorldState::testFunction(){
		return true;
	}
	
	
	
	
		
	void WorldState::cb_tfList( const tf2_msgs::TFMessage::ConstPtr & tf_list ){
		
		// update internal list of TF frame names
		std::string pname;
		for( int i = 0; i < tf_list->transforms.size(); ++i ){
			pname = tf_list->transforms[i].child_frame_id;
			m_tf_list[pname].first = pname;
			m_tf_list[pname].second = tf_list->transforms[i].header.stamp;
		}
		
		// remove old frame names
		ros::Time now = ros::Time::now();
		for( TFMap::iterator it = m_tf_list.begin(); it != m_tf_list.end(); ++it ){
			if( now - it->second.second > ros::Duration(0.2) ){
				ROS_ERROR("Removed %s", it->first.c_str());
				m_tf_list.erase( it->first );
			}
		}
		
	}

		
	bool WorldState::sv_findPartType(world_state::FindPartType::Request & req, world_state::FindPartType::Response & rsp ){
		
		if( req.type != "gear_part" && req.type != "gasket_part" && req.type != "disk_part" && req.type != "pulley_part" && req.type != "piston_rod_part" ){
			rsp.success = false;
			rsp.message = "Did not provide a valid part type";
		}
		else{
			
			for( std::unordered_map<std::string, std::weak_ptr<Part> >::iterator it = m_parts[req.type].begin(); it != m_parts[req.type].end(); ++it ){
				
				std::shared_ptr<Part> p = it->second.lock();
				if( p->isAvailable() && !p->isFaulty() ){
					rsp.part_name = p->getName();
					rsp.success = true;
					return true;
				}
			}
			
			rsp.success = false;
			rsp.message = "could not find any avilable " + req.type + " type";
		}

		return true;
	}
			
	
	bool WorldState::sv_markPartUsed( world_state::MarkPartUsed::Request & req, world_state::MarkPartUsed::Response & rsp ){
					
		std::string type = getTypeFromName(req.name);
		if( type != "gear_part" && type != "gasket_part" && type != "disk_part" && type != "pulley_part" && type != "piston_rod_part" ){
			rsp.success = false;
			rsp.message = "Did not provide a valid part name";
		}
		else{
			if( !m_parts[type][req.name].expired() ){
				m_parts[type][req.name].lock()->markUsed();
				rsp.success = true;
			}
			else{
				rsp.success = false;
				rsp.message = "Could not find the part: " + req.name;
			}	
		}
		
		return true;
	}
	
			
	
	bool WorldState::sv_releasePart( world_state::ReleasePart::Request & req, world_state::ReleasePart::Response & rsp ){
		
		std::string type = getTypeFromName(req.name);
		if( !m_parts[type][req.name].expired() ){
			m_parts[type][req.name].lock()->markAvailable();
			rsp.success = true;
			return true;
		}
		else{
			rsp.success = false;
			rsp.message = "Could not find the part: " + req.name;
			return true;	
		}
		
	}
	
	
	bool WorldState::sv_getGripperPart( world_state::GetGripperPart::Request & req, world_state::GetGripperPart::Response & rsp ){
		
		std::vector<Part> gp = m_gripper.getParts();
		if(gp.size() > 0){
			rsp.part.name = gp[0].getName();
			rsp.part.type = getTypeFromName(rsp.part.name);
			rsp.part.id = getIDFromName(rsp.part.name);
			rsp.part.current_pose = gp[0].getPose();
			rsp.success = true;
			rsp.message = "Found the gripper part " + rsp.part.name;
		}
		else{
			rsp.success = false;
			rsp.message = "Gripper is not holding any part.";
		}
		
		return true;
	}
			
	
	
	bool WorldState::sv_getBoxParts( world_state::GetBoxParts::Request & req, world_state::GetBoxParts::Response & rsp ){
		
		if( m_boxes.size() > 0 ){
			std::vector<Part> bp = m_boxes[0]->getParts();
			rsp.parts.resize(bp.size());
			for( int i = 0; i < bp.size(); ++i ){
				rsp.parts[i].name = bp[i].getName();
				rsp.parts[i].type = getTypeFromName(rsp.parts[i].name);
				rsp.parts[i].id = getIDFromName(rsp.parts[i].name);
				rsp.parts[i].current_pose = bp[i].getPose();
			}
			rsp.success = true;
			rsp.message = "Found all " + std::to_string(bp.size()) + " parts in the box.";
		}
		else{
			rsp.success = false;
			rsp.message = "There are no active boxes in the world!";
		}
		
		return true;
	}
			
	
	
	
	bool WorldState::sv_movePartToBox( world_state::MovePartToBox::Request & req, world_state::MovePartToBox::Response & rsp ){
	
		std::string type = getTypeFromName(req.name);
		ROS_ERROR("%s , %s", req.name.c_str(), type.c_str());
		if( !m_parts[type][req.name].expired() ){
			
			std::shared_ptr<Part> p = m_parts[type][req.name].lock();
			ROS_INFO("Found the part %s", p->getName().c_str());
			if( m_boxes.size() > 0 ){
				m_boxes[0]->addPart( m_parts[type][req.name].lock()->getLocation()->removePart( req.name ) );
				rsp.success = true;
				rsp.message = "Moved the part to the box";
				return true;
			}
			else{
				rsp.success = false;
				rsp.message = "No boxes in the world.";
				return true;
			}
		}
		else{
			rsp.success = false;
			rsp.message = "Could not find the part: " + req.name;
			return true;	
		}
		
		
	}
			
			
				
	
	bool WorldState::sv_removePart( world_state::RemovePart::Request & req, world_state::RemovePart::Response & rsp ){
		
		std::string type = getTypeFromName(req.name);
		if( !m_parts[type][req.name].expired() ){
			
			if( m_boxes.size() > 0 ){
				m_parts[type][req.name].lock()->getLocation()->removePart( req.name );
				rsp.success = true;
				rsp.message = "Removed the part";
				return true;
			}
			else{
				rsp.success = false;
				rsp.message = "No boxes in the world.";
				return true;
			}
		}
		else{
			rsp.success = false;
			rsp.message = "Could not find the part: " + req.name;
			return true;	
		}
		
	}
			
			
	

	bool WorldState::sv_getPartPose( world_state::GetPartPose::Request & req, world_state::GetPartPose::Response & rsp ){

		// check that the part exists
		std::string type = getTypeFromName(req.part_name);
		if( !m_parts[type][req.part_name].expired() ){
			

			rsp.part_pose = m_parts[type][req.part_name].lock()->getPose();
			rsp.offset = partOffset(type);
			rsp.success = true;
		}
		else{
			rsp.success = false;
			rsp.message = "Could not find the part: " + req.part_name;	
		}

		return true;
	}
			
	

	bool WorldState::sv_getBinLocation( world_state::GetBinLocation::Request & req, world_state::GetBinLocation::Response & rsp ){

		State tmp;
		if( m_graph.findState(req.bin_name, tmp) ){
			// TODO: Remove this hardcoding of joint names!
			rsp.joint_names = { "linear_arm_actuator_joint", "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
			rsp.joint_values = tmp.joint_values;
			rsp.success = true;
		}
		else{
			rsp.success = false;
			rsp.message = "Could not find the bin: %s", req.bin_name.c_str();
		}

		return true;
	}
			
	


	bool WorldState::sv_getBoxLocation( world_state::GetBoxLocation::Request & req, world_state::GetBoxLocation::Response & rsp ){

		State tmp;
		if( m_graph.findState(req.box_name, tmp) ){
			// TODO: Remove this hardcoding of joint names!
			rsp.joint_names = { "linear_arm_actuator_joint", "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
			rsp.joint_values = tmp.joint_values;
			rsp.success = true;
		}
		else{
			rsp.success = false;
			rsp.message = "Could not find the box: " + req.box_name;
		}

		return true;


	}
	

	bool WorldState::sv_computeGoalPose( world_state::ComputeGoalPose::Request & req, world_state::ComputeGoalPose::Response & rsp ){

		ROS_ERROR("Relative Pose: %.4f, %.4f, %.4f", req.relative_pose.position.x, req.relative_pose.position.y, req.relative_pose.position.z);
		geometry_msgs::TransformStamped box_tf;
		try {
			box_tf = m_tfBuf.lookupTransform( "world", "logical_camera_6_shipping_box_0_frame", ros::Time(0), ros::Duration( 1.0 ) );
			ROS_ERROR("Transform to world: %.4f, %.4f, %.4f", box_tf.transform.translation.x, box_tf.transform.translation.y, box_tf.transform.translation.z);
			tf2::doTransform(req.relative_pose, rsp.goal_pose, box_tf);
			ROS_ERROR("Goal Pose: %.4f, %.4f, %.4f", rsp.goal_pose.position.x, rsp.goal_pose.position.y, rsp.goal_pose.position.z);
			rsp.success = true;
		} 
		catch (tf2::LookupException e) {
			rsp.success = false;
			rsp.message = "Exception caught, could not find the box tf!";
			ROS_ERROR("%s", rsp.message.c_str());
		}

		return true;
	}









	//****************Helper Functions


	double partOffset( std::string type ){
		
		if( type == "gear_part" ){
			return 0.012;
		}
		else if( type == "disk_part" ){
			return 0.023;
		}
		else if( type == "piston_rod_part" ){
			return 0.0075;
		}
		else if( type == "gasket_part" ){
			return 0.02;
		}
		else if( type == "pulley_part" ){
			return 0.07;
		}
		else{
			ROS_ERROR("Invalid part type, could not determine offset.");
			return 0.0;
		}
	}

	
}
