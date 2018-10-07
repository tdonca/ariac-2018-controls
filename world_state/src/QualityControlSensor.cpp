
#include <world_state/QualityControlSensor.hpp>
#include <geometry_msgs/TransformStamped.h>

namespace world {
	
	std::string getNameFromFrame( std::string camera_name, std::string frame_name );
	std::string getTypeFromName( std::string name );
	std::string getIDFromName( std::string name );
	std::vector<std::string> split(const char *str, char c = '_');
	void TFMapToVec( const TFMap & m, std::vector<std::string> & v );
	void transformToPose( const geometry_msgs::Transform & t, geometry_msgs::Pose & p );
	
	
	bool QualityControlSensor::start(){
		
		// setup to detect parts
		m_update_t = m_node.createTimer( ros::Duration(1.0), &QualityControlSensor::cb_updateParts, this );
		
		return true;
	}
		
			
	std::vector<SensorPart> QualityControlSensor::getVisibleParts(){
		
		return sensorMapToVector( m_parts );
	}
	
	
	
		
	void QualityControlSensor::cb_updateParts( const ros::TimerEvent & t ){
		ROS_INFO("......%s Update.......", getName().c_str());
		// copy tf list
		std::vector<std::string> tf_names;
		TFMapToVec( m_tf_list, tf_names );
		
		
		// tf transforms
		geometry_msgs::TransformStamped to_world;
		
		
		// select only tf from this camera
		for( int i = 0; i < tf_names.size(); ++i ){
			if( tf_names[i].find( getName() ) != std::string::npos ){ // tf from this camera
				if( tf_names[i].find( "model" ) != std::string::npos ){ // tf of a part
					
					// find part
					try {
						to_world = m_tfBuf->lookupTransform( "world", tf_names[i], ros::Time(0), ros::Duration( 1.0 ) );
					} 
					catch (tf2::LookupException e) {
						// part not found, remove tf from list
						ROS_ERROR("Exception caught, a part disappeared from sensor.");
						m_tf_list.erase(tf_names[i]);
						continue;
					}
					
					// create part
					SensorPart new_part;
					// name
					std::string name = getNameFromFrame( getName(), tf_names[i] );
					new_part.name = name;
					// type
					new_part.type = getTypeFromName( name ); 
					// id
					new_part.id = getIDFromName( name ); 
					// pose
					transformToPose( to_world.transform, new_part.pose );
					// time-stamp
					new_part.stamp = ros::Time::now();
					// add part
					addPart(new_part);
				}
			}
		}
		
		
		// remove outdated parts
		std::vector<std::string> old_parts;
		for( SensorPartsMap::iterator it = m_parts.begin(); it != m_parts.end(); ++it ){
			if( ros::Time::now() - it->second.stamp > ros::Duration(0.4) ){
				old_parts.push_back( it->second.name );
			}
		}
		for( int i = 0; i < old_parts.size(); ++i ){
			ROS_ERROR("Quality Sensor erased an old part %s %f.", old_parts[i].c_str(), m_parts[old_parts[i]].stamp.toSec());
			removePart(old_parts[i]);
		}
		
		
		ROS_INFO("%s has %lu parts.", getName().c_str(), m_parts.size());
	}


	void QualityControlSensor::addPart( SensorPart part ){
		
		m_parts[part.name] = part;
	}
	
	void QualityControlSensor::removePart( std::string part_name ){
		
		m_parts.erase(part_name);
	}
	
	
	void QualityControlSensor::printSensor(){
		
		std::vector< SensorPart > parts = getVisibleParts();
		ROS_INFO("--------%s Info: %lu parts----------", getName().c_str(), parts.size());
		for( int i = 0; i < parts.size(); ++i ){
			ROS_INFO("Part %d", i+1);
			parts[i].printPart();
		}
		
		ROS_INFO(" ");
	}
	
		
		
} 
