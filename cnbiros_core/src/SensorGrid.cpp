#ifndef CNBIROS_CORE_SENSORGRID_CPP
#define CNBIROS_CORE_SENSORGRID_CPP

#include "cnbiros_core/SensorGrid.hpp"

namespace cnbiros {
	namespace core {

SensorGrid::SensorGrid(void) {}

SensorGrid::SensorGrid(const float x, const float y, const float r) {
	this->SetGeometry(x, y, r);
}

SensorGrid::~SensorGrid(void) {}

void SensorGrid::AddLayer(const std::string& layer) {
	this->add(layer);
}

bool SensorGrid::RemoveLayer(const std::string& layer) {
	return this->erase(layer);
}

bool SensorGrid::Exists(const std::string& layer) {
	bool exist;

	if ((exist = this->exists(layer)) == false) {
		ROS_WARN("Layer (%s) does not exist", layer.c_str());
	}

	return exist;
}

void SensorGrid::SetGeometry(const float x, const float y, const float r) {
	this->setGeometry(grid_map::Length(x, y), r);
}

grid_map_msgs::GridMap SensorGrid::ToMessage(void) {
	grid_map_msgs::GridMap msg;
	grid_map::GridMapRosConverter::toMessage(*this, msg);
	return msg;
}

void SensorGrid::SetFrame(const std::string& frame) {
	this->setFrameId(frame);
}

std::string SensorGrid::GetFrame(void) {
	return this->getFrameId();
}

void SensorGrid::Reset(const std::string& layer, float value) {

	if(this->Exists(layer)) {
		this->get(layer).setConstant(value);
	}
}

void SensorGrid::Reset(float value) {

	std::vector<std::string> layers = this->getLayers();
	
	for(auto it = layers.begin(); it != layers.end(); ++it) {
		this->Reset(*it, value);
	}
}

bool SensorGrid::Sum(const std::string& target) {
	bool result;
	std::vector<std::string> layers;
	layers = this->getLayers();

	result = this->Sum(target, layers);
	
	return result;
}

bool SensorGrid::Sum(const std::string& target, const std::vector<std::string>& layers) {
	bool result = true;
	std::vector<std::string>::const_iterator it;

	if(this->Exists(target) == false) {
		ROS_WARN("Target layer (%s) does not exist. Cannot sum required layers", target.c_str());
		return false;
	}

	for(it = layers.begin(); it != layers.end(); ++it) {
		if(this->Exists(*it) == false) {
			ROS_WARN("Layer %s does not exist in the grid map. Skipped.", (*it).c_str());
		} else {
			if((*it).compare(target) != 0) {
				this->get(target) += this->get(*it);
			}
		}
	}

	return result;
}

bool SensorGrid::ReplaceNaN(const std::string& layer, float value) {

	bool result = false;
	if(this->Exists(layer)) {
		this->get(layer) = (this->get(layer).array().isNaN()).select(0.0f, this->get(layer)); 	
		result = true;
	}

	return result;
}

bool SensorGrid::ReplaceNaN(float value) {

	bool result = true;
	std::vector<std::string> layers;
	std::vector<std::string>::iterator it;
	
	layers = this->getLayers();

	for (it = layers.begin(); it != layers.end(); ++it) {
		result = result & this->ReplaceNaN(*it, value);
	}
	
	return result;
}

bool SensorGrid::SetMin(const std::string& layer, float minimum) {
	bool result = false;

	if(this->Exists(layer) == true) {
		this->get(layer) = (this->get(layer).array() < minimum).select(minimum, this->get(layer));
		result = true;
	}

	return result;
}

bool SensorGrid::SetMax(const std::string& layer, float maximum) {
	bool result = false;

	if(this->Exists(layer) == true) {
		this->get(layer) = (this->get(layer).array() > maximum).select(maximum, this->get(layer));
		result = true;
	}

	return result;
}

bool SensorGrid::SetMinMax(const std::string& layer, float minimum, float maximum) {
	return this->SetMin(layer, minimum) & this->SetMax(layer, maximum);
}

void SensorGrid::Update(const std::string& layer, sensor_msgs::LaserScan& msg, float radius) {

	float angle, x, y;
	std::vector<float>::iterator itr;
	std::vector<float>::iterator iti;

	angle = msg.angle_min;

	if(msg.intensities.empty()) {
		msg.intensities = std::vector<float>(msg.ranges.size(), 1.0f);
	}

	iti = msg.intensities.begin();
	
	for (itr = msg.ranges.begin(); itr != msg.ranges.end(); ++itr) {
					
		// Skip ranges with inf value
		if(std::isinf(*itr) == false) {
			
			// Get cartesian cohordinates -> To be added: position of the kinect
			x = (*itr + radius)*cos(angle);
			y = (*itr + radius)*sin(angle);

			grid_map::Position position(x, y);

			// Skip positions outside the grid range
			if(this->isInside(position)) {
					
				// Fill the grid cell if ranges are between the min/max limits
				if ( ((*itr) > msg.range_min) & ((*itr) < msg.range_max)) {
					this->atPosition(layer, position) = *iti;
				} else {
					this->atPosition(layer, position) = 0.0f;
				}

			}
		}
		
		angle += msg.angle_increment;
		iti++;
	}
}

void SensorGrid::Update(const std::string& layer, sensor_msgs::PointCloud& msg) {

	grid_map::Position position;
	grid_map::Index    index;

	unsigned int idx = 0;

	for(auto it = msg.points.begin(); it != msg.points.end(); ++it) {
	
		position = grid_map::Position((*it).x, (*it).y);

		if(this->isInside(position)) {
			for(auto iti = msg.channels.begin(); iti != msg.channels.end(); ++iti) {
				if((*iti).name.compare("strength") == 0) {
					this->atPosition(layer, position) = (*iti).values[idx];
				}
			}
		}

		idx++;

	}
}

void SensorGrid::Update(const std::string& layer, geometry_msgs::Point32& msg) {

	grid_map::Position position;
	position = grid_map::Position(msg.x, msg.y);
	
	if(this->isInside(position)) {
		this->atPosition(layer, position) = 1.0f;
	}
}

void SensorGrid::Update(const std::string& layer, grid_map::Matrix& data) {
}

//bool SensorGrid::Transform(const std::string& layer, const std::string& parent) {
//
//	grid_map::Matrix& data = this->get(layer);
//	grid_map::Position position;
//	bool result = true;
//	tf::StampedTransform transform;
//
//	for(grid_map::GridMapIterator it(*this); !it.isPastEnd(); ++it) {
//		const grid_map::Index index(*it);
//		
//		if(data(index(0), index(1)) != 0) {
//		
//			try {
//				this->rostf_listener_.lookupTransform(this->GetFrame(), parent, ros::Time(0), transform);
//			} catch (tf::TransformException &ex) {
//				ROS_WARN_THROTTLE(10, "%s", ex.what());
//				result = false;
//				break;
//			}
//
//
//			// do something
//       }
//
//	}
//
//	return result;
//
//}


	}
}



#endif
