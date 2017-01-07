//
//  movingentity.hpp
//  CrabMan
//
//  Created by Imhoisili Otokhagua on 06/09/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef movingentity_h
#define movingentity_h

#include "entity.h"
#include "gazebo_msgs/SpawnModel.h"
#include <string>

class MovingEntity : public Entity
{
protected:
    glm::vec3 m_velocity;
    
    glm::vec3 m_heading;
    
    glm::vec3 m_side;
    
    float m_mass;
    
    float m_maxSpeed;
    
    float m_maxForce;
    
    float m_maxTurnRate;
    
    float _rx;
    float _ry;
    float _rz;

	ros::NodeHandle m_nodeHandle;
	ros::Publisher m_publisher;
	gazebo_msgs::ModelState m_modelStateMsg;

	virtual void spawn(std::string) {}
    
public:
    
    MovingEntity(glm::vec3 position = glm::vec3(0, 0, 0), float rx = 0.0f, float ry = 0.0f, float rz = 0.0f);
    
    float getMaxSpeed() const;
    
    glm::vec3 getVelocity() const;
    
    glm::vec3 getHeading();
    
    glm::vec3 getSide() const;

};

#endif /* movingentity_h */
