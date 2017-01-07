//
//  entity.h
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef entity_h
#define entity_h
#include "maths.h"
#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"

struct Telegram;
class Entity {
protected:
    int _id;
    glm::vec3 m_position;
public:
    Entity(glm::vec3 position = glm::vec3(0)) : m_position(position) {
		_id = Maths::getRandomNumber();
	}
    virtual bool handleMessage(const Telegram& msg) = 0;
    int ID()const { return _id; }
    virtual void update() = 0;
    
    virtual glm::vec3 getPosition () { return m_position; }

};

#endif /* entity_h */
