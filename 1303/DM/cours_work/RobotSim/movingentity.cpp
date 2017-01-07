//
//  movingentity.cpp
//  CrabMan
//
//  Created by Imhoisili Otokhagua on 06/09/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#include "movingentity.h"

MovingEntity::MovingEntity(int ID, Model* model, glm::vec3 position, float rx, float ry, float rz, float scale, int textureIndex)
Entity(model, position, rx, ry, rz, scale, textureIndex)
{
    m_maxSpeed = 50;
    _id = ID;
}

float MovingEntity::getMaxSpeed() const
{
    return m_maxSpeed;
}

glm::vec3 MovingEntity::getVelocity() const
{
    return m_velocity;
}

glm::vec3 MovingEntity::getHeading()
{
    //calculate heading
    m_heading.x = cosf(glm::radians(_ry) * cosf(glm::radians(_rx)));
    m_heading.y = glm::sin(glm::radians(-_rx));
    m_heading.z = glm::cos(glm::radians(_rx)) * glm::sin(glm::radians(_ry));
    return m_heading;
}

glm::vec3 MovingEntity::getSide() const
{
    return m_side;
}
