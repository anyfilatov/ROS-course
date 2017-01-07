//
//  movingentity.cpp
//  CrabMan
//
//  Created by Imhoisili Otokhagua on 06/09/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#include "movingentity.h"

MovingEntity::MovingEntity(glm::vec3 position, float rx, float ry, float rz):
Entity(position),
_rx(rx),
_ry(ry),
_rz(rz)
{
    m_maxSpeed = 50;
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
