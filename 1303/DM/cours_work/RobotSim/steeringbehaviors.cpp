//
//  steeringbehaviors.cpp
//  CrabMan
//
//  Created by Imhoisili Otokhagua on 06/09/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#include "steeringbehaviors.h"
#include "Robot.h"

SteeringBehaviors::SteeringBehaviors(Robot *owner)
{
    m_pRobot = owner;
}

glm::vec3 SteeringBehaviors::calculate()
{
//    glm::vec3 totalForces = seek(m_pTargetPosition);
//    return seek(m_pTargetPosition);
//    return flee(m_pTargetPosition);
//    return arrive(m_pTargetPosition, 3);
    return pursue(m_pTargetEntity1);
//    return evade(m_pTargetEntity1);
}

glm::vec3 SteeringBehaviors::seek(const glm::vec3 &targetPosition)
{
    glm::vec3 desiredVelocity = glm::normalize(targetPosition - m_pRobot->getPosition()) * m_pRobot->getMaxSpeed();
    
    return desiredVelocity - m_pRobot->getVelocity();
}

glm::vec3 SteeringBehaviors::flee(const glm::vec3 &targetPosition)
{
    // only flee if the target is within 'panic distance'.
    const float PanicDistanceSq = 30.0f * 30.0f;
    if (glm::distance2(m_pRobot->getPosition(), m_pTargetPosition) > PanicDistanceSq) {
//        return glm::vec3(0.0f);
        auto stopVel = -m_pRobot->getVelocity() * 2.0f;
        return stopVel;
    }
    
    glm::vec3 desiredVelocity = glm::normalize(m_pRobot->getPosition() - targetPosition) * m_pRobot->getMaxSpeed();
    
    return desiredVelocity - m_pRobot->getVelocity();
}

glm::vec3 SteeringBehaviors::arrive(const glm::vec3 &targetPosition, float deceleration)
{
    glm::vec3 toTarget = targetPosition - m_pRobot->getPosition();
    
    //calculate the distance to the target position
    
    float dist = glm::length(toTarget);
    
    if (dist > 0) {
        const float decelerationTweaker = 0.3;
        
        // calculate the speed required to reach the target given the desired deceleration
        float speed = dist / (deceleration * decelerationTweaker);
        
        // make sure the velocity does not exceed the max
        speed = min(speed, m_pRobot->getMaxSpeed());
        
        // repeat seak
        glm::vec3 desiredVelocity = (toTarget * speed) / dist;
        
        return desiredVelocity - m_pRobot->getVelocity();
    }     
    return glm::vec3(0);
}

float turnAroundTime(Robot* agent, glm::vec3 targetPosition)
{
    auto toTarget = glm::normalize(targetPosition - agent->getPosition());
    
    float dot = glm::dot(agent->getHeading(), toTarget);
    
    const float coefficient = 0.5f;
    
    return (dot - 1.0f) * -coefficient;
}

glm::vec3 SteeringBehaviors::pursue(Robot *evader)
{
    // if the evader is ahead and facing the agent then seek the evader's current position
    auto toEvader = evader->getPosition() - m_pRobot->getPosition();
    
    float relativeHeading = glm::dot(m_pRobot->getHeading(), evader->getHeading());
    
    if ( glm::dot(toEvader, m_pRobot->getHeading()) > 0 && relativeHeading < -0.95) {
//        return seek(evader->getPosition());
        return arrive(evader->getPosition(), 5);
    }
    
    // Not considered ahead so we predict where the evader will be.
    // the look-ahead time is proportional to the distance between the evader
    // and the pursuer; and is inversely proportional to the sum of
    // the agents' velocities
    
    float lookAheadTime = toEvader.length() / (m_pRobot->getMaxSpeed() + 40.0f);
    lookAheadTime += turnAroundTime(m_pRobot, evader->getPosition());
    
    //now seek to the predicted future position of the evader
//    return seek(evader->getPosition() + evader->getVelocity() * lookAheadTime);
    
    return arrive(evader->getPosition() + evader->getVelocity() * lookAheadTime, 5);
}

glm::vec3 SteeringBehaviors::evade(Robot *pursuer)
{
    glm::vec3 toPursuer = pursuer->getPosition() - m_pRobot->getPosition();
    
    float lookAheadTime = toPursuer.length() / (m_pRobot->getMaxSpeed() + 40.0f);
    
    return flee(pursuer->getPosition() + pursuer->getVelocity() * lookAheadTime);
}

//glm::vec3 SteeringBehaviors::wander()
//{
//    // add a smal random vector to the target's position
//    m_WanderTarget += glm::vec3(rand() % -1 + 1);
//    
//    m_WanderTarget = glm::normalize(m_WanderTarget);
//    
//    m_WanderTarget *= m_dWanderRadius;
//    
//    auto targetLocal = m_WanderTarget + glm::vec3(m_dWanderDistance, 0, m_dWanderDistance);
//    
//    auto targetWorld =
//}


void SteeringBehaviors::setTarget(glm::vec3 target)
{
    m_pTargetPosition = target;
}

void SteeringBehaviors::setTargetAgent1(Robot *v1)
{
    m_pTargetEntity1 = v1;
}

void SteeringBehaviors::setTargetAgent2(Robot *v2)
{
    m_pTargetEntity2 = v2;
}
