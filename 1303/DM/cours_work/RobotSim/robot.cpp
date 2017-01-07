//
//  robot.cpp
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#include "robot.h"
#include "powerup.h"

Robot::Robot(int pid)
{
    _id = pid;
    
    _stateMachine = new StateMachine<Robot>(this);
}

StateMachine<Robot>* Robot::getFSM() const
{
    return _stateMachine;
}

bool Robot::handleMessage(const Telegram & msg)
{
    return _stateMachine->handleMessage(msg);
}

void Robot::addBehaviour(PowerUp<Robot> *powerup)
{
    _powerups.insert(powerup);
}

void Robot::shoot()
{
    //shoot
}

void Robot::advance()
{
    //tells the robot to move around in the battlefield
}

void Robot::update()
{
    _stateMachine->update();
    executeCustomBehaviours();
}

void Robot::executeCustomBehaviours()
{
    for(auto *b : _powerups)
    {
        b->execute(this);
    }
}
