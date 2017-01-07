//
//  robotdecorator.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 26/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef robotdecorator_h
#define robotdecorator_h

#include "robot.h"
#define EXTRA_SPEED 5
#define HEALTH_BOOST 70
#define RANGE_BOOST 10

class RobotDecorator: public RobotBase {
protected:
    RobotBase *owner = nullptr;
    
public:
    virtual void update() override = 0;
};

class SpeedBoost: public RobotDecorator {
    
public:
    SpeedBoost(RobotBase *robot) {
        this->owner = robot;
    }
    
    void update() override {
        owner->p_speed = DEFAULT_SPEED + EXTRA_SPEED;
		owner->update();
    }
};

class HealthBoost: public RobotDecorator {
    bool executed = false;
public:
    HealthBoost(RobotBase *robot) {
        owner = robot;
    }
    
    void update() override {
        if (executed == false) {
            owner->p_health += HEALTH_BOOST;
            executed = true;
        }
		owner->update();
    }
    
    void reset() {
        executed = false;
    }
};

class RangeBoost: public RobotDecorator {
    bool executed = false;
public:
    RangeBoost(RobotBase *robot) {
        owner = robot;
    }
    
    void update() override {
        if (executed == false) {
            owner->p_shootingDistance += RANGE_BOOST;
            executed = true;
        }
		owner->update();
    }
    
    void reset() {
        executed = false;
    }
};

#endif /* robotdecorator_h */
