//
//  speedboostpowerup.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef speedboostpowerup_h
#define speedboostpowerup_h
#include "powerup.h"
#include "robot.h"

#define EXTRA_SPEED 5

class SpeedBoostPowerUp : public PowerUp<Robot> {
public:
    SpeedBoostPowerUp()
    {
        _name = "speed_boost";
    }
    
    void execute(Robot* robot)
    {
        robot->p_speed = DEFAULT_SPEED + EXTRA_SPEED;
    }
};


#endif /* speedboostpowerup_h */
