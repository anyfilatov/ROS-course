//
//  healthboostpowerup.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef healthboostpowerup_h
#define healthboostpowerup_h
#include "powerup.h"
#include "robot.h"

#define HEALTH_BOOST 70

class HealthBoostPowerUp : public PowerUp<Robot> {
public:
    
    HealthBoostPowerUp(Robot* robot)
    {
        _name = "health_boost";
        robot->p_health += HEALTH_BOOST;
    }
    
};


#endif /* healthboostpowerup_h */
