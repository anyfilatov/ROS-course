//
//  main.cpp
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#include <iostream>
#include "robot.h"
#include "battlefield.h"
#include "battleteam.h"
#include "allstrategies.h"
#include "entitymanager.h"
#include "messagedispatcher.h"

int main(int argc, char ** argv) {
    
	ros::init(argc, argv, "roboFight");
    
    auto battleField = new BattleField();
	auto teamA = new BattleTeam(battleField, glm::vec3(0, 5, 0));
	auto teamB = new BattleTeam(battleField, glm::vec3(0, -5, 0));
	
    
    while (ros::ok()) {
        

		teamA->update();
		teamB->update();
        Dispatch->dispatchDelayedMessage();
		
		ros::spinOnce();
    }
    
    return 0;
}
