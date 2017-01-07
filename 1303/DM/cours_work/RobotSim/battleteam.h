//
//  battleteam.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef battleteam_h
#define battleteam_h
#include <vector>
#include "entity.h"
#include "statemachine.h"

class Robot;
class BattleField;
class BattleTeam: Entity {
    StateMachine<BattleTeam>* _stateMachine;
    
protected:
    
    BattleField* _battleField;
    
    std::vector<Robot*> _teamRobots;
    
    Robot* _commander;
    
public:
    
};

#endif /* battleteam_h */
