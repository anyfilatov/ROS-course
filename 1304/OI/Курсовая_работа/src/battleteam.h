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

class RobotBase;
class BattleField;
class BattleTeam: Entity {
    StateMachine<BattleTeam>* _stateMachine;
    
protected:
    
    BattleField* _battleField;
    
    std::vector<RobotBase*> _teamRobots;
    
    RobotBase* _commander;
	
	void create();    

	glm::vec3 _startPosition;

public:

	BattleTeam(BattleField* bf, glm::vec3 startPos = glm::vec3(0));

	bool handleMessage(const Telegram& msg) {}
	void update() override;
    
};

#endif /* battleteam_h */
