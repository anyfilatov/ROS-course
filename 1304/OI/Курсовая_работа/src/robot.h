//
//  robot.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef robot_h
#define robot_h

#include "robotbase.h"
#include "statemachine.h"

class BattleField;
class Robot : public RobotBase {
protected:
    StateMachine<Robot>* _stateMachine;
    BattleField* _battleField;    
	
public:
    
    Robot( BattleField* bf = nullptr, glm::vec3 position = glm::vec3(0));

	StateMachine<Robot>* getFSM() const;
    
    bool handleMessage(const Telegram & msg) override;
    
    void shoot() override;
    
    void advance() override;
    
    void update() override;

	void updateRos() override;

	void spawn(std::string modelName = "/home/imhoisi/.gazebo/models/pioneer3at/model.sdf") override;
};

#endif /* robot_h */
