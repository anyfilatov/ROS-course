//
//  robot.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef robot_h
#define robot_h

#include <set>
#include "entity.h"
#include "statemachine.h"

#define DEFAULT_SPEED 20
#define DEFAULT_MAX_HEALTH 100
#define DEFAULT_SHOOTING_DISTANCE 5

template <class Q>
class PowerUp;

class BattleField;
class Robot : public MovingEntity {
protected:
    StateMachine<Robot>* _stateMachine;
    std::set<PowerUp<Robot>*> _powerups;
    
    BattleField* _battleField;
    
public:
    
    Robot(int pid);
    
    int p_speed = DEFAULT_SPEED;
    int p_health = DEFAULT_MAX_HEALTH;
    int p_shootingDistance = DEFAULT_SHOOTING_DISTANCE;
    // p_position
    // p_velocity
    
    StateMachine<Robot>* getFSM() const;
    
    bool handleMessage(const Telegram & msg) override;
    
    void addBehaviour(PowerUp<Robot>* powerup);
    
    void shoot();
    
    void advance();
    
    void update() override;
    
    void executeCustomBehaviours();
};


//****

enum RobotType {FAST, STRONG, SNIPER};
enum RobotTeam {A, B};

class Robot: public StateMachine
{
private:	
	static int id;
	geometry_msgs::Point position;
	RobotType type;
	RobotTeam team;
	int speed;
	unsigned int health;
	int shootingRange;

public:
	Robot(RobotType rty, RobotTeam rte  );
	void onhit();
	void shoot();
	void advance();
	void update();

}



#endif /* robot_h */
