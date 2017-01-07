
#ifndef robotbase_h
#define robotbase_h

#include "movingentity.h"

#define DEFAULT_SPEED 20
#define DEFAULT_MAX_HEALTH 100
#define DEFAULT_SHOOTING_DISTANCE 5

class BattleField;
class RobotBase : public MovingEntity {
protected:    
public:
    
    RobotBase(BattleField* bf = nullptr, glm::vec3 position = glm::vec3(0));
    
    int p_speed = DEFAULT_SPEED;
    int p_health = DEFAULT_MAX_HEALTH;
    int p_shootingDistance = DEFAULT_SHOOTING_DISTANCE;
    
    virtual bool handleMessage(const Telegram & msg){}
    
    virtual void shoot(){}
    
    virtual void advance(){}
    
    virtual void update() = 0;

	virtual void updateRos(){}
};

#endif
