#include "battleteam.h"
#include "robotdecorator.h"

BattleTeam::BattleTeam(BattleField *bf, glm::vec3 startPos):_battleField(bf), _startPosition(startPos)
{
	_stateMachine = new StateMachine<BattleTeam>(this);
	create();
}

void BattleTeam::create()
{
	_teamRobots.push_back(new SpeedBoost(new Robot(_battleField, glm::vec3(-2, 0, 0) + _startPosition)));
	_teamRobots.push_back(new HealthBoost(new Robot(_battleField, _startPosition)));
	_teamRobots.push_back(new RangeBoost(new Robot(_battleField, glm::vec3(2, 0, 0) + _startPosition)));
	_teamRobots.push_back(new RangeBoost(new Robot(_battleField, glm::vec3(4, 0, 0) + _startPosition)));
	_teamRobots.push_back(new RangeBoost(new Robot(_battleField, glm::vec3(6, 0, 0) + _startPosition)));
}

void BattleTeam::update()
{
	for(int i = 0; i < _teamRobots.size(); i++){
		_teamRobots[i]->update();	
	}
}
