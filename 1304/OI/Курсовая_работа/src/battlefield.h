//
//  battlefield.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef battlefield_h
#define battlefield_h

#include "entity.h"

class BattleField : public Entity {
    
public:

	BattleField():Entity(){}

	bool handleMessage(const Telegram& msg) override {}

	void update() override {}
};

#endif /* battlefield_h */
