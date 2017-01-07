//
//  main.cpp
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#include <iostream>
#include "robot.h"
#include "allstrategies.h"
#include "entitymanager.h"
#include "messagedispatcher.h"


int main(int argc, const char * argv[]) {
    
    auto *r1 = new Robot(001);
    
    EntityMan->registerEntity(r1);
    
    while (true) {
        
        r1->update();
        
        Dispatch->dispatchDelayedMessage();
    }
    
    return 0;
}
