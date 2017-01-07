//
//  powerup.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef powerup_h
#define powerup_h
#include <string>

template <class Q>
class PowerUp {
protected:
    std::string _name = "default";
public:
    virtual void execute(Q*){};
};

#endif /* powerup_h */
