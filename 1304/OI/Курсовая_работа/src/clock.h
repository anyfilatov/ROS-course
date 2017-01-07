//
//  clock.h
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef clock_h
#define clock_h

//how to time a c++ function

#include <iostream>
#include <chrono>
#include <ctime>
using namespace std;
using namespace std::chrono;

class Clock {
    Clock() = delete;
public:
    
    static time_t getCurrentTime()
    {
        auto now = system_clock::now();
        
        auto duration = now.time_since_epoch();
        
        auto millisec = duration_cast<milliseconds>(duration).count();
        
//        auto tt = system_clock::to_time_t(now);
        
        return millisec;
    }
};

#endif /* clock_h */


