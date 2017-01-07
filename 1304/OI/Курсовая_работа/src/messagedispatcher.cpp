//
//  messagedispatcher.cpp
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#include "messagedispatcher.h"
#include "entitymanager.h"
#include "clock.h"

MessageDispatcher* MessageDispatcher::sharedInstance()
{
    static MessageDispatcher instance;
    
    return &instance;
}

void MessageDispatcher::discharge(Entity *pReceiver, const Telegram &msg)
{
    pReceiver->handleMessage(msg);
}

void MessageDispatcher::dispatchMessage(double delay,
                                        int    sender,
                                        int    receiver,
                                        int    msg,
                                        void   *extraInfo)
{
    Entity* pReceiver = EntityMan->getEntityFromID(receiver);
    
    Telegram telegram {0, sender, receiver, msg, extraInfo};
    
    if (delay <= 0.0)
    {
        discharge(pReceiver
                  , telegram);
    }
    else
    {
        double currentTime = Clock::getCurrentTime();
        
        telegram._dispatchTime = currentTime + delay;
        
        _priorityQ.insert(telegram);
    }
}

void MessageDispatcher::dispatchDelayedMessage()
{
    time_t currentTime = Clock::getCurrentTime();
//    long currentTimeInSecs = currentTime/1000;
    
//    std::cout << "Time is: " << ctime(&currentTimeInSecs) << std::endl;
    while ( (_priorityQ.begin()->_dispatchTime < currentTime) && (_priorityQ.begin()->_dispatchTime > 0) && (_priorityQ.size() > 0) )
    {
        Telegram telegram = *_priorityQ.begin();
        
        Entity* pReceiver = EntityMan->getEntityFromID(telegram._receiver);
        
        discharge(pReceiver, telegram);
        
        _priorityQ.erase(_priorityQ.begin());
    }
}
