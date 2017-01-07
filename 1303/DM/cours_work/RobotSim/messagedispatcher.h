//
//  messagedispatcher.h
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef messagedispatcher_h
#define messagedispatcher_h
#include <set>
#include "messagetypes.h"
#include "telegram.h"
#include "entity.h"

class MessageDispatcher {
private:
    std::set<Telegram> _priorityQ;
    
    void discharge(Entity* pReceiver, const Telegram& msg);
    
    MessageDispatcher() {}
    
public:
    
    static MessageDispatcher* sharedInstance();
    
    void dispatchMessage(double delay,
                         int    sender,
                         int    receiver,
                         int    msg,
                         void*  extraInfo);
    
    void dispatchDelayedMessage();
};

#endif /* messagedispatcher_h */
#define Dispatch MessageDispatcher::sharedInstance()
