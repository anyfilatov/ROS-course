//
//  State.h
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 12/08/16.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef State_h
#define State_h

struct Telegram;
template <class Q>
class State {
protected:
    int once;
    
public:
    
    virtual void enter(Q*) = 0;
    
    virtual void execute(Q*) = 0;
    
    virtual void exit(Q*) = 0;
    
    virtual ~State(){}
    
    virtual bool onMessage(Q*, const Telegram&) { return false; }
};

#endif /* State_h */
