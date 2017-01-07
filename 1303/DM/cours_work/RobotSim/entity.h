//
//  entity.h
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef entity_h
#define entity_h

struct Telegram;
class Entity {
protected:
    int _id;
public:
    virtual bool handleMessage(const Telegram& msg) = 0;
    int ID()const { return _id; }
    virtual void update() = 0;
};

#endif /* entity_h */
