//
//  telegram.h
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef telegram_h
#define telegram_h

#include <cmath>

struct Telegram
{
    double _dispatchTime;
    
    int _sender;
    
    int _receiver;
    
    int _message;
    
    void *_extraInfo;
    
    Telegram(double delay, int sender, int receiver, int message, void* xtraInfo)
    {
        _dispatchTime = delay;
        
        _sender = sender;
        
        _receiver = receiver;
        
        _message = message;
        
        _extraInfo = xtraInfo;
    }
    
    bool operator<(const Telegram& rhs) const
    {
        return this->_dispatchTime < rhs._dispatchTime;
    }
    
    bool operator==(const Telegram& rhs) const
    {
        double difference = fabs(this->_dispatchTime - rhs._dispatchTime);
        return this->_dispatchTime == rhs._dispatchTime || difference <= 0.25;
    }
    
};

#endif /* telegram_h */
