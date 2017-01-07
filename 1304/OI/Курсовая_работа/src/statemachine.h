//
//  statemachine.h
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 12/08/16.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef statemachine_h
#define statemachine_h

#include "allstates.h"

template <class Q>
class StateMachine {
private:
    
    Q* _owner;
    
    State<Q>* _currentState;
    
    State<Q>* _previousState;
    
    ///this state logic is called every time the FSM is updated
    State<Q>* _globalState;
    
public:
    StateMachine(Q* owner):_owner(owner), _currentState(nullptr), _previousState(nullptr), _globalState(nullptr) {}
    
    void setCurrentState(State<Q>* s) { _currentState = s; }
    void setGlobalState(State<Q>* s) { _globalState = s; }
    void setPreviousState(State<Q>* s) { _previousState = s; }
    
    void update() const
    {
        if (_globalState) _globalState->execute(_owner);
        
        if (_currentState) _currentState->execute(_owner);
    }
    
    void changeState(State<Q>* newState)
    {
        assert(newState && "<StateMachine::ChangeState>: trying to change to a null state");
        
        _previousState = _currentState;
        
        _currentState->exit(_owner);
        
        _currentState = newState;
        
        _currentState->enter(_owner);
    }
    
    void revertToPreviousState()
    {
        changeState(_previousState);
    }
    
    State<Q>* currentState() const { return _currentState; }
    State<Q>* previousState() const { return _previousState; }
    State<Q>* globalState() const { return _globalState; }
    
    bool isInState(const State<Q>& st) const
    {
        return _currentState == st;
    }
    
    bool handleMessage(const Telegram& msg)
    {
        if (_currentState && _currentState->onMessage(_owner, msg))
        {
            return true;
        }
        
        if (_globalState && _globalState->onMessage(_owner, msg))
        {
            return true;
        }
        
        return false;
    }
};


#endif /* statemachine_h */
