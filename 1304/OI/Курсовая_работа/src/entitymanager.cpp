//
//  entitymanager.cpp
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#include "entitymanager.h"
#include "entity.h"

EntityManager* EntityManager::sharedInstance()
{
    static EntityManager instance;
    
    return &instance;
}

void EntityManager::registerEntity(Entity *entity)
{
    _entityMap[entity->ID()] = entity;
}

Entity* EntityManager::getEntityFromID(int id) const
{
    return _entityMap.at(id);
}

void EntityManager::removeEntity(Entity *entity)
{
//    _entityMap.at(entity->ID()) = nullptr;
    _entityMap.erase(entity->ID());
}
