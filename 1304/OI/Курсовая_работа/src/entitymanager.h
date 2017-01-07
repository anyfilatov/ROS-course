//
//  entitymanager.hpp
//  StateMachineAI
//
//  Created by Imhoisili Otokhagua on 19/08/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef entitymanager_h
#define entitymanager_h

#include <map>

class Entity;
class EntityManager {
private:
    typedef std::map<int, Entity*> EntityMap;
    
    EntityMap _entityMap;
    
    EntityManager() {}
    EntityManager(const EntityManager&);
    EntityManager& operator=(const EntityManager &);
    
public:
    
    static EntityManager* sharedInstance();
    
    /** stores a pointer to the entity in the vector _entities at the position
     indicated by the entity's id*/
    void registerEntity(Entity *entity);
    
    Entity* getEntityFromID(int id) const;
    
    void removeEntity(Entity* entity);
};


#endif /* entitymanager_h */
#define EntityMan EntityManager::sharedInstance()
