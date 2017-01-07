//
//  movingentity.hpp
//  CrabMan
//
//  Created by Imhoisili Otokhagua on 06/09/2016.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef movingentity_h
#define movingentity_h

#include "entity.h"
//#include "geometry_msgs/Point.h"
//#include "geometry_msgs/Pose.h"

class MovingEntity : public Entity
{
protected:

	geometry_msgs::Point position;
	int speed;
	
    
public:
    
    MovingEntity(   );
    
   
};

#endif /* movingentity_h */
