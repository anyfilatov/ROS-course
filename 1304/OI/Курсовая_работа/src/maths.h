//
//  maths.h
//  CrabMan
//
//  Created by Imhoisili Otokhagua on 13/04/16.
//  Copyright © 2016 TriKomand. All rights reserved.
//

#ifndef maths_h
#define maths_h

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/gtx/perpendicular.hpp"
#include "glm/gtc/type_ptr.hpp"

#include <random>
#include <time.h>

class Maths {
private:

public:
    
    static glm::mat4 createTransformationMatrix(glm::vec3 translation, float rx, float ry, float rz, float scale)
    {
        glm::mat4 mMatrix = glm::mat4();
        mMatrix = glm::translate(mMatrix, translation);
        mMatrix = glm::rotate(mMatrix, rx, glm::vec3(1, 0, 0));
        mMatrix = glm::rotate(mMatrix, ry, glm::vec3(0, 1, 0));
        mMatrix = glm::rotate(mMatrix, rz, glm::vec3(0, 0, 1));
        mMatrix = glm::scale(mMatrix, glm::vec3(scale, scale, scale));
        
        return mMatrix;
    }
    
    static glm::vec3 interpolate(glm::vec3 x1, glm::vec3 x2, float dt)
    {
        glm::vec3 diff = x2 - x1;
        return x1 + (dt * diff);
    }
    
    static float interpolate(float x1, float x2, float dt)
    {
        float diff = x2 - x1;
        return x1 + (dt * diff);
    }
    
    static glm::vec2 resultant(glm::vec2 v1, glm::vec2 v2)
    {
//        float xs = v1.x + v2.x;
//        float ys = v1.y + v2.y;
//        return sqrt(xs*xs + ys*ys);
        return v1 + v2;
    }
    
    static glm::vec2 resolve(float v, glm::vec2 x1, glm::vec2 x2)
    {
        glm::vec2 resolution;
        float dot = glm::dot(x1, x2);
        float theta = glm::acos(dot);
        resolution.x = v * dot;
        resolution.y = v * glm::sin(theta);
        return resolution;
    }
    
    static void truncate(glm::vec3 &vec, float value)
    {
        vec.x = vec.x > value ? value : vec.x;
        vec.y = vec.y > value ? value : vec.y;
        vec.z = vec.z > value ? value : vec.z;
    }

	static std::default_random_engine randomEngine;	
	static int getRandomNumber(int min = 1, int max = 100)
	{
		std::uniform_int_distribution<int> intDistribution(min, max);
		
		return intDistribution(randomEngine);
	}
};

#endif /* maths_h */
