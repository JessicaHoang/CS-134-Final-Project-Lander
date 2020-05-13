//
//  Particle.h
//  Lander-Final
//
//  Created by Hoang on 5/13/20.
//

#ifndef Particle_h
#define Particle_h


#endif /* Particle_h */

#pragma once
#include "ofMain.h"

class ParticleForceField;
class Particle{
public:
    Particle();
    
    ofVec3f position;
    ofVec3f velocity;
    ofVec3f acceleration;
    ofVec3f forces;
    float damping;
    float mass;
    float   lifespan;
    float   radius;
    float   birthtime;
    void    integrate();
    void    draw();
    float   age();        // sec
    ofColor color;
    ofShader shader;
}
