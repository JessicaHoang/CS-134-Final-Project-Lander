//
//  Particle.cpp
//  Lander-Final
//
//  Created by Hoang on 5/13/20.
//

#include <stdio.h>
#include "Particle.h"

Particle::Particle(){
    
    // Initial state of particles
    //
    velocity.set(0, 0, 0);
    acceleration.set(0, 0, 0);
    position.set(0, 0, 0);
    forces.set(0, 0, 0);
    lifespan = -1;
    birthtime = 0;
    radius = .05;
    damping = .99;
    mass = 1;
    color = ofColor::red;
}

void Particle::draw(){
    ofSetColor(color);
    ofDrawSphere(position, radius);
}

void Particle::integrate(){
    
    // Checks framerate to avoid divide errors
    //
    float framerate = ofGetFrameRate();
    if(framerate < 1.0) return;
    
    // dt
    //
    float dt = 1.0 / framerate;
    
    // Update position based on velocity
    //
    position += (velocity * dt);
    
    // update acceleration with accumulated paritcles forces
        // remember :  (f = ma) OR (a = 1/m * f)
        //
        ofVec3f accel = acceleration;    // start with any acceleration already on the particle
        accel += (forces * (1.0 / mass));
        velocity += accel * dt;
    
        // add a little damping for good measure
        //
        velocity *= damping;
    
        // clear forces on particle (they get re-added each step)
        //
        forces.set(0, 0, 0);
}

//  return age in seconds
//
float Particle::age() {
    return (ofGetElapsedTimeMillis() - birthtime)/1000.0;
}

