Spring 2020 CS 134 Computer Game Design and Programming Final Project: Lander

Team: Barely Surviving
Jessica Hoang
Justin Magadia

*Note: refer to src when pushing new files


Final Lander requirements:
- Game must have a goal with predefined constraints
    - Example goal: complete a landing 
    - Example constraint: amount of fuel you have
- Goals must have complexity
- Vehicle must be able to move with physics
    - WASD and arrow keys
    - Rotation around “Y” Axis can be implemented w/o physics
- Physics is required for at least one (1) visual effect and must be rendered using a shader
    - Example: rocket exhaust
    - Must include another demonstration of physics
- Must have collision detection with terrain
    - Must be resolved with a suitable impulse force 
        - Must have 1 bounding box on your vehicle, but your terrain should be spatially partitioned using an ochre or kdtree
- The player should be able to switch between camera views
    - “Tracking” camera = aimed at spacecraft from a fixed location
    - One onboard the spacecraft
    - Easycam = player can navigate anywhere over the surface
- Sound is required for effects (spacecraft thrust) and background sounds 
- Start your spacecraft in the air (as seen in lab 5)
- Have a 2D image background

Demo video: https://www.youtube.com/watch?v=QSbsvQw2ljQ&feature=youtu.be
