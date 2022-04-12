# particle_filter_project

## Implementation Plan
 - How will you initialize your particle cloud (`initialize_particle_cloud`)?
   - The particle cloud is an array of `Particle`s that represents an estimation of the robot's location
   - In the beginning this cloud should look like a random distribution of poses within the bounds of the map.
   - In order to generate this cloud, for every particle in our cloud
     - we generate a random coordinate within the bounds of the map,
     - a random yaw between 0 and 2pi radians,
     - and assign it to that particle's pose.
     - Then we should assign the particle a weight of 1
   - Once we have this randomized cloud, we normalize the weights across it.
   - At this point we've initialized a random particle cloud
 - How will you update the position of the particles based on the movements of the robot (`update_particles_with_motion_model`)?
   - For every particle in our cloud, calculate a new position x_t given the robot's reported movement and the particle's previous position x_{t-1}.
   - Assign this position a probability based on the topology of our map. Any points that fall outside our map should be assigned a low, non-zero probability.
   - Update the pose of each particle using x_t and make the weight reflect the probability of the robot being at x_t
 - How you will compute the importance weights of each particle after receiving the robot's laser scan data?(`update_particle_weights_with_measurement_model`)?
   - After we update the positions in our particle cloud, for each particle we take the probability of the particle's pose.
   - We then use that probability and the topology of the map to determine the probability that the robot could have taken its most recent laser scan at that position.
   - Finally we update the weight of the particle to reflect that probability
 - How you will normalize the particles' importance weights (`normalize_particles`) and resample the particles (`resample_particles`)?
   - In order to normalize the weights of our particles, we alter their weights to all add up to one while remaining proportional to one another.
   - We resample by randomly redrawing the particles in our cloud from our current cloud in proportion to each particle's normalized weight.
   - Assign the new particles in our cloud the same weights and normalize them.
 - How you will update the estimated pose of the robot (`update_estimated_robot_pose`)?
   - The estimated pose of the robot should just be the particle in our cloud with the highest weight.
 - How you will incorporate noise into your particle filter localization?
   - Every time we calculate the new position of a particle, we should add some constant to our reported movement such that `abs(constant) <= noise`, where `noise` is some constant arrived at through testing.

## Timeline
Always:
- Fill out the writeup and write comments as we go

This week:
- Get the map of the maze
- Implement particle cloud initialization
  - Test if our initial clouds look random enough
- Implement particle normalization and resampling
  - This will be needed by any strategy we choose to implement
- Implement guessing current position based on state of our particle cloud
  - Straightforward and needed to test the reliability of the system
- Implement probabilistic position updating
  - Design and implement a prototype model that calculates positional probability
  - Test if we can rule out some positions just based on the movement of our robot
  - Try doing this without incorporating noise, and then try incorporating noise
  
Next Week
- Implement particle cloud weight updating using laser scan data
  - Implement a prototype model
- Test functionality
- Tweak noise and out of bounds parameters
- Test functionality
- If needed change models
- Test functionality
- Edit codebase to look nice, have more comments
- Edit writeup
- If time's left do EC
