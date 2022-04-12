# particle_filter_project

## Implementation Plan
 - How you will initialize your particle cloud (initialize_particle_cloud)?
   - Q: what is `map` initialized from? Can we tell what positions are out of bounds from it?
   - The particle cloud is an array of `Particle`s that represents an estimation of the robots location
   - In the beginning this cloud should look like a random distribution of poses within the bounds of the map.
   - In order to generate this cloud, for every particle in our cloud
     - we generate a random coordinate within the bounds of the map,
     - a random yaw between 0 and 2pi radians,
     - and assign it to that particle's pose.
     - Then we should assign the particle a random weight between 0 and 1
   - Once we have this randomized cloud, we normalize the weights across it.
   - At this point we've initialized a random particle cloud
 - How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
   - Calculate bel|(x_t) for every particle given the robot's reported movement and the particles previous position x_{t-1}
   - Update each Poses and weight accordingly (these weights should reflect bel| and shouldn't be normalized)
   - For every particle in the particle cloud, we update the Pose of the particle to reflect the robot's reported change in linear position and yaw.
   - If the resulting Pose is outside the bounds of the map (moves off the map, or through a wall), we assign it a weight of 0.
 - How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?
   - We will need to implement some form of Bayes Filter that, for each particle in our cloud, can update the probability of the robot being in its specified position, given what it just scanned
   - In order to do this we'll need to have some concept of odometry and scanning accuracy
 - How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
   - 
 - How you will update the estimated pose of the robot (update_estimated_robot_pose)?
 - How you will incorporate noise into your particle filter localization?

## Timeline

A brief timeline sketching out when you would like to have accomplished each of the components listed above.