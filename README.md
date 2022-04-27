# particle_filter_project
Alex Miller and Josh Garza

## Flex hours used:
Alex: 13:08
Josh: 13:07

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
- [x] Get the map of the maze
- [x] Implement particle cloud initialization
- [x] Test if our initial clouds look random enough 
- [x] Implement particle normalization and resampling 
- [x] Implement guessing current position based on state of our particle cloud
   - Ended up taking position from average of top 5 percent of guesses
- [x] Implement probabilistic position updating
   - Design and implement a prototype model that calculates positional probability
   - Test if we can rule out some positions just based on the movement of our robot
   - Try doing this without incorporating noise, and then try incorporating noise
  
Next Week
- [x] Implement particle cloud weight updating using laser scan data
  - [x] Implement a prototype model
- Test functionality
- [x] Tweak noise and out of bounds parameters
- Test functionality
- If needed change models
- Test functionality
- Edit codebase to look nice, have more comments
- Edit writeup
- If time's left do EC


## Gif

![](particle_filter.gif)

If this is too low quality, please look at our mp4

## Objectives
The goal of this project is to implement an algorithm to estimate the state (position and orientation) of a robot in a known environment. Given a map of its environment and real-time sensor and odometry data, the goal is to simulate what the robot would see if it were at various different points and compare that to what it is actually seeing. After several iterations, we should be able to come up with an increasingly accurate estimation of its location in real-time.

## High Level Description
We begin by initializing a cloud of randomly (uniformly) distributed particles with random orientations. Each time new sensor data arrives from the robot, we follow a series of steps to update the particles’ positions/orientations, their weights, and the estimated position of the robot. To update the particles’ positions and orientations, we use the rotation, translation, rotation model with some added noise. To update the particles’ weights, we use a likelihood field to determine the probability that the nearest obstacles for a given particle (in the cardinal directions) matches up with the observed nearest objects of the robot. To update the estimated position of the robot, we simply take the unweighted mean of each particle’s position and orientation. We then normalize the particles’ weights and resample the particles to create a new particle cloud based on the updated weights. This new particle cloud serves as the basis for generating the estimated pose, both of which are then published to their respective ROS topics. 

## Steps
###	Initialization of particle cloud
Location: ``initialize_particle_cloud()``
Description: To initialize the particle cloud, we start by defining some local variables with information about the map of the environment, specifically the map data, resolution, width, height, and origin location. We then draw three sets of random samples of length ``num_particles``, one ranging from 0 - ``map_width`` (x values), one from from 0 - ``map_height`` (y values), and one from 0 - 1 (later to be multiplied by 2pi for our yaw values). We use the built-in function ``zip()`` to group the random x, y, and yaw values together to add to the particle clouds, and also initialize each weight to 1.0 if the particle is on the map, and 0.0 otherwise.

### Movement model
Location: ``update_particles_with_motion_model()``
Description: 
``model_odometry_translation()``: This function computes ``d_rot1``, ``d_trans``, and ``d_rot2`` for use in the rotation, translation, rotation particle update movement model. It subtracts the new and old x, y, and yaw values from the current and previous odometry data, then uses geometry to calculate the appropriate values.
``update_particles_with_motion_model()``: We begin by drawing three sets of ``num_particles`` random numbers on a gaussian distribution, which we then combine with ``d_rot1``, ``d_trans``, and ``d_rot2`` from ``model_odometry_translation()`` to add noise. We then combine the corresponding current pose values with the new movement values to update the pose of each particle to the new value. 

### Measurement model
Location: ``update_particles_with_measurement_model()``
Description: For each particle in ``self.particle_cloud``, we “look” in `self.measurement_resolution` directions (every 10th degree) to find the nearest obstacle using a likelihood field. We then calculate the probability of the reported nearest obstacle being the observed/nearest obstacle in each direction to determine the likelihood that that particle is the true location of the robot. The likelihood is set as the weight of each particle. 

### Resampling
Location: ``resample_particles()``
Description: We use ``np.random.choice()`` to resample the particles with replacement based on their normalized weights. First, we use list comprehension to assemble a list of each particles weight. We then feed that list and ``self.particle_cloud`` into ``np.random.choice()`` to compute a new particle cloud of the same length ``num_particles``. 

### Incorporation of noise
Location: ``update_particles_with_motion_model()``
Description: When we update the position attributes of each particle, we add noise from ``self.num_particles`` samples from a 0-centered normal distribution. This is sufficient for adding noise to our model.

### Updating estimated robot pose
Location: ``update_estimated_robot_pose()``
Description: To update the estimated pose of the robot we take the unweighted mean of the position and orientation of all particles in ``self.particle_cloud``. We initialize an empty ``Pose`` and iterate through all of the particles, adding the quotient of their position/orientation and ``self.num_particles`` to the new ``Pose``’s position/orientation. We then set ``self.robot_estimate`` equal to the new ``Pose``.

### Optimization of parameters
- ``self.num_particles``
  - The higher this variable was, the more particles we simulated, and the more accurate our model was. 
  - However, it also increased computational load since it meant processing more particles. 
  - We set this variable as high as possible without triggering errors from our transformers due to lag. Doing so maximized our models performance and accuracy.
  - We decided on a set of 7500 particles
- ``self.measurement_resolution``
  - The higher this variable was, the more particles we simulated, and the more accurate our model was. 
  - However, it also increased computational load since it meant processing sensor measurements for each particle.
  - We set this variable as high as possible without triggering errors from our transformers due to lag. Doing so maximized our models performance and accuracy.
  - We decided on a resolution of 90 measurements per particle per update step
- ``update_particles_with_motion_model``
  - We had to experiment with the bounds of the uniform noise we used. 
  - We settled on `0.1` for all our translation variables; any smaller did not seem to produce noticeable noise in the movement of our particles.
- ``self.z_hit and self.z_rand``
  - These parameters determined how sensitive our model was to reported likelihoods in our measurement model.
  - We had to find a balance between these variables s.t. our model didn't overly privilege any one particle, but also didn't overvalue the weights of inaccurate guesses
  - We settled on a balance of `z_hit` = .65 and `z_rand` = .35
- ``The standard deviation of our gaussian for computing distance likelihood``
  - This parameter determined how sensitive our model was to error in our likelihood field readings
  - If this value was too low then some particles were eliminated from the particle cloud too readily because they were assigned near zero weights
  - We settled on a value of `0.5` for this parameter
## Challenges
This project entailed many difficulties. We are working with a computationally intense model in limited environments, with a framework we're unfamiliar with.
Because of this, it was hard to test our implementation for bugs, recognize them, and come up with well-informed solutions. We spent more time than we probably should trying to guess what was wrong with our code
because we couldn't say why, for example, our particles seemed to move at a pace 10x that of our robot. We countered this by trying to test each part of our code
as methodically as possible and recorded observations so we didn't make repeat mistakes.
It was also challenging to balance so many different parameters in one model, and test them robustly, since their values could radically change the accuracy of the model, and testing was so slow.

## Future Work
There are a couple of things we would improve in our implementation. For one, we settled on using parameters that, loosely, 'worked.' It would be worth the additional time to try
and come to a more rigorous assessment of what parameters maximized the performance of our model. It would be nice to have some method to test and catalog results more rigorous than taking quick notes on scrap paper.

## Takeaways
Testing the correctness of predictive models is very challenging. It took a lot of time to implement, tweak, and test our implementation, and even now I am not sure how robust it is in the face of edge cases.
In the future I know it helps to think methodically through the effects of setting certain parameters to certain values and testing my predictions, in order to come to a greater understanding of my implementation.
