# particle_filter_project

Names: Andre Dang, Riya Sahni

* To do initialize_particle_cloud, we will make completely random guesses for the robot's location, given all possible locations that the robot can be in. This is how we create all of our particles.
* To do update_particles_with_motion_model, based on how the robot moves and orients itself, we can simply add this data to the linear and angular attributes of the particles.
* To do update_particle_weights_with_measurement_model, we will use the sensor data and we will mathematically figure out what the hypothetical sensor readings are for those particles, comparing these data and therefore giving particles varying amounts of weight.
* To normalize_particles we will simply take these particle weights and take each computed weight, dividing it by the total weight. That way, the normalized weights will add up to 1 all together. To resample_particles, we will take these weights and using these weights, which are really just probabilities, we can use something like a random number generator to generate the same number of particles
* We update_estimated_robot_pose by taking those particle weights, with higher weighted particles by definition having a higher probability of surviving to the next generation of particles. By doing so repeatedly, we generate more consolidated clusters, reflecting how we have higher confidence in the robot's estimated position.
* We will construct a single range of values to add or subtract to the linear and angular values of both the particles and the sensor data.

## Timeline

We would like to have finished initialize_particle_cloud and update_particles_with_motion_model by Wednesday/Thursday, as well as having started update_particle_weights_with_measurement_model by the end of the week. In the second week, we would like to have done update_particle_weights_with_measurement_model and normalize_particles by the beginning of the week. We then want to finish resample_particles and update_estimated_robot_pose by wednesday/thursday of the second week of the project.

## Objectives

The goal of this project is to use Monte Carlo Localization to use particles to figure out where our robot is. Using movement data from the robot, knowledge of our environment, and sensor data, over time these particles will converge to tell us the location and orientation of the robot in the maze.

## High-level description

To solve this problem, we had to randomly place particles on the map, making sure that we put them in valid locations, meaning not on obstacles or outside the maze. We then had to keep track of the robot's movements and make the particles move in the direction according to its orientation. We then had to incorporate the robot's sensor data, and project what that would look like for the particles, and then computing the nearest wall for the distances projected onto these particles. We then used a zero centered gaussian to compute the probability using this distance. This was used to reassign particle weight and make good particles more likely to survive. We then resampled our particles using these adjusted weights and updated our estimated location and orientation of the robot, usign an average of all the particles that survived.

![ezgif com-gif-maker](https://user-images.githubusercontent.com/75453797/198522707-068304e7-81fd-4434-adc3-c620fd8f7ca5.gif)

![ezgif com-gif-maker-2](https://user-images.githubusercontent.com/75453797/198522727-54584720-00ce-4ef3-86da-2eac4f9f80e2.gif)

## Main Steps
Particle filter localization steps:
1) Initialization of particle cloud

Code location: We did the initialization of the particle cloud in the initialize_particle_cloud function on line 136. 

Functions/code description: First, we keep regenerating coordinates until we can find the position of a particle on a valid part of the grid. Then, we randomly generate a direction for our particle and add it to our particle cloud. We create self.num_particles number of particles.

2) Movement model

Code location: We did the movement model step in the update_particles_with_motion_model function on line 363.

Functions/code description: First, we calculate the hypotenuse movement of the robot using its delta_y, delta_x and change in yaw, and then, we calculate the total horizontal movement and total vertical movement of the particle to replicate the robot's movement, but from the particle's perspective. We use a flag to indicate whether the robot/particle moves forward or backward. In this function, we also add noise to the particle by randomly generating values using a Gaussian distribution to adjust the coordinates and yaw by.

3)  Measurement model

Code location: We did the measurement model step in the update_particle_weights_with_measurement_model function on line 323.

Functions/code description: First, we create an array of angles to fill an array of Lidar sensor measurements. Then, we project the sensor readings onto each particle in the particle cloud and compute the probability that the particle accurately reflects the robot's true position. This probability becomes the particle's new "weight", which is later used to resample and regenerate a new particle cloud.

4) Resampling

Code location: We did the resampling step in the resample_particles function on line 207. 

Functions/code description: First, we initialize an empty array and fill it with existing particle weights. Then, we use a Python built-in function to randomly choose a new list of particles given the weights of the particles from the initial particle cloud. Then, we deep copy these newly made particles into the emptied particle cloud array.

5) Incorporation of noise 

Code location: We added noise to our particles in the update_particles_with_motion_model function from lines 388-390, line 397, and lines 402 and 404. 

Functions/code description: We used a gaussian distribution to generate values between -0.069 and 0.069 to add to the vertical and horizontal movements of the particles, and we also generated a pi/36 radian distribution to add noise to the directions of the particles. We experimented a lot with these numbers to find optimal noise levels without negatively impacting the estimated robot's pose.

6) Updating estimated robot pose

Code location: We estimated our robot pose in the update_estimated_robot_pose function on line 296.

Functions/code description: We found the average yaw and coordinate position from our particle cloud and used that to define our estimated robot pose's position. As the particles become more clustered from resampling, the robot's estimated pose updates and more accurately reflects the robots true position.

7) Optimization of parameters

Code location: Overall, we experimented a lot with the parameters to find out what our optimal noise levels should be (see (6) for code location), what the optimal sd should be for our gaussian distributions (see (5) for code location), and we also used functions like get_closest_obstace_distance to optimize the results from our projections onto particles (line 351).

Functions/code description: Initially, we had our noise level set to 0.5, which we noticed was interfering with the particle filter's ability to predict the robot's pose. At the same tie, we noticed that setting noise to 0.05 for the particle's coordinate positions was not large enough to catch subtleties in the robot's movements and to help the pose predictor's adjust accordingly. We did a lot of trial-and-error with the noise levels and the gaussian distribution's sd to find optimal parameters for the particles' rates of adjustment and redistributions.
  
## Challenges

We faced a lot of challenges in trying to complete this project. Understanding how to use the sensor measurements to adjust the weight was conceptually tricky, and we referred to what we had studied in class to guide this, but the mathematical component was definitely a little difficult to understand. We then also dealt with a myriad of bug fixes, like figuring out how to properly check distances for being not a number, or simply using radians instead of degrees. We also had a longstanding issue where many of the distances were coming out as 0, and it took a long time to realize that this was because of the robot, not our code. We also did not understand that we had to make deep copies, as this was leading the particles to behave very erratically, and we were not sure why that was the case. Overall, the majority of our difficulties stemmed from just having trouble understanding small parts here and there, that made a large difference in our program's functionality.

## Future Work

If we had more time, it would be interesting to keep playing with the noise, to try to see what could change with that. We didn't get enough time to observe what more extreme values would lead to, and that would have been very interesting. It would also be interesting to see if we could get more particles on the field without having too much computation, so figuring out a way to optimize our code more.
