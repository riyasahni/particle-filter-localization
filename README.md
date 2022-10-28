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

[gif1]
[gif2]

## Challenges

We faced a lot of challenges in trying to complete this project. Understanding how to use the sensor measurements to adjust the weight was conceptually tricky, and we referred to what we had studied in class to guide this, but the mathematical component was definitely a little difficult to understand. We then also dealt with a myriad of bug fixes, like figuring out how to properly check distances for being not a number, or simply using radians instead of degrees. We also had a longstanding issue where many of the distances were coming out as 0, and it took a long time to realize that this was because of the robot, not our code. We also did not understand that we had to make deep copies, as this was leading the particles to behave very erratically, and we were not sure why that was the case. Overall, the majority of our difficulties stemmed from just having trouble understanding small parts here and there, that made a large difference in our program's functionality.

## Future Work

If we had more time, it would be interesting to keep playing with the noise, to try to see what could change with that. We didn't get enough time to observe what more extreme values would lead to, and that would have been very interesting. It would also be interesting to see if we could get more particles on the field without having too much computation, so figuring out a way to optimize our code more.