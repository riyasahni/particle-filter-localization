# particle_filter_project

Names: Andre Dang, Riya Sahni

* To do initialize_particle_cloud, we will make completely random guesses for the robot's location, given all possible locations that the robot can be in. This is how we create all of our particles.
* To do update_particles_with_motion_model, based on how the robot moves and orients itself, we can simply add this data to the linear and angular attributes of the particles.
* To do update_particle_weights_with_measurement_model, we will use the sensor data and we will mathematically figure out what the hypothetical sensor readings are for those particles, comparing these data and therefore giving particles varying amounts of weight.
* To normalize_particles we will simply take these particle weights and take each computed weight, dividing it by the total weight. That way, the normalized weights will add up to 1 all together. To resample_particles, we will take these weights and using these weights, which are really just probabilities, we can use something like a random number generator to generate the same number of particles
* We update_estimated_robot_pose by taking those particle weights, with higher weighted particles by definition having a higher probability of surviving to the next generation of particles. By doing so repeatedly, we generate more consolidated clusters, reflecting how we have higher confidence in the robot's estimated position.
* We will construct a single range of values to add or subtract to the linear and angular values of both the particles and the sensor data.

# Timeline

We would like to have finished initialize_particle_cloud and update_particles_with_motion_model by Wednesday/Thursday, as well as having started update_particle_weights_with_measurement_model by the end of the week. In the second week, we would like to have done update_particle_weights_with_measurement_model and normalize_particles by the beginning of the week. We then want to finish resample_particles and update_estimated_robot_pose by wednesday/thursday of the second week of the project.