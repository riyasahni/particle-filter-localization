#!/usr/bin/env python3

import rospy
import copy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random, uniform, choices, gauss

from likelihood_field import *

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


#def draw_random_sample(particle_cloud, num_particles):
# We commented this function out because we figured we could just put the code
# in resample
#
#    TODO
#    return
#

class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 1000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # generate the likelihood field
        self.likelihood = LikelihoodField()

        # give things enough time to generate
        rospy.sleep(5)
        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data


    def initialize_particle_cloud(self):

        counter = 0
        # create counter to generate self.num_particles # of particles
        while counter < self.num_particles:
            while True:
                # keep regenerating random coordinates for new particle until the random coordinates land on a valid, available spot
                randx = uniform(self.map.info.origin.position.x, self.map.info.origin.position.x+self.map.info.width*self.map.info.resolution)
                randy = uniform(self.map.info.origin.position.y, self.map.info.origin.position.y+self.map.info.height*self.map.info.resolution)
                # convert coordinates to pixel units
                pixelrandx = (randx - self.map.info.origin.position.x)  / self.map.info.resolution
                pixelrandy = (randy - self.map.info.origin.position.y) / self.map.info.resolution
                # stop regenerating random coordinates for new particle once coordinates his a valid spot
                if self.map.data[int(pixelrandx) * self.map.info.width + int(pixelrandy)] == 0:
                    break

            z = 0
            # construct new point with random coordinates
            randPoint = Point(randy, randx, z)
            # select random particle direction for particle
            randDirEuler = uniform(0, 2*np.pi)
            # conver euler to quaternion
            convertedValues = quaternion_from_euler(0.0, 0.0, randDirEuler)
            randDir = Quaternion(convertedValues[0], convertedValues[1], convertedValues[2], convertedValues[3])
            # create random pose with random position and random direction
            randPose = Pose(randPoint, randDir)
            # create new random particle with average weight
            particle_weight = round(1/self.num_particles, 5)
            sampleParticle = Particle(randPose, particle_weight)
            self.particle_cloud.append(sampleParticle)
            # increase counter to generate another particle
            counter +=1

        # normalize nad publish particle cloud
        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0

        # compute the total weight to know what to divide by
        totalweight = 0
        for p in self.particle_cloud:
            totalweight += p.w

        # divide each particle by the total weight
        for p in self.particle_cloud:
            p.w = p.w/totalweight

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        # initialize empty array for particle weights
        weightsarr = []

        # fill array with weights of existing particles
        for i in self.particle_cloud:
            weightsarr.append(i.w)

        # randomly pick particles based on weights
        random_list = choices(self.particle_cloud, weights = weightsarr, k = self.num_particles)
        self.particle_cloud = []

        # deep copy particles into the array, rather than pointing to instances
        for p in random_list:
            self.particle_cloud.append(copy.deepcopy(p))

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                # the function we use in resample does not require normalization of weight
                #self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        totalx = 0
        totaly = 0
        totalangle = 0

        # iterate through all the particles and get the totals for these values
        for p in self.particle_cloud:
            totalx += p.pose.position.x
            totaly += p.pose.position.y
            # the z value is the only one we care about
            totalangle += get_yaw_from_pose(p.pose)

        # get the averages
        avgx = totalx / self.num_particles
        avgy = totaly / self.num_particles
        avgangle = totalangle / self.num_particles

        # make the new point and quaternion
        avgPoint = Point(avgx, avgy, 0)
        avgArray = quaternion_from_euler(0, 0, avgangle)
        avgQuat = Quaternion(avgArray[0], avgArray[1], avgArray[2], avgArray[3])

        # make the new pose and update the estimate
        newPose = Pose(avgPoint, avgQuat)
        self.robot_estimate = newPose

    def update_particle_weights_with_measurement_model(self, data):
        # Monte Carlo Localization (MCL) ALgorithm

        # Generate the lists of angles in radians and degrees
        lidar_angles = [np.pi/4, np.pi/2, .75*np.pi, np.pi, 1.25*np.pi, 1.5*np.pi, 1.75*np.pi, 2*np.pi]
        lidar_angles_deg = [45, 90, 135, 180, 225, 270, 315, 360]
        robot_sensor_distances = []
        # collect robot's sensor measurements for given angles:
        for angle in lidar_angles_deg:
            robot_sensor_distances.append(data.ranges[angle - 1])
        # compute the max distance for cases when dist=nan, set to the max
        max_dist = max(robot_sensor_distances)

        # loop through each particle
        for p in self.particle_cloud:
            # start with the weight at 1
            q = 1

            # set an index to iterate through the lidar_angles
            index = 0
            # loop through each laser range finder measurement recieved by robot
            for k in robot_sensor_distances:
                # compute the projected x and y using the formula
                p_projected_x = p.pose.position.x + k*math.cos(get_yaw_from_pose(p.pose) + lidar_angles[index])
                p_projected_y = p.pose.position.y + k*math.sin(get_yaw_from_pose(p.pose) + lidar_angles[index])

                # built-in func from likelihood_field.py, to compute the
                # distance from the nearest obstacle
                dist = self.likelihood.get_closest_obstacle_distance(p_projected_x, p_projected_y)

                # if dist is nan, the projection is out of bounds, so we make
                # the dist the max dist in the array so that the weight will
                # be low, as this is not a good prediction if the projection
                # is out of bounds
                if math.isnan(dist) == True:
                    dist  = max_dist # dist = max distance
                q = q*compute_prob_zero_centered_gaussian(dist, 0.1)
                index += 1
            p.w = q

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        # caluclate robots "up" and "sideways" changes based on its 'x' and 'y' movements
        delta_up = curr_x - old_x
        delta_sideways = curr_y - old_y

        # check if robot is traveling backwards, flag_dir = -1
        flag_dir = 1
        if delta_up < 0:
            flag_dir = -1
        # calculate distance robot travels (hypotenuse)
        rob_hypot = math.sqrt(delta_up**2 + delta_sideways**2)
        # calculate robot's change in yaw
        delta_yaw = curr_yaw - old_yaw

        for p in self.particle_cloud:
            # makes some noise for position and angle
            move_noise_x = gauss(0, .069)
            move_noise_y = gauss(0, .069)
            move_noise_angle = gauss(0, np.pi/36)
            # for the partcle p to mimic a "sideways" movement from the robot:
            p_yaw = get_yaw_from_pose(p.pose)
            # for the particle p to mimix an "up"  movement from the robot:
            p_side = rob_hypot*math.sin(p_yaw + delta_yaw)
            p_up = rob_hypot*math.cos(p_yaw + delta_yaw)
            # update particle direction from robot's updated yaw
            p_new_yaw = p_yaw + delta_yaw + move_noise_angle
            p_new_yaw_quat = quaternion_from_euler(0.0, 0.0, p_new_yaw)
            p_new_dir = Quaternion(p_new_yaw_quat[0], p_new_yaw_quat[1], p_new_yaw_quat[2], p_new_yaw_quat[3])
            # set new particle pos. and yaw based off robot's movements:
            # the 'x' direction is UP
            p.pose.position.x = p.pose.position.x + (p_up+move_noise_x)*flag_dir
            # the 'y' direction is SIDEWAYS
            p.pose.position.y = p.pose.position.y + (p_side+move_noise_y)*flag_dir
            # particle's new yaw
            p.pose.orientation = p_new_dir

if __name__=="__main__":
    pf = ParticleFilter()

    rospy.spin()









