#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from likelihood_field import LikelihoodField

import numpy as np
from numpy.random import random_sample
import math
from math import pi

from random import randint, random, uniform

import time, sys

from math import radians

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


def draw_random_sample():
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # TODO
    return



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

        # initialize our map
        self.map = OccupancyGrid()
        # initialize a likelihood field of the map
        self.likelihood_field = LikelihoodField()

        self.z_hit = .65
        self.z_rand = .35

        # the number of particles used in the particle filter
        self.num_particles = 7500

        # Keep track of our total normalized weights for error checking during resample
        self.weight_sum = 0

        # Set a resolution for how many measurements we want to take per particle
        # 90 measurements per particle means include every 4th degree
        self.measurement_resolution = 90  # measurements per particle
        if 360 % self.measurement_resolution != 0:
            print("Bad measurement resolution chosen: ", self.measurement_resolution)
            sys.exit()
        self.measurement_directions = [
            direction * (360 // self.measurement_resolution) for direction in range(self.measurement_resolution)
        ]
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


        # intialize the particle cloud
        print("Initializing cloud...")
        self.initialize_particle_cloud()

        self.initialized = True
        print("Initialized.")

    def get_map(self, data):
        self.map = data

    def initialize_particle_cloud(self):
        # Ensure that self.map has been properly initialized before initializing particle cloud
        while self.map.info.width == 0:
            time.sleep(0.5)
        # Pulls a MapMetaData object from the map: http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
        map_info = self.map.info
        # The data of our map specifying occupancy probabilities
        map_data = self.map.data
        # A float describing m / cell fo the map
        map_resolution = map_info.resolution
        print("Map res: ", map_resolution)
        # How many cells the map is across
        map_width = map_info.width
        # How many cells the map is up and down
        map_height = map_info.height
        # The pose of the map's origin; we'll use this to calculate a positional offsets for our random particles
        map_origin = map_info.origin

        # A list of integers between 0 and map_width to generate random x coords for all our particles
        w_rands = np.random.randint(map_width, size=self.num_particles)
        # A list of integers between 0 and map_height to generate random y coords for all our particles
        h_rands = np.random.randint(map_height, size=self.num_particles)
        # a list of random scalars to generate random yaws for our particles
        yaw_rands = np.random.rand(self.num_particles)

        # For every particle we want
        # a random x_cell position
        # a random y_cell position
        # and a random yaw scalar
        for w_r, h_r, yaw_r in zip(w_rands, h_rands, yaw_rands):

            # Draw a random x,y position using our height and width
            pose_x = w_r * map_resolution + map_origin.position.x
            pose_y = h_r * map_resolution + map_origin.position.y

            # Draw a random yaw from 0 to 2pi
            pose_yaw = yaw_r * 2 * pi
            # Calculate the quaternion
            pose_quaternion = quaternion_from_euler(0, 0, pose_yaw)

            # Initialize a new pose to hold our data
            pose = Pose()

            # Populate that pose with our new random position
            pose.position.x = pose_x
            pose.position.y = pose_y
            pose.position.z = 0
            # and orientation
            pose.orientation.x = pose_quaternion[0]
            pose.orientation.y = pose_quaternion[1]
            pose.orientation.z = pose_quaternion[2]
            pose.orientation.w = pose_quaternion[3]

            # Assign that position a probability weight based on whether the robot could feasibly be there
            # For each cell, a value of 1 is good, while 0 or -1 indicates a wall or empty space
            weight = 0.0
            if map_data[w_r + h_r * map_width] >= 0:
                weight = 1.0

            # add this to our cloud
            self.particle_cloud.append(Particle(pose, weight))

        self.normalize_particles()
        self.publish_particle_cloud()

    def normalize_particles(self):
        # Extract the un-normalized weights
        weights = np.array([p.w for p in self.particle_cloud])
        all_equal = True
        for w in weights:
            if w != weights[0]:
                all_equal = False
        if all_equal:
            weights = np.array([1.0] * self.num_particles)
        normalized_weights = weights / weights.sum()
        self.weight_sum = normalized_weights.sum()
        # Update every particle using the new normalized weight
        for p, nw in zip(self.particle_cloud, normalized_weights):
            p.w = nw

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
        # Resample the particle cloud with replacement based on their normalized weights.
        particle_probabilities = [p.w for p in self.particle_cloud]
        try:
            # Resample the particles based on their weights
            pc = np.random.choice(
                self.particle_cloud,
                size=self.num_particles,
                p=particle_probabilities
            )

            self.particle_cloud = pc

            print("Resampled set size: ", len(set(self.particle_cloud)))
        # On some sort of exception, log and error.
        except ValueError as e:
            print(e)
            print("Normalizer: ", self.weight_sum)
            sys.exit()

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

        if self.particle_cloud is not None:
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

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose

    def update_estimated_robot_pose(self):
        # Initialize a new estimated pose
        estimate = Pose()
        # Calculate an unweighted average of all particles
        # First, set all values to 0
        estimate.position.x = 0
        estimate.position.y = 0
        estimate.position.z = 0
        estimate.orientation.x = 0
        estimate.orientation.y = 0
        estimate.orientation.z = 0
        estimate.orientation.w = 0
        
        # Iterate through all particles to sum their positions and orientations
        for p in self.particle_cloud:
            estimate.position.x += p.pose.position.x
            estimate.position.y += p.pose.position.y
            estimate.orientation.x += p.pose.orientation.x
            estimate.orientation.y += p.pose.orientation.y
            estimate.orientation.z += p.pose.orientation.z
            estimate.orientation.w += p.pose.orientation.w

        # Caluclate the average of each value by dividing by the total number
        # of particles
        estimate.position.x /= self.num_particles
        estimate.position.y /= self.num_particles
        estimate.orientation.x /= self.num_particles
        estimate.orientation.y /= self.num_particles
        estimate.orientation.z /= self.num_particles
        estimate.orientation.w /= self.num_particles

        # Set the estimated pose to our new estimate
        self.robot_estimate = estimate

    def update_particle_weights_with_measurement_model(self, data):
        # count = 0
        print("Updating particles by measurement")
        pc = self.particle_cloud
        self.particle_cloud = []
        for p in pc:
            theta = get_yaw_from_pose(p.pose)
            # print("Updating particle: pos: <", p.pose.position.x, p.pose.position.y, theta, "> | weight: ", p.w)
            q = 1
            usable_dist_info = False
            # Iterate through each direction
            for a in self.measurement_directions:
                z_k = data.ranges[a]
                # print("Looking in direction: ", a, "| range: ", z_k, "| q: ", q)
                # Account for both simulated and live runs
                if np.isfinite(z_k) or z_k == 0:
                    # Calculate X and Y given z^k_t
                    x_z = p.pose.position.x + z_k * np.cos(theta + np.radians(a)) # * self.map.info.resolution
                    y_z = p.pose.position.y + z_k * np.sin(theta + np.radians(a)) # * self.map.info.resolution
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_z, y_z)
                    # if this is a position on the map
                    # print("Likelihood pos: pos: <", x_z, y_z, "> | dist: ", dist)
                    if not math.isnan(dist):
                        usable_dist_info = True
                        prob = compute_prob_zero_centered_gaussian(dist, 0.5)
                        q *= self.z_hit * prob + self.z_rand / data.range_max
            if usable_dist_info:
                # print("New weight: ", q)
                p.w = q
            self.particle_cloud.append(Particle(p.pose, p.w))

    # Determine the appropriate translation for each particle based on the robots reported odometry update
    def model_odometry_translation(self):
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # Using the model Sarah referenced in the Slack Channel
        # Online source here: https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf
        d_rot1 = math.atan2(curr_y - old_y, curr_x - old_x) - old_yaw
        d_trans = math.sqrt(pow(old_x - curr_x, 2) + pow(old_y - curr_y, 2))
        d_rot2 = curr_yaw - old_yaw - d_rot1

        return d_rot1, d_trans, d_rot2

    def update_particles_with_motion_model(self):
        lost_particles = False
        d_rot1, d_trans, d_rot2 = self.model_odometry_translation()

        # rot1_rands = trans_rands = rot2_rands = [0] * self.num_particles
        pc = self.particle_cloud
        self.particle_cloud = []
        # for p, r1_r, t_r, r2_r in zip(pc, rot1_rands, trans_rands, rot2_rands):
        for p in pc:
            d_hat_rot1 =  d_rot1
            d_hat_trans = d_trans
            d_hat_rot2 = d_rot2

            pose_yaw = get_yaw_from_pose(p.pose)

            # Update p's x, y, and yaw coords in accordance with our reported translation, while including uniform noise
            pose_x = p.pose.position.x + (d_hat_trans * math.cos(pose_yaw + d_hat_rot1) + uniform(-.1, .1)) * self.map.info.resolution
            pose_y = p.pose.position.y + (d_hat_trans * math.sin(pose_yaw + d_hat_rot1) + uniform(-.1, .1)) * self.map.info.resolution

            pose_yaw = pose_yaw + d_hat_rot1 + d_hat_rot2 + uniform(-.1,.1)

            # If any particle moves off the edge of the map, lower its weight so it gets sampled out
            if not self.likelihood_field.get_closest_obstacle_distance(pose_x, pose_y):
                lost_particles = True
                p.w = p.w / 1000

            # Calculate the quaternion for our new pose
            pose_quaternion = quaternion_from_euler(0, 0, pose_yaw)

            # Populate that pose with our new random position
            p.pose.position.x = pose_x
            p.pose.position.y = pose_y
            p.pose.position.z = 0

            # and orientation
            p.pose.orientation.x = pose_quaternion[0]
            p.pose.orientation.y = pose_quaternion[1]
            p.pose.orientation.z = pose_quaternion[2]
            p.pose.orientation.w = pose_quaternion[3]

            self.particle_cloud.append(Particle(p.pose, p.w))
        if lost_particles:
            print("[ERROR] lost at least one particle off the side of the map")


if __name__=="__main__":

    pf = ParticleFilter()

    rospy.spin()









