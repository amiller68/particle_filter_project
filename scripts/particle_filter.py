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

from random import randint, random



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


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

        # inialize our map
        self.map = OccupancyGrid()
        self.likelihood_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        self.weight_sum = 0

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
        # Pulls a MapMetaData object from the map: http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
        map_info = self.map.info
        # The data of our map specifying occupancy probabilities
        map_data = self.map.data
        # A float describing m / cell fo the map
        map_resolution = map_info.resolution
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
        normalized_weights = weights / weights.sum()
        self.weight_sum = normalized_weights.sum()
        # Update every particle using the new normalized weight
        for p, nw in zip(self.particle_cloud,normalized_weights):
            p.w = nw

    def publish_particle_cloud(self):
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        # particle_cloud_pose_array.poses

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
            self.particle_cloud = np.random.choice(
                self.particle_cloud,
                size = self.num_particles,
                p=particle_probabilities
            )
        # On some sort of exception, log and error.
        except ValueError as e:
            print(e)
            print("Normalizer: ", self.weight_sum)
            sys.exit(1)

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
        # Pull the top 5% of guesses of particle by weight
        best_guesses = sorted(self.particle_cloud, key=lambda p: p.w, reverse=True)[:self.num_particles // 20]

        # Initialize a new estimated pose
        estimate = Pose()
        # For now, just create an unweighted average of these particles
        for p in best_guesses:
            # Add to the pose's position
            estimate.position.x = estimate.position.x + p.pose.position.x / len(best_guesses)
            estimate.position.y = estimate.position.y + p.pose.position.y / len(best_guesses)
            estimate.position.z = 0

            # and orientation
            estimate.orientation.x = estimate.orientation.x + p.pose.orientation.x / len(best_guesses)
            estimate.orientation.y = estimate.orientation.y + p.pose.orientation.y / len(best_guesses)
            estimate.orientation.z = estimate.orientation.z + p.pose.orientation.z / len(best_guesses)
            estimate.orientation.w = estimate.orientation.w + p.pose.orientation.w / len(best_guesses)
        # Set the estimated pose to our new estimate
        self.robot_estimate = estimate

    def update_particle_weights_with_measurement_model(self, data):
        # For every particle
        # BELOW IS CODE I USED FOR CLASS ASSIGNEMENT. I THINK IT MOSTLY WORKS

        # for p in self.particle_cloud:
        #     x = p.pose.position.x
        #     y = p.pose.position.y
        #     theta = (euler_from_quaternion(
        #         [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])[2])
        #     print("Updating particle at [", x, y, theta, "]")
        #     q = 1
        #     info = False
        #     for cd, cr in zip(cardinal_directions_idxs, cardinal_radians):
        #         z_k = scan.ranges[cd]
        #         print("Looking in direction: ", cd, "| range: ", z_k)
        #         if not math.isinf(z_k):
        #             print("hmm")
        #             info = True
        #             # Calculate X and Y given z^k_t
        #             x_z = x + 0 * np.cos(theta) - 0 * np.sin(theta) + z_k * np.cos(theta + cr)
        #             y_z = y + 0 * np.cos(theta) - 0 * np.sin(theta) + z_k * np.sin(theta + cr)
        #             dist = self.likelihood_field.get_closest_obstacle_distance(x_z, y_z)
        #             prob = compute_prob_zero_centered_gaussian(dist, 0.1)
        #             print("Scan Value: ", z_k, " | Projected Scan location: ", x_z, y_z, " | Dist: ", dist, "| prob: ",
        #                   prob)
        #             q = q * prob
        #         else:
        #             print("Too far to tell")
        #     if info:
        #         p.w = q
        #     else:
        #         print("no information to update q!")
        pass
        # TODO

    def update_particles_with_motion_model(self):
        # Basically:
        # For each particle
        #  - Calculate a new position using a movement command drawn from a gaussian distribution based
        #    on the robots reported movement.
        #  - If that pos is outside the map, give that particle a low weight programmaticaly. We can check this with the likelihood thing
        
        pass
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO

if __name__=="__main__":

    pf = ParticleFilter()

    rospy.spin()









