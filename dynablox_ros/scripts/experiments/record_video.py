#!/usr/bin/env python3

import rospy
import rosbag
from visualization_msgs.msg import Marker, MarkerArray
from voxblox_msgs.msg import Mesh
from threading import Lock


class VideoRecorder(object):
    """ Record all visualization messages to a bag at normalized original play rate. """

    def __init__(self):
        """  Initialize ros node and read params """
        # params
        self.bag_file = rospy.get_param('~bag_file', '')
        self.global_frame_name = rospy.get_param('~global_frame_name', 'map')
        self.name_space = rospy.get_param(
            '~name_space', '/motion_detector/visualization')

        # Constants
        # All topics: ['lidar_points', 'detections/point/dynamic', 'detections/point/static', 'detections/cluster/dynamic', 'detections/cluster/static', 'detections/object/dynamic', 'detections/object/static', 'ground_truth/point', 'ground_truth/cluster', 'ground_truth/object', 'ever_free', 'never_free', 'mesh', 'slice/ever_free', 'slice/never_free', 'slice/points']
        # Special type topics: ['clusters', 'mesh', 'slice/points']

        # ROS
        self.sub1 = rospy.Subscriber(
            f"{self.name_space}/lidar_pose", Marker, self.cb_1, queue_size=100)
        self.sub2 = rospy.Subscriber(
            f"{self.name_space}/detections/object/dynamic", Marker, self.cb_2, queue_size=100)
        self.sub1 = rospy.Subscriber(
            f"{self.name_space}/detections/object/static", Marker, self.cb_3, queue_size=100)

        # setup
        self.mutex = Lock()
        self.current_time = None
        self.stamps = {}    # Lookups of past stamps.
        self.bag = rosbag.Bag(self.bag_file, 'w')
        print(f"Recording video to '{self.bag_file}'.")

    def cb_1(self, msg):
        self.callback(msg, f"{self.name_space}/lidar_pose")   

    def cb_2(self, msg):
        self.callback(msg, f"{self.name_space}/detections/object/dynamic")

    def cb_3(self, msg):
        self.callback(msg, f"{self.name_space}/detections/object/static")

    def callback(self, msg, topic):
        self.mutex.acquire()
        new_time = self.get_time(msg.header.stamp)
        msg.header.stamp = new_time
        self.bag.write(topic, msg, new_time)
        self.mutex.release()

    def get_time(self, stamp):
        if self.current_time is None:
            # First setup.
            self.current_time = stamp
            self.stamps[stamp.nsecs] = stamp
            return stamp
        if stamp.nsecs in self.stamps:
            # Known time.
            return self.stamps[stamp.nsecs]
        else:
            # New time.
            self.current_time.nsecs += 100000000
            if self.current_time.nsecs > 1000000000:
                self.current_time.nsecs -= 1000000000
                self.current_time.secs += 1
            self.stamps[stamp.nsecs] = self.current_time
            return self.current_time


if __name__ == '__main__':
    rospy.init_node('vis_player')
    recorder = VideoRecorder()
    rospy.spin()
    recorder.bag.close()
    print("Closed bag.")
