#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
import tf


class FlatDataPlayer(object):
    def __init__(self):
        self.pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=100)
        self.sub = rospy.Subscriber("/lidar_undistortion/pointcloud_corrected", PointCloud2, self.callback, queue_size=100)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.start_srv = rospy.Service('play', Empty, self.play)
        self.transforms =[]
        self.clouds= []
        self.timer = None
        self.i = 0

        # Params
        self.world_frame = 'map'

    def play(self, _):
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        return None

    def callback(self, msg):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(msg.header.frame_id,'map', msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Could not lookup tf!")
            return

        self.transforms.append([trans, rot])
        self.clouds.append(msg)
        print(f'Cached cloud {len(self.clouds)}.')

    def timer_callback(self, _):
        if self.i >= len(self.clouds):
            self.timer = None
            print(f"Finished playing {len(self.clouds)} messages!")
            return
        
        stamp = rospy.Time.now()
        transform = self.transforms[self.i]
        cloud = self.clouds[self.i]
        cloud.header.stamp = stamp
        self.tf_broadcaster.sendTransform(transform[0], transform[1], stamp, 'map', cloud.header.frame_id)
        self.pub.publish(cloud)
        self.i = self.i + 1
        print(f"Sent message {self.i}/{len(self.clouds)}.")

if __name__ == '__main__':
    rospy.init_node('cloud processor')
    flat_data_player = FlatDataPlayer()
    rospy.spin()
