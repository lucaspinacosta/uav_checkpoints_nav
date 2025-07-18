#!/usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from datetime import datetime
import os
import rospkg


class PosePublisherStep:
    def __init__(self):
        rospy.init_node('pose_publisher_step_node', anonymous=True)

        # Load params
        self.pose_file = os.path.join(rospkg.RosPack().get_path('uav_checkpoints_nav'), rospy.get_param('~PATH_FILE_OUTPUT', 'data/zigzag_path.json'))
        self.topic_name = rospy.get_param('~PUBLISHED_TOPIC', 'next_pose')

        # Load file
        with open(self.pose_file, 'r') as f:
            self.data = json.load(f)

        self.poses = self.data['poses']
        self.start_index = self.data.get('start_index', 0)
        self.frame_id = self.data.get('header', {}).get('frame_id', 'map')

        self.publisher = rospy.Publisher(self.topic_name, PoseStamped, queue_size=10)

        rospy.loginfo(f"[pose_publisher] Loaded {len(self.poses)} poses from {self.pose_file}")
        rospy.loginfo(f"[pose_publisher] Starting at index: {self.start_index}")

    def pose_from_dict(self, pose_dict) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id

        pose_msg.pose.position.x = pose_dict['position']['x']
        pose_msg.pose.position.y = pose_dict['position']['y']
        pose_msg.pose.position.z = pose_dict['position']['z']

        pose_msg.pose.orientation.x = pose_dict['orientation']['x']
        pose_msg.pose.orientation.y = pose_dict['orientation']['y']
        pose_msg.pose.orientation.z = pose_dict['orientation']['z']
        pose_msg.pose.orientation.w = pose_dict['orientation']['w']

        return pose_msg

    def run(self):
        index = self.start_index
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and index < len(self.poses):
            _input = input(f"Press Enter to publish pose #{index} (or type 'exit'): ").strip().lower()
            if _input == 'exit':
                rospy.loginfo("[pose_publisher] Exiting...")
                break

            pose_msg = self.pose_from_dict(self.poses[index])
            self.publisher.publish(pose_msg)
            rospy.loginfo(f"[pose_publisher] Published pose #{index}")
            index += 1
            rate.sleep()

        rospy.loginfo("[pose_publisher] All poses published or user exited.")


if __name__ == '__main__':
    try:
        PosePublisherStep().run()
    except rospy.ROSInterruptException:
        pass
