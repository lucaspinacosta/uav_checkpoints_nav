#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
from datetime import datetime
from pyproj import Transformer
import os
import rospkg


class GrabGoals:
    def __init__(self):
        rospy.init_node('grab_goals_node', anonymous=True, disable_signals=True)

        # Load params from the parameter server
        self.gps_topic = rospy.get_param('~GPS_TOPIC', 'gps/filtered')
        self.frame_id = rospy.get_param('~FRAME_ID', 'map')

        self.output_file = os.path.join(rospkg.RosPack().get_path('uav_checkpoints_nav'), rospy.get_param('~PATH_FILE_OUTPUT', 'data/zigzag_path.json'))

        rospy.loginfo(f"[grab_goals] Subscribing to GPS topic: {self.gps_topic}")
        rospy.loginfo(f"[grab_goals] Output file will be: {self.output_file}")

        self.latest_navsatfix: NavSatFix = None
        self.subscriber = rospy.Subscriber(self.gps_topic, NavSatFix, self.callback)

        # Transformer for lat/lon to UTM (replace 32633 with the UTM zone you're in)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32633", always_xy=True)

        self._json_2_save = {
            'header': {
                'frame_id': self.frame_id,
                'stamp': "",
            },
            'poses': [],
            'start_index': 0,
        }

    def callback(self, data: NavSatFix):
        self.latest_navsatfix = data

    def convert_navsat_to_pose(self, fix: NavSatFix) -> Pose:
        x, y = self.transformer.transform(fix.longitude, fix.latitude)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = fix.altitude
        return pose

    def pose_to_dict(self, pose: Pose):
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            }
        }

    def save_json_to_file(self):
        if not os.path.exists(self.output_file):
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, 'w') as f:
            json.dump(self._json_2_save, f, indent=4)
        rospy.loginfo(f"[grab_goals] Saved {len(self._json_2_save['poses'])} poses to {self.output_file}")

    def run(self):
        rate = rospy.Rate(1)
        print("Listening to GPS fix. Press 'Y' to save current fix, 'exit' to finish.")

        while not rospy.is_shutdown():
            _input1 = input("Save current GNSS as UTM pose? [Y/N]: ").strip().lower()
            if _input1 == 'y':
                if self.latest_navsatfix:
                    pose = self.convert_navsat_to_pose(self.latest_navsatfix)
                    pose_dict = self.pose_to_dict(pose)
                    self._json_2_save['poses'].append(pose_dict)
                    self._json_2_save['start_index'] += 1
                    print(f"Saved pose #{self._json_2_save['start_index']}: {pose_dict}")
                else:
                    print("⚠️ No GNSS fix received yet.")
            elif _input1 == 'n':
                print("Skipping this point.")
            else:
                print("Invalid input.")

            _input2 = input("Press Enter to continue or type 'exit' to finish and save: ").strip().lower()
            if _input2 == 'exit':
                if self._json_2_save['poses']:
                    self._json_2_save['start_index'] = 0
                    self.save_json_to_file()
                print("Exiting.")
                break

            rate.sleep()


if __name__ == '__main__':
    try:
        GrabGoals().run()
    except rospy.ROSInterruptException:
        pass
