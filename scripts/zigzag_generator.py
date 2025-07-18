#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import numpy as np
from datetime import datetime
from pyproj import Transformer
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
import os
import rospkg


class ZigzagCollector:
    def __init__(self):
        rospy.init_node('grab_and_zigzag_node', anonymous=True)

        # Parameters
        self.gps_topic = rospy.get_param('~GPS_TOPIC', 'gps/filtered')
        self.frame_id = rospy.get_param('~FRAME_ID', 'map')
        self.output_file = os.path.join(rospkg.RosPack().get_path('uav_checkpoints_nav'), rospy.get_param('~PATH_FILE_OUTPUT', 'data/zigzag_path.json'))
        self.spacing_m = rospy.get_param('~SPACING', 2.0)

        rospy.loginfo(f"[zigzag_collector] GPS topic: {self.gps_topic}")
        rospy.loginfo(f"[zigzag_collector] Output file: {self.output_file}")
        rospy.loginfo(f"[zigzag_collector] Spacing: {self.spacing_m}")

        # GPS Subscriber
        self.latest_fix = None
        self.sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)

        # Lat/lon to UTM transformer (modify EPSG if needed)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32633", always_xy=True)

        self.utm_corners = []  # Collected UTM corner points
        self.zigzag_path = []

    def gps_callback(self, msg: NavSatFix):
        self.latest_fix = msg

    def navsat_to_utm_pose(self, fix: NavSatFix) -> Pose:
        pose = Pose()
        x, y = self.transformer.transform(fix.longitude, fix.latitude)
        pose.position.x = x
        pose.position.y = y
        pose.position.z = fix.altitude
        return pose

    def utm_to_latlon_dict(self, x, y, z=0.0):
        lat, lon = self.transformer.transform(x, y, direction='INVERSE')
        return {
            "position": {
                "x": lat,
                "y": lon,
                "z": z
            },
            "orientation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "w": 1
            }
        }

    def collect_corners(self):
        rate = rospy.Rate(1)
        print("Collecting GPS poses for zigzag corners...")
        while not rospy.is_shutdown() and len(self.utm_corners) < 4:
            cmd = input(f"Press 'Y' to save current GNSS as corner #{len(self.utm_corners)+1}: ").strip().lower()
            if cmd == 'y':
                if self.latest_fix:
                    pose = self.navsat_to_utm_pose(self.latest_fix)
                    self.utm_corners.append([pose.position.x, pose.position.y])
                    print(f"Saved corner {len(self.utm_corners)}: x={pose.position.x}, y={pose.position.y}")
                else:
                    print("⚠️ No GPS fix received yet.")
            elif cmd == 'exit':
                print("Exiting without generating path.")
                return False
            else:
                print("Invalid input.")
            rate.sleep()
        return True

    def generate_zigzag_path(self):
        if len(self.utm_corners) != 4:
            raise ValueError("Need exactly 4 corners to generate zigzag path.")

        p1, p2, p3, p4 = self.utm_corners
        total_width = np.linalg.norm(np.array(p2) - np.array(p1))
        num_passes = int(total_width // self.spacing_m)

        rospy.loginfo(f"[zigzag_collector] Generating {num_passes+1} zigzag passes.")

        for i in range(num_passes + 1):
            t = i / num_passes
            left_pt = (1 - t) * np.array(p1) + t * np.array(p2)
            right_pt = (1 - t) * np.array(p4) + t * np.array(p3)

            if i % 2 == 0:
                self.zigzag_path.append(left_pt)
                self.zigzag_path.append(right_pt)
            else:
                self.zigzag_path.append(right_pt)
                self.zigzag_path.append(left_pt)

    def save_path_as_json(self):
        poses_out = [self.to_pose_dict(x, y) for (x, y) in self.zigzag_path]
        output_data = {
            "header": {
                "frame_id": self.frame_id,
                "stamp": str(datetime.now())
            },
            "poses": poses_out,
            "start_index": 0  # unified structure
        }

        with open(self.output_file, 'w') as f:
            json.dump(output_data, f, indent=4)
        rospy.loginfo(f"[zigzag] Saved {len(poses_out)} zigzag points to {self.output_file}")

    def run(self):
        if self.collect_corners():
            self.generate_zigzag_path()
            self.save_path_as_json()


if __name__ == '__main__':
    try:
        ZigzagCollector().run()
    except rospy.ROSInterruptException:
        pass
