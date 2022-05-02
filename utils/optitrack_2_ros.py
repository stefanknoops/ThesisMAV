import csv
from time import time
import numpy as np
import os
import pandas as pd
import matplotlib.pyplot as plt

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped


class Position:
    def __init__(self) -> None:
        self.x = np.array([])
        self.y = np.array([])
        self.z = np.array([])


class Quaternion:
    def __init__(self) -> None:
        self.x = np.array([])
        self.y = np.array([])
        self.z = np.array([])
        self.w = np.array([])


class OptitrackObject:
    def __init__(self, name):
        self.name = name
        self.rostopic_name = '/OptitrackPose_' + self.name
        self.t = pd.DataFrame()
        self.pos = Position()
        self.ori = Quaternion()

    def cropWithTime(self, t_crop):
        idxs = self.t > t_crop

        self.t = self.t[idxs]
        self.t -= t_crop

        self.pos.x = self.pos.x[idxs]
        self.pos.y = self.pos.y[idxs]
        self.pos.z = self.pos.z[idxs]

        self.ori.x = self.ori.x[idxs]
        self.ori.y = self.ori.y[idxs]
        self.ori.z = self.ori.z[idxs]
        self.ori.w = self.ori.w[idxs]

    def plotXProfile(self):
        plt.plot(self.t, self.pos.y, label=self.name + ' X profile')

    def plotYProfile(self):
        plt.plot(self.t, self.pos.y, label=self.name + ' Y profile')

    def plotZProfile(self):
        plt.plot(self.t, self.pos.z, label=self.name + ' Z profile')


# Discrepancies in seconds between rosbag start time and CSV start time.
# Obtained by looking at the take_off time of the drones from the camera
# recordings and the vertical position plots (plotZProfile)
t_crop = {
    0: 8.2,
    1: 8.5,
    2: 11,
    3: 7.3,
    4: 7.8,
    5: 6.8,
    6: 10.7,
    7: 7.9,
    8: 8.5,
    9: 7.7,
    10: 10.4,
    11: 10.3
}

# ID of the test to analize
test_id = 3

# Select test to edit
test_number = test_id
dvs_topic = '.'

# Get file and folder names
test_name = str(test_number)

csv_name = 'optitrack_' + test_name + '.csv'

root_dir = os.path.dirname(__file__)

csv_list = []

 # Read csv data
with open(csv_name) as csv_file:
      file = csv.reader(csv_file, delimiter=',')
      for row in file:
          # Append only first 100 elements, as they contain the useful information
          # needed.
          csv_list.append(row[:100])

  # Convert to df
csv_df = pd.DataFrame(csv_list[0:])

# Populate object structures with optitrack data
bebop = OptitrackObject('bebop')


bebop.t = csv_df.loc[1:, csv_df.iloc[0] == 'time [ns]'].to_numpy().astype(float)
bebop.pos.x = csv_df.loc[1:, csv_df.iloc[0] ==
                               'x [m]'].to_numpy().astype(float)
bebop.pos.y = csv_df.loc[1:, csv_df.iloc[0] ==
                               'y [m]'].to_numpy().astype(float)
bebop.pos.z = csv_df.loc[1:, csv_df.iloc[0] ==
                               'z [m]'].to_numpy().astype(float)
bebop.ori.x = csv_df.loc[1:, csv_df.iloc[0] ==
                               'a [rad]'].to_numpy().astype(float)
bebop.ori.y = csv_df.loc[1:, csv_df.iloc[0] ==
                               'b [rad]'].to_numpy().astype(float)
bebop.ori.z = csv_df.loc[1:, csv_df.iloc[0] ==
                               'c [rad]'].to_numpy().astype(float)
bebop.ori.w = csv_df.loc[1:, csv_df.iloc[0] ==
                               'd [rad]'].to_numpy().astype(float)

# Crop csv data to match start of bag
# bebop.cropWithTime(t_crop[test_number])

# Save data to rosbag
with rosbag.Bag('OPTITRACK_' + str(test_number) + '_DVS.bag',
                'w') as merged_bag:
    start_time = None

    # Add DVS data to bag
    with rosbag.Bag(str(test_number) + '.bag') as dvs_bag:
        for topic, msg, ts in dvs_bag:
            # Change topic name, from /data (DVS rosbag name) to /dvs/events
            # (OPTITRACK_DVS)
            merged_bag.write(dvs_topic, msg, ts)
            if start_time == None or start_time > ts:
                start_time = ts

    for idx, t in enumerate(bebop.t):
        timestamp = rospy.Duration.from_sec(t)/1000000000 + start_time

        bebop_pose_msg = PoseStamped()

        # Populate ros msgs
        bebop_pose_msg.header.stamp = timestamp
        bebop_pose_msg.pose.position.x = bebop.pos.x[idx]
        bebop_pose_msg.pose.position.y = bebop.pos.y[idx]
        bebop_pose_msg.pose.position.z = bebop.pos.z[idx]
        bebop_pose_msg.pose.orientation.x = bebop.ori.x[idx]
        bebop_pose_msg.pose.orientation.y = bebop.ori.y[idx]
        bebop_pose_msg.pose.orientation.z = bebop.ori.z[idx]
        bebop_pose_msg.pose.orientation.w = bebop.ori.w[idx]
        merged_bag.write(bebop.rostopic_name, bebop_pose_msg, timestamp)
