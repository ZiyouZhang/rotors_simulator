#!/usr/bin/env python

import rosbag
import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
from tf.transformations import euler_from_quaternion

filename = "/home/ziyou/Desktop/bag/" + sys.argv[1] + ".bag"

def get_rpy(msg):

    roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                              msg.pose.pose.orientation.y,
                                              msg.pose.pose.orientation.z,
                                              msg.pose.pose.orientation.w])
    return roll, pitch, yaw

class topic_data:

    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.data = {
            "time": [],
            "r_x": [],
            "r_y": [],
            "r_z": [],
            "q_x": [],
            "q_y": [],
            "q_z": [],
            "q_w": [],
            "roll": [],
            "pitch": [],
            "yaw": [],
            "v_x": [],
            "v_y": [],
            "v_z": [],
            "omega_x": [],
            "omega_y": [],
            "omega_z": []
        }

    def get_data_from_bag(self, filename):
        with rosbag.Bag(filename) as bag:
            for topic, msg, t in bag.read_messages(topics=[self.topic_name]):
                self.data["time"].append(t.secs + t.nsecs/1e9)
                self.data["r_x"].append(msg.pose.pose.position.x)
                self.data["r_y"].append(msg.pose.pose.position.y)
                self.data["r_z"].append(msg.pose.pose.position.z)
                self.data["q_x"].append(msg.pose.pose.orientation.x)
                self.data["q_y"].append(msg.pose.pose.orientation.y)
                self.data["q_z"].append(msg.pose.pose.orientation.z)
                self.data["q_w"].append(msg.pose.pose.orientation.w)
                self.data["roll"].append(get_rpy(msg)[0])
                self.data["pitch"].append(get_rpy(msg)[1])
                self.data["yaw"].append(get_rpy(msg)[2])
                self.data["v_x"].append(msg.twist.twist.linear.x)
                self.data["v_y"].append(msg.twist.twist.linear.y)
                self.data["v_z"].append(msg.twist.twist.linear.z)
                self.data["omega_x"].append(msg.twist.twist.angular.x)
                self.data["omega_y"].append(msg.twist.twist.angular.y)
                self.data["omega_z"].append(msg.twist.twist.angular.z)


truth = topic_data('/tag_box_pose_ground_truth')
detection = topic_data('/detected_object_state')
predicted = topic_data('/predicted_object_state')
ekf = topic_data('/updated_object_state')

truth.get_data_from_bag(filename)
detection.get_data_from_bag(filename)
predicted.get_data_from_bag(filename)
ekf.get_data_from_bag(filename)

# rows = [truth.time,
#         truth.r_z,
#         truth.roll,
#         truth.v_z,
#         truth.omega_x,
#         predicted.time,
#         predicted.r_z,
#         predicted.roll,
#         predicted.v_z,
#         predicted.omega_x,
#         detection.time,
#         detection.r_z,
#         detection.roll,
#         detection.v_z,
#         detection.omega_x,
#         ekf.time,
#         ekf.r_z,
#         ekf.roll,
#         ekf.v_z,
#         ekf.omega_x]
rows = []

with open('analysis.csv', 'wb') as f:
    writer = csv.writer(f, delimiter=',', quoting=csv.QUOTE_ALL)
    for row in rows:
        writer.writerow(row)

def compare_plot_pose(attr):
    plt.plot(truth.data['time'], truth.data[attr],  'r.-', label='truth')
    plt.plot(detection.data['time'],
             detection.data[attr], 'g.-', label='detected')
    plt.plot(predicted.data['time'],
             predicted.data[attr], 'y.-', label='predicted')
    plt.plot(ekf.data['time'], ekf.data[attr], 'b.-', label='ekf')
    plt.title(attr)
    plt.legend()
    plt.xlim(2.5, 3.2)
    # plt.show()
    plt.savefig(attr, dpi=300)
    plt.cla()

def compare_plot_twist(attr):
    plt.plot(truth.data['time'], truth.data[attr],  'r.-', label='truth')
    plt.plot(predicted.data['time'],
             predicted.data[attr], 'y.-', label='predicted')
    plt.plot(ekf.data['time'], ekf.data[attr], 'b.-', label='ekf')
    plt.title(attr)
    plt.legend()
    plt.xlim(2.5, 3.2)
    # plt.show()
    plt.savefig(attr, dpi=400)
    plt.cla()


compare_plot_pose('r_x')
compare_plot_pose('r_y')
compare_plot_pose('r_z')
compare_plot_pose('q_x')
compare_plot_pose('q_y')
compare_plot_pose('q_z')
compare_plot_pose('q_w')
compare_plot_pose('roll')
compare_plot_pose('pitch')
compare_plot_pose('yaw')
compare_plot_twist('v_x')
compare_plot_twist('v_y')
compare_plot_twist('v_z')
compare_plot_twist('omega_x')
compare_plot_twist('omega_y')
compare_plot_twist('omega_z')
