#!/usr/bin/env python

import rosbag
import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
import glob
import os
from tf.transformations import euler_from_quaternion
from scipy.interpolate import interp1d

if (len(sys.argv) > 1):
    filename = "../bag/" + sys.argv[1] + ".bag"
else:
    list_of_files = glob.glob('../bag/*')
    latest_file = max(list_of_files, key=os.path.getctime)
    filename = latest_file


def get_rpy(msg):

    roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                              msg.pose.pose.orientation.y,
                                              msg.pose.pose.orientation.z,
                                              msg.pose.pose.orientation.w])
    return roll, pitch, yaw


class topic_data:

    def __init__(self, topic_name, filename):
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
        self.get_data_from_bag(filename)

    def get_data_from_bag(self, filename):
        with rosbag.Bag(filename) as bag:
            for topic, msg, t in bag.read_messages(topics=[self.topic_name]):
                self.data["time"].append(
                    msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
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

    def calculate_velocities(self):
        self.data["v_x"] = []
        self.data["v_y"] = []
        self.data["v_z"] = []
        self.data["omega_x"] = []
        self.data["omega_y"] = []
        self.data["omega_z"] = []

        self.data["v_x"].append(0)
        self.data["v_y"].append(0)
        self.data["v_z"].append(0)
        self.data["omega_x"].append(0)
        self.data["omega_y"].append(0)
        self.data["omega_z"].append(0)

        for i in range(len(self.data["time"])-1):
            self.data["v_x"].append((self.data["r_x"][i+1] - self.data["r_x"]
                                     [i]) / (self.data["time"][i+1] - self.data["time"][i]))
            self.data["v_y"].append((self.data["r_y"][i+1] - self.data["r_y"]
                                     [i]) / (self.data["time"][i+1] - self.data["time"][i]))
            self.data["v_z"].append((self.data["r_z"][i+1] - self.data["r_z"]
                                     [i]) / (self.data["time"][i+1] - self.data["time"][i]))

            self.data["omega_x"].append((self.data["roll"][i+1] - self.data["roll"][i]) / (
                self.data["time"][i+1] - self.data["time"][i]))
            self.data["omega_y"].append((self.data["pitch"][i+1] - self.data["pitch"][i]) / (
                self.data["time"][i+1] - self.data["time"][i]))
            self.data["omega_z"].append(
                (self.data["yaw"][i+1] - self.data["yaw"][i]) / (self.data["time"][i+1] - self.data["time"][i]))


truth = topic_data('/tag_box_pose_ground_truth', filename)
detection = topic_data('/detected_object_state', filename)
predicted = topic_data('/predicted_object_state', filename)
ekf = topic_data('/updated_object_state', filename)

detection.calculate_velocities()


def compare_plot(attr, include_detected):
    plt.plot(truth.data['time'], truth.data[attr],  'r.-', label='truth')
    if include_detected:
        plt.plot(detection.data['time'],
                 detection.data[attr], 'g.-', label='detected')
    plt.plot(predicted.data['time'],
             predicted.data[attr], 'y.-', label='predicted')
    plt.plot(ekf.data['time'], ekf.data[attr], 'b.-', label='ekf')
    plt.title(attr)
    plt.legend()
    plt.xlim(2, 3.2)
    plt.show()
    # plt.savefig(attr, dpi=400)
    plt.cla()


def generate_plot(ax, attr, include_detected):
    ax.plot(truth.data['time'], truth.data[attr],  'r.-', label='truth')
    if include_detected:
        ax.plot(detection.data['time'], detection.data[attr], 'g.-', label='detected')
    ax.plot(predicted.data['time'], predicted.data[attr], 'y.-', label='predicted')
    ax.plot(ekf.data['time'], ekf.data[attr], 'b.-', label='ekf')
    ax.set_title(attr)
    ax.legend()
    ax.set_xlim([2.4, 3.1])
    # plt.savefig(attr, dpi=400)


def error_plot(ax, attr):
    f = interp1d(truth.data['time'], truth.data[attr])
    err_det = detection.data[attr] - f(detection.data['time'])
    err_ekf = ekf.data[attr] - f(ekf.data['time'])

    ax.plot(detection.data['time'], err_det, 'g.-', label='detected')
    ax.plot(ekf.data['time'], err_ekf, 'b.-', label='ekf')
    
    plt.title(attr)
    plt.legend()
    plt.xlim(2, 3.0)

if __name__ == "__main__":

    fig, axs = plt.subplots(4, 3)
    error_plot(axs[0][0], 'r_x')
    error_plot(axs[0][1], 'r_y')
    error_plot(axs[0][2], 'r_z')
    error_plot(axs[1][0], 'roll')
    error_plot(axs[1][1], 'pitch')
    error_plot(axs[1][2], 'yaw')
    error_plot(axs[2][0], 'v_x')
    error_plot(axs[2][1], 'v_y')
    error_plot(axs[2][2], 'v_z')
    error_plot(axs[3][0], 'omega_x')
    error_plot(axs[3][1], 'omega_y')
    error_plot(axs[3][2], 'omega_z')
    plt.show()


    # fig, axs = plt.subplots(4, 3)
    # generate_plot(axs[0][0], 'r_x', True)
    # generate_plot(axs[0][1], 'r_y', True)
    # generate_plot(axs[0][2], 'r_z', True)
    # generate_plot(axs[1][0], 'roll', True)
    # generate_plot(axs[1][1], 'pitch', True)
    # generate_plot(axs[1][2], 'yaw', True)
    # generate_plot(axs[2][0], 'v_x', True)
    # generate_plot(axs[2][1], 'v_y', True)
    # generate_plot(axs[2][2], 'v_z', True)
    # generate_plot(axs[3][0], 'omega_x', True)
    # generate_plot(axs[3][1], 'omega_y', True)
    # generate_plot(axs[3][2], 'omega_z', True)
    # plt.show()


# compare_plot('r_x', True)
# compare_plot('r_y', True)
# compare_plot('r_z', True)
# compare_plot('q_x', True)
# compare_plot('q_y', True)
# compare_plot('q_z', True)
# compare_plot('q_w', True)
# compare_plot('roll', True)
# compare_plot('pitch', True)
# compare_plot('yaw', True)
# compare_plot('v_x', False)
# compare_plot('v_y', False)
# compare_plot('v_z', False)
# compare_plot('omega_x', False)
# compare_plot('omega_y', False)
# compare_plot('omega_z', False)

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

# with open('analysis.csv', 'wb') as f:
#     writer = csv.writer(f, delimiter=',', quoting=csv.QUOTE_ALL)
#     for row in rows:
#         writer.writerow(row)
