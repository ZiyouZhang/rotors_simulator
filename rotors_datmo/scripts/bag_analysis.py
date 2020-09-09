import rosbag
import sys
import matplotlib.pyplot as plt
import numpy as np
import csv

filename = "/home/ziyou/Desktop/bag/" + sys.argv[1] + ".bag"

truth_time = []
truth_r_z = []
truth_v_z = []
truth_omega_x = []

predicted_time = []
predicted_r_z = []
predicted_v_z = []
predicted_omega_x = []

detection_time = []
detection_r_z = []
detection_v_z = []
detection_omega_x = []

ekf_time = []
ekf_r_z = []
ekf_v_z = []
ekf_omega_x = []

with rosbag.Bag(filename) as bag:
    for topic, msg, t in bag.read_messages(topics=['/tag_box_pose_ground_truth']):
        truth_time.append(t.secs + t.nsecs/1e9)
        truth_r_z.append(msg.pose.pose.position.z)
        truth_v_z.append(msg.twist.twist.linear.z)
        truth_omega_x.append(msg.twist.twist.angular.x)

    for topic, msg, t in bag.read_messages(topics=['/detected_object_state']):
        detection_time.append(t.secs + t.nsecs/1e9)
        detection_r_z.append(msg.pose.pose.position.z)
        detection_v_z.append(msg.twist.twist.linear.z)
        detection_omega_x.append(msg.twist.twist.angular.x)

    for topic, msg, t in bag.read_messages(topics=['/predicted_object_state']):
        predicted_time.append(t.secs + t.nsecs/1e9)
        predicted_r_z.append(msg.pose.pose.position.z)
        predicted_v_z.append(msg.twist.twist.linear.z)
        predicted_omega_x.append(msg.twist.twist.angular.x)

    for topic, msg, t in bag.read_messages(topics=['/updated_object_state']):
        ekf_time.append(t.secs + t.nsecs/1e9)
        ekf_r_z.append(msg.pose.pose.position.z)
        ekf_v_z.append(msg.twist.twist.linear.z)
        ekf_omega_x.append(msg.twist.twist.angular.x)

rows = [ekf_time,
        ekf_r_z,
        ekf_v_z,
        ekf_omega_x,
        truth_time,
        truth_r_z,
        truth_v_z,
        truth_omega_x,
        predicted_time,
        predicted_r_z,
        predicted_v_z,
        predicted_omega_x,
        detection_time,
        detection_r_z,
        detection_v_z,
        detection_omega_x,
        ekf_time,
        ekf_r_z,
        ekf_v_z,
        ekf_omega_x]

with open('analysis.csv', 'wb') as f:
    writer = csv.writer(f, delimiter=',', quoting=csv.QUOTE_ALL)
    for row in rows:
        writer.writerow(row)

# plt.plot(truth_time, truth_r_z, 'r.-')
# plt.plot(detection_time, detection_r_z, 'g.-')
# plt.plot(predicted_time, predicted_r_z, 'y.-')
# plt.plot(ekf_time, ekf_r_z, 'b.-')

# plt.title("r_z")
# plt.show()
# plt.cla()

# plt.plot(truth_time, truth_v_z, 'r.-')
# # plt.plot(detection_time, detection_v_z, 'g.-')
# plt.plot(predicted_time, predicted_v_z, 'y.-')
# plt.plot(ekf_time, ekf_v_z, 'b.-')
# plt.title("v_z")
# plt.show()
# plt.cla()

# plt.plot(truth_time, truth_omega_x, 'r.-')
# # plt.plot(detection_time, detection_omega_x, 'g.-')
# plt.plot(predicted_time, predicted_omega_x, 'y.-')
# plt.plot(ekf_time, ekf_omega_x, 'b.-')
# plt.title("omega_x")
# plt.show()
# plt.cla()
