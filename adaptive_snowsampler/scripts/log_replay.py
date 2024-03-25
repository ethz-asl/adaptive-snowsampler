#! /usr/bin/env python

"""
Convert a ULog file into rosbag file(s)
"""

from collections import defaultdict
import argparse
import re
import rospy # pylint: disable=import-error
import rosbag # pylint: disable=import-error
# from px4_msgs import msg as px4_msgs # pylint: disable=import-error
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
import os
import numpy as np
import math

from pyulog import core

#pylint: disable=too-many-locals, invalid-name

def quatMultiplication(q, p):
  quat = np.array([p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3], p[0] * q[1] + p[1] * q[0] - p[2] * q[3] + p[3] * q[2],
      p[0] * q[2] + p[1] * q[3] + p[2] * q[0] - p[3] * q[1], p[0] * q[3] - p[1] * q[2] + p[2] * q[1] + p[3] * q[0]])
  return quat

def main():
    """Command line interface"""

    parser = argparse.ArgumentParser(description='Convert ULog to rosbag')
    parser.add_argument('filename', metavar='file.ulg', help='ULog input file or directory')
    parser.add_argument('bag', metavar='file.bag', help='rosbag output file')

    parser.add_argument('-i', '--ignore', dest='ignore', action='store_true',
                        help='Ignore string parsing exceptions', default=False)

    args = parser.parse_args()

    convert_ulog2rosbag(args.filename, args.bag, args.ignore)

# https://stackoverflow.com/questions/19053707/converting-snake-case-to-lower-camel-case-lowercamelcase
def to_camel_case(snake_str):
    """ Convert snake case string to camel case """
    components = snake_str.split("_")
    return ''.join(x.title() for x in components)

def parseData(d, topic, idx):
    return d.data[topic][idx]


def appendBag(path, bag):
        msg_filter={'vehicle_global_position', 'vehicle_attitude', 'distance_sensor'}
        disable_str_exceptions=False
        ulog = core.ULog(path, msg_filter, disable_str_exceptions)
        data = ulog.data_list

        multiids = defaultdict(set)
        for d in data:
            multiids[d.name].add(d.multi_id)

        items = []
        for d in data:
            for i in range(len(d.data['timestamp'])):
                if d.name == "vehicle_global_position":
                    topic = "/mavros/global_position/global"

                    msg = NavSatFix()
                    msg.latitude = parseData(d, 'lat', i)
                    msg.longitude = parseData(d, 'lon', i)
                    msg.altitude = parseData(d, 'alt_ellipsoid', i)
                elif d.name == "vehicle_attitude":
                    topic = "mavros/local_position/pose"
                    msg = PoseStamped()
                    vehicle_attitude = np.array([parseData(d, 'q[0]', i), parseData(d, 'q[1]', i), -parseData(d, 'q[2]', i), -parseData(d, 'q[3]', i)])
                    attitude_offset = np.array([np.cos(0.5 * 0.5 * math.pi), 0.0, 0.0, np.sin(0.5 * 0.5 * math.pi)])
                    vehicle_aligned_attitude = quatMultiplication(attitude_offset, vehicle_attitude)
                    msg.pose.orientation.w = vehicle_aligned_attitude[0]
                    msg.pose.orientation.x = vehicle_aligned_attitude[1]
                    msg.pose.orientation.y = vehicle_aligned_attitude[2]
                    msg.pose.orientation.z = vehicle_aligned_attitude[3]
                                    
                elif d.name == "distance_sensor":
                    topic = "mavros/distance_sensor"
                    msg = Range()
                    msg.range = parseData(d, 'current_distance', i)

                # for f in d.field_data:
                #     result = array_pattern.match(f.field_name)
                #     value = d.data[f.field_name][i]
                #     print(f.field_name)
                #     print(value)
                    # if result:
                    #     field, array_index = result.groups()
                    #     array_index = int(array_index)
                    #     if isinstance(getattr(msg, field), bytes):
                    #         attr = bytearray(getattr(msg, field))
                    #         attr[array_index] = value
                    #         setattr(msg, field, bytes(attr))
                    #     else:
                    #         getattr(msg, field)[array_index] = value
                    # else:
                    #     setattr(msg, f.field_name, value)
                ts = rospy.Time(nsecs=d.data['timestamp'][i]*1000)
                items.append((topic, msg, ts))
        items.sort(key=lambda x: x[2])
        for topic, msg, ts in items:
            bag.write(topic, msg, ts)

def convert_ulog2rosbag(path, rosbag_file_name, messages, disable_str_exceptions=False):
    """
    Coverts and ULog file to a CSV file.

    :param ulog_file_name: The ULog filename to open and read
    :param rosbag_file_name: The rosbag filename to open and write
    :param messages: A list of message names

    :return: No
    """

    with rosbag.Bag(rosbag_file_name, 'w') as bag:
        if os.path.isdir(path):
            list = os.listdir(path)
            list.sort()

            for filename in list:
                print(filename)
                appendBag(os.path.join(path, filename), bag)
        else:
            appendBag(path, bag)

if __name__ == "__main__":
    main()
