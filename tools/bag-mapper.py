#!/usr/bin/env python
import os
import rosbag
import numpy as np
import json
import pandas as pd
import argparse
from rosbag import Bag
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
import pymap3d

def get_args():
    parser = argparse.ArgumentParser(description="Outputs location messages to a CSV")
    parser.add_argument('--bags', nargs='+', help="The inputs bag files (in order)", required=True)
    parser.add_argument('--output', type=str, help="The output csv file")
    parser.add_argument('--topic', type=str, help="The topic to listen for localisation messages")
    parser.add_argument('--datum', type=str, help="The datum (comma separated)")
    parser.add_argument('--datum-topic', type=str, help="The datum topic")
    args = parser.parse_args()
    if args.datum and args.datum_topic:
        raise ValueError("Both datum and datum topic specified")
    return args

def get_datum(bag, datum_str=None, datum_topic=None):
    if datum_str is not None:
        datum_lat, datum_lon, datum_alt = [float(x) for x in datum_str.split(',')]
    else:
        for topic, msg, time in bag.read_messages(topics=[datum_topic]):
            datum_lat = msg.latitude
            datum_lon = msg.longitude
            datum_alt = msg.altitude
            break
    return datum_lat, datum_lon, datum_alt



def main(args):

    results = {'x': [], 'y': [], 'z': [], "lat": [], "lon": [], "alt": []}


    for bagfile in args.bags:
        with Bag(bagfile, 'r') as input_bag:
            if args.datum or args.datum_topic:
                datum_lat, datum_lon, datum_alt = get_datum(input_bag, args.datum, args.datum_topic)
            else:
                datum_lat, datum_lon, datum_alt = None, None, None

            for topic, msg, time in input_bag.read_messages(topics=[args.topic]):
                if msg._type == "sensor_msgs/NavSatFix":
                    lat = msg.latitude
                    lon = msg.longitude
                    alt = msg.altitude
                    if datum_lat is None:
                        datum_lat = lat
                        datum_lon = lon
                        datum_alt = alt
                    x,y,z, = pymap3d.geodetic2enu(lat,lon,alt, datum_lat, datum_lon, datum_alt)

                elif msg._type == "nav_msgs/Odometry":
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z
                    if not args.local:
                        lat, lon, alt = pymap3d.enu2geodetic(x,y,z,datum_lat, datum_lon, datum_alt)
                    else:
                        lat, lon, alt = None, None, None
                elif msg._type == "geometry_msgs/Pose":
                    x = msg.position.x
                    y = msg.position.y
                    z = msg.position.z
                    if datum_lat is not None:
                        lat, lon, alt = pymap3d.enu2geodetic(x,y,z,datum_lat, datum_lon, datum_alt)
                    else:
                        lat, lon, alt = None, None, None
                elif msg._type == "geometry_msgs/PoseStamped":
                    x = msg.pose.position.x
                    y = msg.pose.position.y
                    z = msg.pose.position.z
                    if datum_lat is not None:
                        lat, lon, alt = pymap3d.enu2geodetic(x,y,z,datum_lat, datum_lon, datum_alt)
                    else:
                        lat, lon, alt = None, None, None
                elif msg._type == "geometry_msgs/PoseWithCovariance":
                    x = msg.pose.position.x
                    y = msg.pose.position.y
                    z = msg.pose.position.z
                    if datum_lat is not None:
                        lat, lon, alt = pymap3d.enu2geodetic(x,y,z,datum_lat, datum_lon, datum_alt)
                    else:
                        lat, lon, alt = None, None, None
                elif msg._type == "geometry_msgs/PoseWithCovarianceStamped":
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z
                    if datum_lat is not None:
                        lat, lon, alt = pymap3d.enu2geodetic(x,y,z,datum_lat, datum_lon, datum_alt)
                    else:
                        lat, lon, alt = None, None, None
                results['lat'].append(lat)
                results['lon'].append(lon)
                results['alt'].append(alt)
                results['x'].append(x)
                results['y'].append(y)
                results['z'].append(z)


        if args.output.endswith('.csv'):
            df = pd.DataFrame.from_dict(results)
            df.to_csv(args.output, index=False)
        else:
            raise ValueError("Output format %s not supported" %os.path.splitext(args.output)[1])


if __name__ == "__main__":
    main(get_args())