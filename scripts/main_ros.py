#!/usr/bin/env python
# ^ DO NOT DELETE ABOVE ^ #

import rospy
import argparse
import configparser
import os
import Tkinter as tk
from std_msgs.msg import String

import config
import gui


def talker(args=None):

    # Handle arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default='/home/nick/GitHub/4d-agriculture/main_datapath/config.ini',
                        help='Path to config file')
    args = parser.parse_args()

    # Create config.ini if it doesn't already exist
    if not os.path.exists(args.config):
        config.create_config_global(args.config)

    config.verify_config(args.config)

    # Create a new configparser
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(args.config)

    # Create publisher node
    pub = rospy.Publisher('sample_points', String, queue_size=10)
    rospy.init_node('sample_gui', anonymous=False)

    # Create gui
    master = tk.Tk()
    gui.Main(master, parser, pub)

    # Run gui
    master.mainloop()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
