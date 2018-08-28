#!/usr/bin/env python
# ^ DO NOT DELETE ABOVE ^ #

import argparse
import configparser
import os
# import tkinter as tk
import Tkinter as tk

import config
import gui

try:
    import rospy
# except ModuleNotFoundError:
except ImportError:
    pass


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default='config.ini',
                        help='Path to config file')
    args = parser.parse_args()

    if not os.path.exists(args.config):
        config.create_config_global(args.config)

    config.verify_config(args.config)

    parser = configparser.ConfigParser(interpolation=None)
    parser.read(args.config)

    master = tk.Tk()
    main_gui = gui.Main(master, parser, None)

    master.mainloop()


if __name__ == '__main__':
    main()