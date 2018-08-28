# -*- coding: utf-8 -*-

import configparser
import json
import os.path
# import tkinter as tk
import Tkinter as tk
import numpy as np
import scipy.misc
import seaborn as sns
import matplotlib.backends.tkagg as tkagg
import matplotlib.pyplot as plt

from sys import platform
from threading import Thread
from datetime import datetime
# from tkinter import filedialog, messagebox
import tkMessageBox as messagebox
import tkFileDialog as filedialog
from webbrowser import open_new
from matplotlib import cm
from matplotlib.colors import ListedColormap
from matplotlib.backends.backend_agg import FigureCanvasAgg

import export
import plotting
import viewer
import custom

try:
    import rospy
    from std_msgs.msg import String
    print("ROS found, publishing enabled.")
# except ModuleNotFoundError:
except ImportError:
    print("ROS not found, publishing disabled.")
    pass


class Main:
    # def __init__(self, master: tk.Tk, config: configparser.ConfigParser, pub):
    def __init__(self, master, config, pub):
        """Create data and Initialize the gui for the master window.

        Reads in the data from the cache or file, then performs all of the
        transformations on the data. Then all of the widgets for the gui are
        created and drawn into the window.
        :param master - The tkinter window that the gui will be drawn into
        :param config - The parsed configuration information
        """

        # Window and argument initialization
        # ----------------------------------------------------------------------
        self.master = master  # master window
        self.master.resizable(0, 0)

        self.loading = tk.StringVar()
        self.loading.set("Loading Data")
        self.load_label = tk.Label(master, textvariable=self.loading,
                                   font=("Courier", 12))
        self.load_label.grid(column=2, row=1, padx=(20, 20), pady=(20, 20))
        self.master.update()

        self.pub = pub

        master.title("Sample point picker")
        self.config_global = config
        self.config_path = None
        self.config = None
        self.converter = None
        self.name_date_fmt = None
        # ----------------------------------------------------------------------

        # Variables and Prototypes
        # ----------------------------------------------------------------------

        # Hidden (Non-config) variables
        self.point_rad = 5              # Pixel radius of selected points
        self.remove_rad = 25            # Max dist from center to remove point
        self.mask_scale_rad = 2         # Radius when downsampling the mask
        self.mask_threshold = (20, 40)  # The cutoffs for low-med-high stress
        self.date_disp = '%Y-%m-%d'     # The date format for the display box

        # OS Specific Variables (Account for UI differences)
        if platform.startswith('linux'):
            cbox_pady = (6, 0)  # Add extra space between data mode buttons
            cbox_spc = ' '      # Add space between box and label for data modes
            btn_padx = (5, 5)   # Reduce space between the buttons
        else:
            cbox_pady = (0, 0)
            cbox_spc = ''
            btn_padx = (10, 10)

        # load_data() prototypes
        self.height_raw = None   # Raw height data as 3d numpy array
        self.dates_raw = None    # Sorted list of dates for each snapshot
        self.path_coords = None  # Raw gps coordinates of rover path
        self.sat_image = None    # Data for satellite image background

        # create_datasets() prototypes
        self.height_4d = None  # 4D DataSet with height data at each snapshot
        self.growth_4d = None  # 4D DataSet with growth data at each snapshot
        self.canopy_4d = None  # 4D DataSet with canopy data at each snapshot
        self.custom_4d = None  # 4D DataSet with data from custom_function()
        self.active_4d = None  # 4D DataSet with sum of selected overlays

        # analyze_stress() prototypes
        self.stress_regions = None  # Mask of high stress regions
        self.stress_map_4d = None   # 4D DataSet with stress map over time
        self.path = None            # Contains data defining path spline
        self.path_points = None     # List of locations on the path spline

        # gui elements
        self.photo = None       # Holds the currently drawn graph
        self.cmap = None        # Holds the colormap for the active data
        self.label = ""         # Holds the string for the graph units
        self.data_trans = None  # Transform obj between pixel/data coordinates
        self.points = []        # List of object ids for selected points
        self.points_x = []      # List of x coordinates of points
        self.points_y = []      # List of y coordinates of points
        self.points_event = []  # List of events corresponding to points
        # ----------------------------------------------------------------------

        # Find all of the available yearly datasets
        # ----------------------------------------------------------------------
        # Create list of all folders in data path
        main_dir = os.listdir(self.config_global['Path']['DataPath'])

        # Parse list for all xxxx_height_data folders and extract years
        # Note: If this code is being used in the year 10000 this will break
        remove_index = []
        year_nums = []
        for i in range(len(main_dir)):
            if main_dir[i][5:] != 'height_data':
                remove_index.append(i)
            else:
                year_nums.append(int(main_dir[i][0:4]))
        self.year_list = np.delete(main_dir, remove_index).tolist()

        # Select initial year as most recent available
        self.curr_year = np.amax(year_nums)
        # ----------------------------------------------------------------------

        # Perform analysis on newest year's data
        # ----------------------------------------------------------------------
        # Read in the data for the newest year
        self.check_config()
        self.load_data()
        self.update_load()

        # Create 4d datasets for each of the visualization modes
        self.create_datasets()
        self.update_load()

        # Determine the regions of stress for the data
        self.analyze_stress()
        self.update_load()
        # ----------------------------------------------------------------------

        # Create GUI
        # ----------------------------------------------------------------------

        # Background for Matplotlib figure
        # ----------------------------------------------------------------------
        self.plot_base = tk.Canvas(master, width=795, height=595, bd=2,
                                   relief='groove')
        self.plot_base.grid(row=2, column=1, padx=(5, 5))
        self.plot_base.grid_propagate(0)
        self.plot_base.bind('<Button-1>', self.place_point)
        self.plot_base.bind('<Button-3>', self.delete_point)
        # ----------------------------------------------------------------------

        # Control container: right of plot
        # ----------------------------------------------------------------------
        self.sidebar = tk.Frame(self.master, width=150)
        self.sidebar.grid(row=2, column=2, sticky='ns')
        self.sidebar.grid_propagate(0)

        #    Data Visualization Options Menu
        self.custom_frame = tk.Frame(self.sidebar, bd=2, relief='groove')
        self.custom_frame.pack(side=tk.TOP, padx=(10, 5), pady=(10, 10),
                               fill='both')

        self.custom_title = tk.Label(self.custom_frame,
                                     text='Data Visualization:',
                                     justify=tk.LEFT)
        self.custom_title.grid(row=1, column=1, padx=(10, 5), pady=(4, 0))

        #    Data Visualization Options Menu Elements
        button_left_pad = 14
        self.sat = tk.IntVar()
        self.sat_overlay_button = tk.Checkbutton(self.custom_frame,
                                                 variable=self.sat,
                                                 text=cbox_spc + "Satellite",
                                                 command=self.change_data_cb)
        self.sat_overlay_button.grid(row=2, column=1, sticky="w",
                                     padx=(button_left_pad, 0), pady=cbox_pady)
        self.chb = tk.IntVar()
        self.custom_height_button = tk.Checkbutton(self.custom_frame,
                                                   variable=self.chb,
                                                   text=cbox_spc + "Height",
                                                   command=self.change_data_cb)
        self.custom_height_button.grid(row=3, column=1, sticky="w",
                                       padx=(button_left_pad, 0),
                                       pady=cbox_pady)
        self.cgb = tk.IntVar()
        self.custom_growth_button = tk.Checkbutton(self.custom_frame,
                                                   variable=self.cgb,
                                                   text=cbox_spc + "Growth",
                                                   command=self.change_data_cb)
        self.custom_growth_button.grid(row=4, column=1, sticky="w",
                                       padx=(button_left_pad, 0),
                                       pady=cbox_pady)
        self.cnb = tk.IntVar()
        self.custom_canopy_button = tk.Checkbutton(self.custom_frame,
                                                   variable=self.cnb,
                                                   text=cbox_spc + "Canopy",
                                                   command=self.change_data_cb)
        self.custom_canopy_button.grid(row=5, column=1, sticky="w",
                                       padx=(button_left_pad, 0),
                                       pady=cbox_pady)
        self.cmb = tk.IntVar()
        self.custom_mask_button = tk.Checkbutton(self.custom_frame,
                                                 variable=self.cmb,
                                                 text=cbox_spc + "Stress Map",
                                                 command=self.change_data_cb)
        self.custom_mask_button.grid(row=6, column=1, sticky="w",
                                     padx=(button_left_pad, 0), pady=cbox_pady)

        self.rvb = tk.IntVar()
        self.rover_path_button = tk.Checkbutton(self.custom_frame,
                                                variable=self.rvb,
                                                text=cbox_spc + "Rover Path",
                                                command=self.change_data_cb)
        self.rover_path_button.grid(row=7, column=1, sticky="w",
                                    padx=(button_left_pad, 0), pady=cbox_pady)

        self.ccb = tk.IntVar()
        self.custom_custom_button = tk.Checkbutton(self.custom_frame,
                                                   variable=self.ccb,
                                                   text=cbox_spc + "Stress "
                                                                   "Regions",
                                                   command=self.change_data_cb)
        self.custom_custom_button.grid(row=8, column=1, sticky="w",
                                       padx=(button_left_pad, 0),
                                       pady=cbox_pady)

        self.cb_norm = tk.IntVar()
        self.custom_normalize_button = tk.Checkbutton(self.custom_frame,
                                                      variable=self.cb_norm,
                                                      text=cbox_spc +
                                                      "Normalize Data",
                                                      command=self.
                                                      change_data_cb)
        self.custom_normalize_button.grid(row=9, column=1,
                                          padx=(0, 0), pady=(14, 10))

        #    Statistics Panel
        self.stat_frame = tk.Frame(self.sidebar, bd=2, relief='groove')
        self.stat_frame.pack(side=tk.TOP, padx=(10, 5),
                             pady=(8, 0), fill='both')
        self.stats_title = tk.Label(self.stat_frame, text='Plot Statistics:',
                                    justify=tk.LEFT)
        self.stats_title.grid(row=1, column=1, padx=(18, 18), pady=(4, 0))

        self.stats_str = tk.StringVar()
        self.stats = tk.Label(self.stat_frame, textvariable=self.stats_str,
                              justify=tk.LEFT, font=("Courier", 8))
        self.stats.grid(row=2, column=1, padx=(4, 4))

        #    Export Stress Map Button
        self.plot3d_button = tk.Button(self.sidebar, text='Export Stress Map',
                                       command=self.export_map)
        self.plot3d_button.pack(side=tk.TOP, pady=(10, 0), padx=(10, 5),
                                fill='both')

        #    Export Points Button
        self.save_button = tk.Button(self.sidebar, text='Export Points',
                                     command=self.export_json_cb)
        self.save_button.pack(side=tk.TOP, pady=(10, 0), padx=(10, 5),
                              fill='both')

        # Label for point exporting
        self.export_label = tk.Label(self.sidebar, text="Exporting Points",
                                     font=("Courier", 10))
        # ----------------------------------------------------------------------

        # Control container: above plot
        # ----------------------------------------------------------------------
        self.top_controls = tk.Frame(self.master, width=853, height=90)
        self.top_controls.grid(row=1, column=1, sticky='w', columnspan=2)

        # Open configuration window
        self.config_button = tk.Button(self.top_controls, text='Configuration',
                                       command=self.config_window)
        self.config_button.grid(row=0, column=0, padx=btn_padx, pady=(5, 5))

        # Previous date control
        self.prev_button = tk.Button(self.top_controls, text='← Previous',
                                     command=self.prev_data_cb)
        self.prev_button.grid(row=0, column=1, padx=btn_padx, pady=(5, 5))

        # Current date info label
        self.current_date = tk.StringVar(self.master,
                                         value=self.active_4d.get_date().
                                         strftime(self.date_disp))
        self.date_label = tk.Label(self.top_controls,
                                   textvariable=self.current_date, bg='white')
        self.date_label.grid(row=0, column=2, padx=(0, 0), pady=(5, 5))

        # Next date control
        self.next_button = tk.Button(self.top_controls, text='  Next →  ',
                                     command=self.next_data_cb)
        self.next_button.grid(row=0, column=3, padx=btn_padx, pady=(5, 5))

        # Switch Year Dropdown
        self.year_var = tk.StringVar()
        self.year_var.set("Change Year...  ")
        self.year_menu = tk.OptionMenu(self.top_controls, self.year_var,
                                       *self.year_list,
                                       command=self.change_year)
        self.year_menu.grid(row=0, column=4, padx=btn_padx, pady=(5, 5))

        # Suggest Points element
        self.suggest_button = tk.Button(self.top_controls,
                                        text='Suggest Points',
                                        command=self.suggest_points)
        self.suggest_button.grid(row=0, column=5, padx=btn_padx)

        # Clear Points element
        self.suggest_button = tk.Button(self.top_controls,
                                        text='Clear Points',
                                        command=self.clear_points)
        self.suggest_button.grid(row=0, column=6, padx=btn_padx)

        # Open gif visualization
        self.suggest_button = tk.Button(self.top_controls,
                                        text='Gif Visualization',
                                        command=self.gif_viewer)
        self.suggest_button.grid(row=0, column=7, padx=btn_padx)
        # ----------------------------------------------------------------------
        # ----------------------------------------------------------------------

        # Display initial data
        # ----------------------------------------------------------------------
        self.sat.set(1)
        self.custom_height_button.invoke()

        # Remove loading text
        self.load_label.grid_forget()
        # ----------------------------------------------------------------------

    def analyze_stress(self):

        # Create stress map and regions of high stress
        stress_map = plotting.create_stress_map(self.height_4d.data,
                                                self.canopy_4d.data,
                                                self.mask_scale_rad,
                                                self.mask_threshold)
        self.stress_regions = plotting.define_regions(stress_map[:, :, -1],
                                                      self.mask_scale_rad)
        self.stress_map_4d = plotting.DataSet4D(stress_map, self.dates_raw)

        # Create a spline from the rover path and eval at linspaced points
        self.path = plotting.create_path_splines(self.path_coords,
                                                 self.height_4d.data.shape,
                                                 self.converter)
        self.path_points = plotting.evaluate_path_spline(self.path, 5000)

    def change_data(self):
        """Callback function whenever custom data window is changed.

        Updates self.active_4d with a combination of whatever checkboxes are
        selected, and then normalizes the data if that checkbox is selected.
        The date is also reset every time this function is called in order to
        account for growth's length-1"""

        # Reset custom data to zeros
        self.active_4d.data = np.zeros(self.height_raw.shape)
        self.active_4d.set_dates(self.height_4d)
        self.label = ""

        # Don't do anything else if none of the data modes are selected
        if not (self.chb.get() or self.cnb.get() or self.ccb.get() or
                self.cmb.get() or self.cgb.get()):
            plt.gcf().clf()

            if self.sat.get():
                self.draw_image()
                self.draw_figure()
                self.data_trans = plt.gcf().gca().transData.inverted()
                self.redraw_points()
            if self.rvb.get():
                self.draw_gps_points()

            # Re-enable everything inside custom_frame before exiting
            for child in self.custom_frame.winfo_children():
                child.configure(state='normal')
            return

        # Normalize data if normalize CheckButton is selected
        if self.cb_norm.get() != 0:
            height = (self.height_4d.data / np.amax(self.height_4d.data,
                                                    axis=(0, 1))) * 100
            growth = (self.growth_4d.data / np.amax(self.growth_4d.data,
                                                    axis=(0, 1))) * 100
            canopy = (self.canopy_4d.data / np.amax(self.canopy_4d.data,
                                                    axis=(0, 1))) * 100
            stress_map = (self.stress_map_4d.data
                          / np.amax(self.stress_map_4d.data, axis=(0, 1))) * 100
            custom = (self.custom_4d.data / np.amax(
                self.custom_4d.data, axis=(0, 1))) * 100
        else:
            height = self.height_4d.data
            growth = self.growth_4d.data
            canopy = self.canopy_4d.data
            stress_map = self.stress_map_4d.data
            custom = self.custom_4d.data

        # We know at this point that at least one dataset is selected,
        # so we can do the "+ name", "+ name and then remove the first two chars
        label = ""

        # Add data to custom based on selected CheckButtons
        if self.chb.get() != 0:
            self.active_4d.data = np.add(self.active_4d.data, height)
            label += " + meters"
            self.cmap = transparent_cmap(cm.get_cmap("GnBu"))
        if self.cgb.get() != 0:
            self.active_4d.data = np.add(self.active_4d.data, growth)
            label += " + meters/day"
            self.cmap = transparent_cmap(cm.get_cmap("GnBu"))
        if self.cnb.get() != 0:
            self.active_4d.data = np.add(self.active_4d.data, canopy)
            label += " + leaf area index"
            self.cmap = transparent_cmap(cm.get_cmap("GnBu"))
        if self.cmb.get() != 0:
            self.active_4d.data = np.add(self.active_4d.data, stress_map)
            label += " + stress frequency"
            self.cmap = transparent_cmap(cm.get_cmap("OrRd"))
        if self.ccb.get() != 0:
            self.active_4d.data = np.add(self.active_4d.data, custom)
            label += " + custom scale"
            self.cmap = transparent_cmap(cm.get_cmap("PuRd"))

        if (self.chb.get() + self.cnb.get() + self.ccb.get()
                + self.cmb.get() + self.cgb.get()) > 1:
            self.cmap = transparent_cmap(cm.get_cmap("BuPu"))

        self.label = label[3:]
        self.label = "Units: " + self.label

        # Refresh the image
        self.active_4d.reset_date()
        self.active_4d.refresh_statistics()
        self.current_date.set(self.active_4d.get_date().
                              strftime(self.date_disp))
        self.show_data()

        # Re-enable everything inside custom_frame before exiting
        for child in self.custom_frame.winfo_children():
            child.configure(state='normal')

    def change_data_cb(self):

        # Disable all children in frame while the data is being changed
        for child in self.custom_frame.winfo_children():
            child.configure(state='disable')

        # Call change_data in a new thread
        change_data_thread = Thread(target=self.change_data)
        change_data_thread.start()

    def change_year(self, year):

        # Parse year from folder name
        year = int(year[0:4])

        # Don't do anything if changing to same year
        if year == self.curr_year:
            return

        # Create loading indicator
        self.plot_base.create_text(380, 250,
                                   text="Changing Year to " + str(year),
                                   justify=tk.CENTER,
                                   font=("default", 20))
        self.plot_base.create_text(380, 300, text="Please Wait",
                                   justify=tk.CENTER, font=("default", 20))

        self.master.update()

        # Update curr_year variable, then reset data
        self.curr_year = year

        # Read in the new year's data
        self.check_config()
        self.load_data()

        # Update the datasets with the new year's data
        self.create_datasets()

        # Reevaluate the stress map for the new year
        self.analyze_stress()

        # Reset overlays and point arrays
        self.points = []
        self.points_x = []
        self.points_y = []
        self.points_event = []
        self.chb.set(0)
        self.cgb.set(0)
        self.cnb.set(0)
        self.ccb.set(0)
        self.cmb.set(0)
        self.rvb.set(0)
        self.cb_norm.set(0)

        # Display initial data
        self.sat.set(1)
        self.custom_height_button.invoke()

    def check_config(self):
        # Data path to yaerly config, if it exists
        data_path = os.path.abspath(self.config_global['Path']['DataPath']
                                    + r"/" + str(self.curr_year)
                                    + "_height_data/config.ini")

        # Reset config parser
        self.config = configparser.ConfigParser(interpolation=None)

        # If there is a yearly config, use it. If not, use the global
        if os.path.exists(data_path):
            self.config.read(data_path)
            self.config_path = data_path
            self.config['Path']['DataPath'] =\
                self.config_global['Path']['DataPath']
        else:
            self.config_path = self.config_global['Path']['DataPath'] \
                               + "/config.ini"
            self.config.read(self.config_path)

        # Update config-reliant variables
        self.converter = plotting.LatLngConverter(self.config)
        self.name_date_fmt = self.config['Files']['filename']

    def clear_points(self):

        # Remove all existing points
        for i in range(len(self.points)):
            # Remove coordinates from list
            del self.points_x[0]
            del self.points_y[0]

            # Delete visual of point, then remove it from the list
            self.plot_base.delete(self.points[0])
            del self.points[0]
            del self.points_event[0]

    def config_window(self):
        """Open up a new configuration window."""

        config_master = tk.Toplevel(self.master)
        config_master.bind("<Destroy>", lambda x: self.config_window_destroy())

        ConfigWindow(config_master, self.config, self.config_path)

        self.master.withdraw()
        config_master.mainloop()

    def config_window_destroy(self):
        self.master.deiconify()
        self.check_config()

    def create_datasets(self):

        # Create 4D data elements for each data set
        #    Height data
        heights = plotting.filter_outliers(self.height_raw,
                                           np.float(self.config['Filter']
                                                    ['min_value']),
                                           np.float(self.config['Filter']
                                                    ['max_value']))
        self.height_4d = plotting.DataSet4D(self.height_raw, self.dates_raw)

        #    Growth data
        growth = self.height_4d.derivative()
        self.growth_4d = plotting.DataSet4D(growth, self.dates_raw)

        #    Canopy data
        canopy = plotting.canopy_cover(heights, 3)
        self.canopy_4d = plotting.DataSet4D(canopy, self.height_4d.dates)

        #    Custom function data
        # custom_data = custom.custom_process(self.height_4d.data)
        custom_data = plotting.define_regions(
            plotting.create_stress_map(self.height_4d.data,
                                       self.canopy_4d.data,
                                       self.mask_scale_rad,
                                       self.mask_threshold)
            [:, :, -1],
            self.mask_scale_rad).map
        self.custom_4d = plotting.DataSet4D(custom_data, self.dates_raw)

        #    Create container for custom data window
        self.active_4d = plotting.DataSet4D(self.height_4d.data, self.dates_raw)

    def data_cache(self, cache_file):
        """Create a cache file(.npz) for the raw data."""
        if not os.path.exists(cache_file):
            np.save(cache_file, self.height_raw)

    def delete_point(self, event):
        """Right click callback removes a point within range of the cursor.

        This function gets the cursor's x and y pixel coordinates from the
        event, then converts it to data coordinates for the figure. If there
        is a point in the list that is within self.remove_rad pixels from the
        cursor, this function then deleted that point.
        :param event - The click event that contains the
                       pixel coordinates of the click
        """

        # Transform the pixel location of the click to a data location
        x, y = self.data_trans.transform((event.x, event.y))
        rows, cols, _ = self.height_4d.data.shape
        y = rows - y

        # Check that point is in bounds and there is a point to remove
        if not ((0 <= y <= rows) and (0 <= x <= cols)):
            return

        # Loop through points list and find a point in range
        for i in range(len(self.points_x)):
            if abs(self.points_x[i] - x) < self.remove_rad \
                   and abs(self.points_y[i] - y) < self.remove_rad:

                # Remove coordinates from list
                del self.points_x[i]
                del self.points_y[i]

                # Delete visual of point, then remove it from the list
                self.plot_base.delete(self.points[i])
                del self.points[i]
                del self.points_event[i]
                break

    def draw_figure(self, loc=(0, 0)):
        """Draw a new figure onto the canvas and redraw the points over it.

        This function creates a new 'photo' from the plot, and displays it
        onto the tkinter canvas widget. It then assigns the self.photo
        instance variable, which must be kept active for the image to remain
        visible.
        """

        # Make sure instructions are retained when new figure is drawn
        plt.gcf().set_dpi(100)
        plt.gcf().set_figwidth(8)
        plt.gcf().set_figheight(6)
        plt.xlabel("Left click to place points. Right click on a point to "
                   "remove it.", fontsize=10)

        # Create a PhotoImage object for the figure
        figure_canvas_agg = FigureCanvasAgg(plt.gcf())
        figure_canvas_agg.draw()
        f_x, f_y, f_w, f_h = plt.gcf().bbox.bounds
        f_w, f_h = int(f_w), int(f_h)
        photo = tk.PhotoImage(master=self.plot_base, width=f_w, height=f_h)

        # Position: convert from top-left anchor to center anchor
        self.plot_base.create_image(loc[0] + f_w / 2, loc[1] + f_h / 2,
                                    image=photo)
        tkagg.blit(photo, figure_canvas_agg.get_renderer()._renderer,
                   colormode=2)
        self.photo = photo  # this handle must be kept alive
        self.redraw_units()

    def draw_gps_point(self):
        """Plot the base anchor point for reference"""

        # Origin Point gps location
        p = (0, 31.473566, -83.529994)

        # This gives us the data coordinates from the gps location
        data_y = int(self.converter.lat_to_y(p[1]))
        data_x = int(self.converter.lng_to_x(p[2]))

        # x-y swap done in converter method
        rows, cols, _ = self.height_4d.data.shape
        data_x = cols - data_x  # pixel/matrix rows are reversed
        data_y = rows - data_y

        # Transform the data location of the point to a pixel location
        pix_x, pix_y = self.data_trans.inverted().transform((data_x, data_y))

        self.plot_base.create_oval(pix_x - 3,
                                   pix_y - 3,
                                   pix_x + 3,
                                   pix_y + 3, fill='purple')

    def draw_gps_points(self):
        """Draw each path gps coordinate onto the screen"""

        points = self.path_points

        # Update axis transformation
        self.data_trans = plt.gcf().gca().transData.inverted()

        for p in points.itertuples():

            # Unpack the tuple into the x and y coordinates
            data_x = p[1]
            data_y = p[2]

            # Skip to the next point if it is out of bounds of the plot
            rows, cols, _ = self.height_4d.data.shape
            rows -= 3
            cols -= 3
            if not ((0 <= data_y <= rows) and (0 <= data_x <= cols)):
                continue

            rows, cols, _ = self.height_4d.data.shape
            pix_x = cols - data_x  # pixel/matrix rows are reversed
            pix_y = rows - data_y

            # Transform the data location of the point to a pixel location
            pix_x, pix_y = self.data_trans.inverted().transform((pix_x, pix_y))

            self.plot_base.create_oval(pix_x - 1,
                                       pix_y - 1,
                                       pix_x + 1,
                                       pix_y + 1, fill='orange')

    def draw_image(self):

        # Plot the image onto the figure
        plt.imshow(self.sat_image)

        # Remove tick marks from plot
        plt.xticks([])
        plt.yticks([])

    def export_json(self):
        """Export the selected points in GeoJSON format.

        Creates a GeoJSON object with the latitudes and longitudes of each of
        the points selected in the program. This file will then be used by the
        autonomous vehicle to collect leaf and soil samples at these locations.
        """

        if len(self.points_x) == 0:
            messagebox.showinfo('Error', 'No points selected')
            self.export_label.pack_forget()
            return

        if len(self.points_x) > int(self.config['Points']['MaxPoints']):
            messagebox.showinfo('Error',
                                'Too many points selected. Please export with '
                                + self.config['Points']['MaxPoints']
                                + ' or fewer points selected.')
            self.export_label.pack_forget()
            return

        ftypes = [('JSON binary', '*.json'), ('Text', '*.txt')]
        f = filedialog.asksaveasfile(mode='w', defaultextension='.json',
                                     filetypes=ftypes,
                                     initialfile='points.json',
                                     title='Export points')
        if f is None:
            self.export_label.pack_forget()
            return

        # Orient both sets of points to anchor in the top right
        rows, cols, _ = self.height_4d.data.shape
        self.points_y = np.subtract(rows, self.points_y)
        self.points_x = np.subtract(self.points_x, cols).tolist()
        self.points_y = np.subtract(self.points_y, rows).tolist()

        feature_collection = export.points_to_collection(self.points_x,
                                                         self.points_y,
                                                         self.converter,
                                                         self.path_coords)
        json.dump(feature_collection, f, indent=2)

        try:
            self.pub.publish(String(json.dumps(feature_collection)))
            rospy.loginfo(rospy.get_caller_id() + ' Published %s sample points',
                          str(len(self.points_x)))
        except AttributeError:
            pass
        print("Points Exported")
        self.export_label.pack_forget()

    def export_json_cb(self):
        """Start a new thread to export the currently selected points"""

        point_export_thread = Thread(target=self.export_json)
        point_export_thread.start()

        # Show label that points are currently being exported
        self.export_label.pack(side=tk.TOP, padx=(10, 5), pady=(8, 0))

    def export_map(self):
        """Export map of stressed areas as a csv.

        Create a csv with the stress map data so that it can be used in
        future years to show areas that are stressed year after year."""

        final_map = self.stress_map_4d.data[:, :, -1]

        # Make sure that the data has stressed areas
        if len(np.nonzero(final_map)) <= 0:
            messagebox.showinfo('Error', 'No stressed areas located in data')
            return

        # Create a new csv file to hold the information
        date = str(self.height_4d.get_dates()[-1].year)
        ftypes = [('CSV', '*.csv')]
        f = filedialog.asksaveasfile(mode='w', defaultextension='.csv',
                                     filetypes=ftypes,
                                     initialfile=date + '_stress_map',
                                     title='Export Stress Map')
        if f is None:
            return

        # Export the map data to the new csv file
        np.savetxt(f.name, final_map, delimiter=',')
        print("Stress Map Exported")

    def gif_viewer(self):

        gif_window = tk.Toplevel(self.master)
        gif_window.bind("<Destroy>", lambda x: self.master.deiconify())

        viewer.Viewer(gif_window, self.active_4d, self.sat_image, self.cmap)

        self.master.withdraw()
        gif_window.mainloop()

    def load_data(self):

        # Read in data files
        data_path = os.path.abspath(self.config_global['Path']['DataPath']
                                    + r"/" + str(self.curr_year)
                                    + "_height_data")
        dates = [datetime.strptime(f, self.name_date_fmt)
                 for f in os.listdir(data_path) if f[-4:] == '.csv']
        self.dates_raw = sorted(dates)
        cache_name, _ = os.path.splitext(data_path)
        cache_file = cache_name + '/cache.npy'

        # Automatically caches to numpy binary format; faster than reading CSVs
        self.read_cache(data_path, cache_file)
        self.data_cache(cache_file)

        # Add extra dimension in case of only one data
        # set so it can be used as 3D array
        if len(self.height_raw.shape) == 2:
            self.height_raw = np.expand_dims(self.height_raw, axis=2)

        # Load in rover path data
        self.path_coords = export.read_path_file(self.config['Path']['DataPath']
                                                 + r"/rover_path_data/gps_info_"
                                                 + str(self.curr_year) + ".csv")

        # Open satellite overlay image and resize to fit data
        pth = os.path.abspath(self.config['Path']['DataPath']
                              + r"/" + str(self.curr_year)
                              + "_height_data/field_sat_"
                              + str(self.curr_year) + ".png")
        self.sat_image = plt.imread(fname=pth)
        scale = (self.height_raw.shape[0], self.height_raw.shape[1])
        self.sat_image = scipy.misc.imresize(self.sat_image, scale)

    def next_data(self):
        """Call data.next_data() and update the window if the date changes."""

        # Do nothing if there is no data selected
        if not (self.chb.get() or self.cnb.get() or self.ccb.get() or
                self.cmb.get() or self.cgb.get()):
            self.next_button.config(state="normal")
            return

        self.active_4d.next_data()
        self.current_date.set(self.active_4d.get_date().
                              strftime(self.date_disp))
        self.show_data()
        self.next_button.config(state="normal")

    def next_data_cb(self):

        self.next_button.config(state="disabled")

        next_date_thread = Thread(target=self.next_data)
        next_date_thread.start()

    def place_point(self, event):
        """Left click callback places a point at the position of the cursor.

        This function gets the cursor's x and y pixel coordinates from the
        event, then converts it to data coordinates for the figure. It then
        creates a new point at that location and draws it onto the canvas.
        :param event - The click event that contains the
                       pixel coordinates of the click
        """

        # Transform the pixel location of the click to a data location
        x, y = self.data_trans.transform((event.x, event.y))
        rows, cols, _ = self.height_4d.data.shape
        y = rows - y  # pixel/matrix rows are reversed
        if not ((0 <= y <= rows) and (0 <= x <= cols)):
            return

        # Append the point to the end of the list
        self.points_x.append(x)
        self.points_y.append(y)

        # Draw the new point onto the figure
        o = self.plot_base.create_oval(event.x - self.point_rad,
                                       event.y - self.point_rad,
                                       event.x + self.point_rad,
                                       event.y + self.point_rad, fill='red')
        self.points.append(o)
        self.points_event.append(event)

    def prev_data(self):
        """Call data.prev_data() and update the window if the date changes."""

        # Do nothing if there is no data selected
        if not (self.chb.get() or self.cnb.get() or self.ccb.get() or
                self.cmb.get() or self.cgb.get()):
            self.prev_button.config(state="normal")
            return

        self.active_4d.prev_data()
        self.current_date.set(self.active_4d.get_date().
                              strftime(self.date_disp))
        self.show_data()
        self.prev_button.config(state="normal")

    def prev_data_cb(self):
        self.prev_button.config(state="disabled")

        prev_date_thread = Thread(target=self.prev_data)
        prev_date_thread.start()

    def read_cache(self, data_path, cache_file):
        """Read in the cache file(.npz) or .csv files for the dataset."""
        if os.path.exists(cache_file):
            self.height_raw = np.load(cache_file)
        else:
            self.height_raw = export.read_folder(data_path,
                                                 self.name_date_fmt)

    def redraw_points(self):
        """Redraw each selected point back onto the screen."""

        self.points = []  # Clear out old shape id numbers
        rows, _, _ = self.height_4d.data.shape
        for i in range(len(self.points_x)):
            x = self.points_x[i]
            y = self.points_y[i]
            y = rows - y
            x, y = self.data_trans.inverted().transform((x, y))
            o = self.plot_base.create_oval(x - self.point_rad,
                                           y - self.point_rad,
                                           x + self.point_rad,
                                           y + self.point_rad, fill='red')
            self.points.append(o)  # Repopulate list with new id numbers

    def redraw_units(self):
        self.plot_base.create_text(640, 300, text=self.label,
                                   justify=tk.CENTER,
                                   font=("default", 11), angle=270)

    def show_data(self):
        """Display the active data in the gui.

        Creates a new heatmap for the current active data. Then the function
        draws the figure onto the canvas and updates the stats in the
        statistics pane.
        """
        plt.ioff()
        plt.gcf().clf()
        data = self.active_4d.get_data()
        sns.heatmap(data, vmin=0, vmax=self.active_4d.max_val,
                    cmap=self.cmap,
                    xticklabels=False, yticklabels=False, square=1)

        plt.tight_layout()
        if self.sat.get() != 0:
            self.draw_image()
        self.draw_figure()
        if self.rvb.get():
            self.draw_gps_points()
        self.update_stats()

        # transformer object, pixel coordinates to data coordinates
        self.data_trans = plt.gcf().gca().transData.inverted()

        self.redraw_points()

    def suggest_points(self):
        """Create a list of points in stress-likely areas of the plot.

        This function uses the stress map to determine high stress areas of
        the plot and select random points within those high stress areas. The
        function then simulates mouse clicks at each of those locations to
        create points at those positions.
        """

        points = []

        # Remove all existing points
        self.clear_points()

        for region_data in self.stress_regions.regions:
            region = self.stress_regions.map[region_data]

            # Determine points per region based on size of region
            num_select = int(np.ceil(np.nonzero(region != 0)[0].size *
                             float(self.config["GPS"]["size"])
                             / (100 * int(self.config["Points"]
                                          ["MaxPoints"]))))

            # Get list of nonzero values (in region), return if empty
            count = np.nonzero(region)  # This returns r,c not x,y
            if len(count[0]) <= 0:
                return

            # Select points from the list of nonzero values
            selected = np.random.random_integers(0, len(count[0]) - 1,
                                                 num_select)

            # Determine random points from array
            for i in range(len(selected)):

                # convert from r,c to x,y and store in a tuple
                x_offset = region_data[1].start
                y_offset = region_data[0].start
                point = [x_offset + count[1][selected[i]],
                         y_offset + count[0][selected[i]]]

                # Make sure this new point is not close to any other points
                # If so, delete it
                for p in points:
                    if (abs(point[0] - p[0]) < 20) \
                          and (abs(point[1] - p[1]) < 20):
                        break
                else:
                    points.append(point)

        rows, cols, _ = self.height_4d.data.shape

        for i in range(len(points)):
            points[i][1] = rows - points[i][1]
            pixels = self.data_trans.inverted().transform(points[i])

            # Generate an event object for a mouse click at the point
            self.plot_base.event_generate('<Button-1>',
                                          x=pixels[0], y=pixels[1])

    def update_load(self):
        self.loading.set(self.loading.get() + ".")
        self.master.update()

    def update_stats(self):
        """Update the string for the statistics pane for the current plot."""
        self.stats_str.set('''
     Average: {0:5.2f}
Standard Dev: {1:5.2f}
     Maximum: {2:5.2f}
  % Coverage: {3:5.2f}
        '''.format(self.active_4d.average, self.active_4d.std_dev,
                   self.active_4d.max_val, self.active_4d.pct_coverage))

    def open_pyplot(self):
        # Create a pyplot window for zoom and scaling
        # NOT IN USE YET

        plt.figure(1)
        ax = plt.gca()
        sns.heatmap(self.active_4d.get_data(), vmin=0,
                    vmax=self.active_4d.max_val,
                    cmap=self.cmap, ax=ax,
                    xticklabels=False, yticklabels=False, square=1)
        plt.show(1)


def transparent_cmap(cmap):

    my_cmap = cmap(np.arange(cmap.N))
    my_cmap[:, -1] = np.linspace(0, 1, cmap.N)
    my_cmap = ListedColormap(my_cmap)

    return my_cmap


class ConfigWindow:
    # def __init__(self, master: tk.Toplevel,
    #              config_arg: configparser.ConfigParser, config_path):
    def __init__(self, master, config_arg, config_path):
        """Initialize the gui for the configuration window.

        Creates all of the widgets for the configuration window.
        :param config - The parsed configuration information
        """

        self.master = master
        self.master.title('Configure')
        self.master.resizable(0, 0)
        self.config = config_arg
        self.config_path = config_path
        self.new_path = None

        # Latitude input
        self.lat_label = tk.Label(self.master, text='Origin latitude')
        self.lat_label.grid(row=1, column=1, padx=(5, 5),
                            pady=(5, 5), sticky='w')
        lat_v = tk.StringVar(self.master,
                             value=self.config['GPS']['origin_lat'])
        self.lat_box = tk.Entry(self.master, textvariable=lat_v)
        self.lat_box.grid(row=1, column=2, padx=(5, 5), pady=(5, 5))

        # Longitude input
        self.lng_label = tk.Label(self.master, text='Origin longitude')
        self.lng_label.grid(row=2, column=1, padx=(5, 5),
                            pady=(5, 5), sticky='w')
        lng_v = tk.StringVar(self.master,
                             value=self.config['GPS']['origin_lng'])
        self.lng_box = tk.Entry(self.master, textvariable=lng_v)
        self.lng_box.grid(row=2, column=2, padx=(5, 5), pady=(5, 5))

        # Scale input
        self.scale_label = tk.Label(self.master, text='Grid Size (meters)')
        self.scale_label.grid(row=3, column=1, padx=(5, 5),
                              pady=(5, 5), sticky='w')
        scale_v = tk.StringVar(self.master, value=self.config['GPS']['size'])
        self.scale_box = tk.Entry(self.master, textvariable=scale_v)
        self.scale_box.grid(row=3, column=2, padx=(5, 5), pady=(5, 5))

        # Minimum std filter input
        self.min_filter_label = tk.Label(self.master, text='Minimum Std Filter')
        self.min_filter_label.grid(row=4, column=1, padx=(5, 5),
                                   pady=(5, 5), sticky='w')
        min_value = tk.StringVar(self.master,
                                 value=self.config['Filter']['min_value'])
        self.min_filter_box = tk.Entry(self.master, textvariable=min_value)
        self.min_filter_box.grid(row=4, column=2, padx=(5, 5), pady=(5, 5))

        # Maximum std filter input
        self.max_filter_label = tk.Label(self.master, text='Maximum Std Filter')
        self.max_filter_label.grid(row=5, column=1, padx=(5, 5),
                                   pady=(5, 5), sticky='w')
        max_value = tk.StringVar(self.master,
                                 value=self.config['Filter']['max_value'])
        self.max_filter_box = tk.Entry(self.master, textvariable=max_value)
        self.max_filter_box.grid(row=5, column=2, padx=(5, 5), pady=(5, 5))

        # Maximum selected points input
        self.max_points_label = tk.Label(self.master,
                                         text='Max Selected Points')
        self.max_points_label.grid(row=6, column=1, padx=(5, 5),
                                   pady=(5, 5), sticky='w')
        max_points = tk.StringVar(self.master,
                                  value=self.config['Points']['MaxPoints'])
        self.max_points_box = tk.Entry(self.master, textvariable=max_points)
        self.max_points_box.grid(row=6, column=2, padx=(5, 5), pady=(5, 5))

        # Data Path input
        self.path_label = tk.Label(self.master, text='Data Path')
        self.path_label.grid(row=7, column=1, padx=(5, 5), pady=(5, 5))
        path_v = tk.StringVar(self.master,
                              value=self.config['Path']['DataPath'])
        self.path_box = tk.Entry(self.master, textvariable=path_v)
        self.path_box.grid(row=7, column=2, padx=(5, 5), pady=(5, 5))

        # Data Path browse button
        self.browse_button = tk.Button(self.master, text='Browse',
                                       command=lambda: self.browse_path(path_v))
        self.browse_button.grid(row=7, column=3, padx=(0, 5), pady=(5, 5))

        # Filename input
        self.fname_label = tk.Label(self.master, text='Filename format')
        self.fname_label.grid(row=8, column=1, padx=(5, 5), pady=(5, 5))
        fname_v = tk.StringVar(self.master,
                               value=self.config['Files']['filename'])
        self.fname_box = tk.Entry(self.master, textvariable=fname_v)
        self.fname_box.grid(row=8, column=2, padx=(5, 5), pady=(5, 5))

        # Datetime format help button
        url = r'https://docs.python.org/3/library/' \
              r'datetime.html#strftime-strptime-behavior'
        self.qbutton = tk.Button(self.master, text=' ? ',
                                 command=lambda: open_new(url))
        self.qbutton.grid(row=8, column=3, padx=(0, 5), pady=(5, 5))

        # Save button
        self.save_button = tk.Button(self.master, text='Save',
                                     command=self.save)
        self.save_button.grid(row=9, column=2, padx=(5, 5),
                              pady=(5, 5), sticky='e')

    # def browse_path(self, path_v: tk.StringVar):
    def browse_path(self, path_v):
        """Prompt the user to select the path to the data."""

        self.new_path = filedialog.askdirectory(title="Select path to Data")
        path_v.set(self.new_path)

    def save(self):
        """Save the new configuration values."""

        lat = self.lat_box.get()
        lng = self.lng_box.get()
        size = self.scale_box.get()
        filter_min = self.min_filter_box.get()
        filter_max = self.max_filter_box.get()
        max_points = self.max_points_box.get()
        path = self.path_box.get()

        try:
            # Validate user input
            lat = float(lat)
            lng = float(lng)
            size = float(size)
            filter_min = float(filter_min)
            filter_max = float(filter_max)
            max_points = int(max_points)
            path = str(path)
        except ValueError as e:
            messagebox.showinfo('Error', e)
            return

        # Config parameters must be strings
        self.config['GPS']['origin_lat'] = str(lat)
        self.config['GPS']['origin_lng'] = str(lng)
        self.config['GPS']['size'] = str(size)
        self.config['Filter']['min_value'] = str(filter_min)
        self.config['Filter']['max_value'] = str(filter_max)
        self.config['Path']['DataPath'] = path
        self.config['Points']['MaxPoints'] = str(max_points)
        with open(self.config_path, 'w') as f:
            self.config.write(f)

        self.master.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    config = configparser.ConfigParser(interpolation=None)
    config.read('config.ini')
    gui = Main(root, config, None)

    root.mainloop()
