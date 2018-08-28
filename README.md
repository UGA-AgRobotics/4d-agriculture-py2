# 4d-agriculture-py2 specific README

This repo is 4d-agriculture converted to Python 2. In addition,
there is now a condensed requirements.txt for the Python libraries,
and it can run in a Docker container.

### Build and deploy without Docker:

1. Pull 4d-agriculture-py2 into ROS workspace's /src folder.

2. Install requirements from 4d-agriculture-py2/requirements.txt file. Make
   sure it's the python2 used by ROS.

	$ pip install -r requirements.txt

3. cd to workspace level, and build ROS workspace:

	$ catkin_make

4. Source workspace environment, set ROS_MASTER_URI (for roscore) and CONFIG
   (for 4d-agriculture-py2) env vars:

	$ source devel/setup.bash
	$ export ROS_MASTER_URI=http://192.168.0.188:11311 

5. Run the 4d-agriculture-py2 ROS node:

	$ rosrun agriculture-4d-py2 main_ros.py -config $CONFIG

### Build and deploy with Docker:

1. Assuming the available machine as Docker installed, first build the image (cd to 4d-agriculture-py2/ folder first):

	$ docker build -t 4dagpy2:local .

2. Next, run the docker image:

	$ docker run --rm --net="host" -it -e ROS_MASTER_URI=http://192.168.0.188:11311 4dagpy2:local

   --rm - removes container after it's shutdown.
   --net - sets network for container to host machine.
    -it - runs the container in an interactive shell.
    -e - sets an environment variable. 







# Below is the original README for 4d-agriculture


## NRI Agsampling - 4D Data Analysis and Visualization

### Project Overview
This application serves to aid the grower in identifying areas of
possible abiotic stress within a field, and allows them to queue
potential points of interest for the autonomous rover to collect soil
and leaf samples for diagnosis. It inputs the 4d point maps generated
by the rover and presents maps and overlays to show the user the status
of the crops in the field.

### Basic User Instructions
When this application opens, the user will see the data from the most
recent year's height data. The user can inspect the data using a variety
of different visualizations, located along the right sidebar. Using this
information, the user will select points of interest where they believe
there are signs of stress in the plants. The user can also press the
*Suggest Points* button in the header to have the program automatically
determine points for the user.

Once the user has selected a number of points to be sampled, they can
then press the *Export Points* button to save the gps locations of those
points as a GEOjson object. If the application is being run as a ROS
node, the gps information will also be automatically published to a ROS
topic to signal to a rover to take soil and leaf samples at those
locations.

##### Visualizations
* Satellite - Displays the satellite imagery of the plot
* Height - The heights of the plants at each point
* Growth - The rate of change of the height each day
* Canopy - The number of data points near this location that have nonzero values
* Stress Map - The likelyhood that the plants in an area are experienceing some form of stress
* Rover Path - The path that the rover will take when sampling points
* Custom Func - A custom defined function used for experimenting
* Normalize Data - Scales all of the data from 0-100 and equally weights all inputs

##### Buttons and Panels
* Configuration - Opens up a new window with various settings and options
* Previous/Next - Changes the date throughout the current year
* Change Year - Opens a dropdown menu to select a different year
* Suggest Points - Uses the stress map to select points for the user
* Clear Points - Removes all selected points from the plot
* Gif Visualization - Opens a new window with a continuous view of the selected viasuals
* Data Visualization - Select several different overlays or combinations of overlays
* Plot Statistics - Shows various statistics about the current overlay(s)
* Export Stress Map - Export the year's stress map as a csv
* Export Points - Save the selected points as a GEOjson object. If ROS is enabled, also published a String with the coordinates

### Future Progress / Ongoing Steps
* Zoom and pan functions for plot
* Efficiency improvements when exporting/publishing points
* Topography Data Visualization
* Window resizing - Changing gui elements to scale
* Using historical data to predict high risk areas in future growing seasons
* Add threading to isolate gui from calculations

### Setup
* Python Version 3.6.5
* Anaconda to manage dependencies

### Project Files
* main_ros.py - Sets up and calls gui inside a ROS node
* main_standalone.py - Sets up and calls gui without ROS implemented
* gui.py - All code for interacting with gui, includes IO, plotting, and managing helper functions
* plotting.py - Helper methods to analyze/manipulate data maps
* export.py - Helper methods for data IO
* config.py - Helper methods to create/verify config.ini files
* viewer.py - Gui class for gif visualizations window
* custom.py - Contains method for custom data manipulation

### Similar Projects
* https://www.hs-osnabrueck.de/fileadmin/HSOS/Homepages/COALA/Veroeffentlichungen/2010-VDI-Phenotyping.pdf
* http://www.mdpi.com/1424-8220/15/3/4823/htm
* http://journals.plos.org/plosone/article?id=10.1371/journal.pone.0152839
* https://www.researchgate.net/publication/6537890_Does_growing_on_a_slope_affect_tree_xylem_structure_and_water_relations_Tree_Physiol
