# Sets env vars for running 4d-agriculture-py2 in a docker container:
export CONFIG="${CONFIG:-/home/docker/ros_workspace/src/4d-agriculture-py2/config-docker.ini}"  # 4d-agriculture-py2's config.ini path
#export ROS_IP="${ROS_IP:-192.168.0.146}"  # default is last IP laptop had on RedRoverWifi
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.0.188:11311}"  # default is roscore on Clifford (red rover's rpi3)
source "${WORKSPACE_ENV:-/home/docker/ros_workspace/devel/setup.bash}"