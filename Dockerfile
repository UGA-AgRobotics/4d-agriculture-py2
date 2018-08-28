# Dockerfile for testing 4d-agriculture
# Using a ROS image with Python 2.7.12 baked in:
FROM ros:kinetic

# Creates "docker" user, installs python-tk, sudo, then gives "docker" sudo rights (todo: test without sudo rights?)
RUN useradd --create-home --shell /bin/bash docker && \
	apt-get update && \
	apt-get install -y python-tk && \
	apt-get install -y sudo && \
	apt-get install -y unzip && \
	usermod -aG sudo docker

# Creates "docker" user's password
RUN echo 'docker:testpass' | chpasswd

# Makes ROS workspace and folder for 4d-agriculture-py2
RUN mkdir -p /home/docker/ros_workspace/src/4d-agriculture-py2

# Copies contents from this directory to the container's
COPY . /home/docker/ros_workspace/src/4d-agriculture-py2

# Sets "docker" as owner of /home/docker, gives "docker" r-x rights on script file
RUN chown -R docker:docker /home/docker && \
	chmod 754 /home/docker/ros_workspace/src/4d-agriculture-py2/docker_run_4d-agriculture-py2.sh

# NOTE: pip version < 10 can uninstall disutils packages, which is
# needed for scikit-fuzzy requirement (requires pyyaml==3.13, disutil's pyyaml is 3.11)
# Link: https://stackoverflow.com/questions/49911550/how-to-upgrade-disutils-package-pyyaml

# Installs pip==8.1.0, installs 4d-agriculture-py2's requirements
RUN easy_install pip==8.1 && \
	pip install -r /home/docker/ros_workspace/src/4d-agriculture-py2/requirements.txt

# Sets work directory to 4d-agriculture-py2 ROS package
WORKDIR /home/docker/ros_workspace/src/4d-agriculture-py2/

# Sets $DISPLAY to show GUI on host machine
ENV DISPLAY=:0

# Sets default user to "docker"
USER docker

# Runs ROS build and deploy script when docker image is run
CMD ["./docker_run_4d-agriculture-py2.sh"]