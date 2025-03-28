FROM osrf/ros:humble-desktop

ARG GZ_VERSION=fortress

WORKDIR /nav_ws

SHELL ["/bin/bash", "-c"]

ENV GZ_VERSION=${GZ_VERSION}

RUN apt-get update \
    && apt-get install -y lsb-release wget gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y "gz-${GZ_VERSION}"  \
    && rm -rf /var/lib/apt/lists/*


# Install the matching ros_gz version, eg. `ros-humble-ros-gzharmonic`.
# For Fortress, a suffix is not necessary, so it would just be `ros-humble-ros-gz`
RUN apt-get update \
    && apt-get install -y "ros-${ROS_DISTRO}-ros-gz${GZ_VERSION#fortress}" emacs htop byobu python3 pip less ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    curl \
    software-properties-common

COPY nav_ws/ /nav_ws/

RUN apt-get update && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

ENV TURTLEBOT3_MODEL=waffle

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
RUN echo "source /nav_ws/install/setup.bash" >> /root/.bashrc

# Copy the entrypoint script and make it executable
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]