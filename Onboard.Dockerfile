FROM ros:humble

# Install Debian packages
RUN apt update \
    && apt install -y --no-install-recommends \
        sudo \
        wget \
        gedit \
        nano \
        vim \
        curl \
        unzip \
        net-tools \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir --upgrade pip "setuptools<70" "packaging>=21.3" wheel
RUN pip3 install attrdict
#Had to change to allow for node to work, as some stuff doesnt support numpy 2
RUN pip3 install "numpy<2.0"
RUN pip3 install pillow
RUN pip3 install "opencv-contrib-python<4.11"
RUN pip3 install eventlet==0.33.3
RUN pip3 install Flask==1.1.1
RUN pip3 install Flask-SocketIO==4.1.0
RUN pip3 install python-socketio==4.2.0
RUN pip3 install python-engineio==3.13.0
RUN pip3 install greenlet==1.1.0
RUN pip3 install gevent==21.12.0
RUN pip3 install gevent-websocket==0.10.1
RUN pip3 install Jinja2==3.0.3
RUN pip3 install itsdangerous==2.0.1
RUN pip3 install werkzeug==2.0.3
RUN pip3 install transforms3d

# Install ROS 2 dependencies
RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-nav2-lifecycle-manager


RUN /bin/bash -c 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc' \
    && /bin/bash -c 'echo "source /home/ros_ws/install/setup.bash" >> ~/.bashrc' \
    && /bin/bash -c 'source ~/.bashrc'



COPY ros_ws/. /home/ros_ws

# Resolve whatever the workspace's package.xml files actually declare, instead
# of relying only on the hand-written apt list above.
#
# That list had drifted and the image stopped building: io_context declares
# `asio` and `udp_msgs`, neither of which was installed, so it failed and took
# serial_driver -> vesc_driver down with it, then vesc_msgs / rf2o / planner_node
# aborted alongside. rosdep maps those to libasio-dev and ros-$ROS_DISTRO-udp-msgs
# (plus joy, sick_scan_xd, python3-serial, python3-smbus, example-interfaces)
# straight from the manifests, so adding a package to ros_ws no longer means
# remembering to edit this file too.
#
# This runs AFTER the COPY because it needs those manifests. The apt list above
# is still worth keeping: navigation2, nav2-bringup, nav2-lifecycle-manager,
# imu-tools and tf-transformations are not declared by any package here.
RUN apt update \
    && rosdep update --rosdistro $ROS_DISTRO \
    && cd /home/ros_ws \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && cd /home/ros_ws && colcon build


# Set work directory and expose port
WORKDIR /home/ros_ws
EXPOSE 4567

# Set entrypoint
COPY onboard_setup.sh /home/onboard_setup.sh
RUN chmod +x /home/onboard_setup.sh

ENTRYPOINT ["/home/onboard_setup.sh"]

CMD ["ros2","launch","neo_telemetry","neo_telemetry_sim_launch.py"]