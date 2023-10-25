# ROS Distro
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

# Disable interactive mode
ARG DEBIAN_FRONTEND=noninteractive

# Set working directory
ENV PACKAGE_NAME=tum_lanelet2_osm_fusion

# Install RViz and additional dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Copy tum_lanelet2_osm_fusion project into container
WORKDIR /workspace
COPY . /workspace/src/$PACKAGE_NAME

# Update and install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -r -y

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"


RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. /workspace/install/local_setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
