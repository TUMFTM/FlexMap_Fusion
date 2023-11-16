# ROS Distro
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

# Disable interactive mode
ARG DEBIAN_FRONTEND=noninteractive

# Set working directory
ENV PACKAGE_NAME=flexmap_fusion

# Install RViz and additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-tk \
    vim \
    ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Copy tum_lanelet2_osm_fusion project into container
WORKDIR /workspace
COPY . /workspace/src/$PACKAGE_NAME

# Install python packages
# Install python packages
RUN pip3 install --upgrade pip && pip3 install --no-cache-dir -r /workspace/src/$PACKAGE_NAME/docker/requirements.txt \
  && rm -rf ~/.cache/pip/*

# Update and install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -r -y

# Make parameter configuration scripts executable
RUN chmod +x /workspace/src/$PACKAGE_NAME/config/config_param.py
RUN chmod +x /workspace/src/$PACKAGE_NAME/config/get_param.py
RUN chmod +x /workspace/src/$PACKAGE_NAME/analysis/plot_traj_matching.py
RUN chmod +x /workspace/src/$PACKAGE_NAME/analysis/plot_matching.py

# Source ROS and build package
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Set entrypoint by sourcing overlay workspace
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. /workspace/install/local_setup.bash\nexport PATH=$PATH:/workspace/src/$PACKAGE_NAME/config\nexport PATH=$PATH:/workspace/src/$PACKAGE_NAME/analysis\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
