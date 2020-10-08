ARG DOCKER_ROS_DISTRO=melodic
FROM ros:${DOCKER_ROS_DISTRO}

# install catkin_tools
RUN apt-get update && apt-get install -y \
    python-catkin-tools\
    && rm -rf /var/lib/apt/lists/*

# Booststrap workspace.
ENV CATKIN_DIR=/catkin_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && mkdir -p $CATKIN_DIR/src \
 && cd $CATKIN_DIR \
 && catkin init \
 && catkin build
WORKDIR $CATKIN_DIR

# We want the development workspace active all the time.
RUN echo "#!/bin/bash\n\
set -e\n\
source \"${CATKIN_DIR}/devel/setup.bash\"\n\
exec \"\$@\"" > /startup.sh \
 && chmod a+x /startup.sh \
 && echo "source ${CATKIN_DIR}/devel/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/startup.sh"]
CMD ["bash"]
