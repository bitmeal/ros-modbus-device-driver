ARG DOCKER_ROS_DISTRO=melodic
FROM ros:${DOCKER_ROS_DISTRO}

# Booststrap workspace.
ENV CATKIN_DIR=/catkin_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
 && mkdir -p $CATKIN_DIR/src \
 && cd $CATKIN_DIR \
 && catkin_make
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
