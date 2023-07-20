FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y gdb python3-dev python3-pip ros-noetic-desktop && rm -rf /var/lib/apt/lists/*
ENV PYTHONUNBUFFERED 1
ENV PYTHONDONTWRITEBYTECODE 1

COPY library /~/fillow_the_drow_library
RUN pip install /~/fillow_the_drow_library

ENV CMAKE_INSTALL_PREFIX /usr/local
RUN mkdir /~/fillow_the_drow_library/cpp_core/build && cd /~/fillow_the_drow_library/cpp_core/build && cmake .. && make && make install
ENV FollowTheDrow_INCLUDE_DIRS /~/fillow_the_drow_library/cpp_core/include
ENV FollowTheDrow_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib
ENV FollowTheDrow_DIR ${CMAKE_INSTALL_PREFIX}/cmake

ENV ROS_LANG_DISABLE geneus:genlisp:gennodejs
RUN mkdir -p /~/catkin_ws/src
RUN cd /~/catkin_ws/src && source /opt/ros/noetic/setup.bash && catkin_init_workspace
RUN cd /~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make clean && catkin_make install
WORKDIR /~/catkin_ws

COPY deploy/docker/entrypoint.sh ./
ENTRYPOINT ["bash", "entrypoint.sh"]
CMD ["rosversion", "ros"]
