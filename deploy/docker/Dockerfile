FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y software-properties-common && rm -rf /var/lib/apt/lists/*
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install --no-install-recommends -y gdb python3.9 python3.9-dev python3-pip ros-noetic-desktop && rm -rf /var/lib/apt/lists/*

ENV PYTHONUNBUFFERED 1
ENV PYTHONDONTWRITEBYTECODE 1
COPY library /~/fillow_the_drow_library
RUN python3.9 -m pip install -e /~/fillow_the_drow_library
ENV FOLLOW_THE_DROW_LIBRARY /~/fillow_the_drow_library/cpp_core

ENV ROS_LANG_DISABLE geneus:genlisp:gennodejs
RUN mkdir -p /~/catkin_ws/src
RUN cd /~/catkin_ws/src && source /opt/ros/noetic/setup.bash && catkin_init_workspace
RUN cd /~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make clean && catkin_make install
WORKDIR /~/catkin_ws

COPY deploy/docker/entrypoint.sh ./
ENTRYPOINT ["bash", "entrypoint.sh"]
CMD ["rosversion", "ros"]
