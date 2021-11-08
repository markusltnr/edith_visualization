# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic

RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated curl lsb-release wget ca-certificates

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# setup keys
RUN wget --no-check-certificate -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update
RUN apt-get install --no-install-recommends -y --allow-unauthenticated ros-melodic-desktop-full

# install python2.7
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
      python-rosinstall \
      python-rosinstall-generator \
      python-wstool \
      python-rosdep \
      python-vcstools \
      && rm -rf /var/lib/apt/lists/*

# install python packages
RUN apt-get update
RUN apt-get install python2.7
RUN apt-get install -y python-pip 
RUN pip install --upgrade pip setuptools
RUN pip install open3d
RUN rosdep init && \
    rosdep update

# catkin tools
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
     python-catkin-tools \
     && rm -rf /var/lib/apt/lists/*

# for ros environments
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Prepare catkin build
RUN mkdir -p ~/catkin_ws/src

# install git
RUN apt-get update && apt-get install git 

RUN apt-get install --no-install-recommends -y --allow-unauthenticated mongodb ros-melodic-mongodb-store

RUN git clone   https://gitlab+deploy-token-52:5s2pbPHLsA1ji89pwoKD@rgit.acin.tuwien.ac.at/markus_projects/edith_visualization.git ~/catkin_ws/src/
RUN rm ~/catkin_ws/src/README.md
RUN git clone https://gitlab+deploy-token-50:cqZ5fHtQTWYXFtP2DRE-@rgit.acin.tuwien.ac.at/hsr-tidyup-pipeline/obj_det_ppf_matching_msgs.git ~/catkin_ws/src/obj_det_ppf_matching_msgs
RUN git clone https://markus-docker:R_3nRtD8NL1TET_p8tku@rgit.acin.tuwien.ac.at/leitnermrks/table_extractor.git  ~/catkin_ws/src/table_extractor
RUN git clone https://gitlab+deploy-token-51:U2TwdynH5NrMTUVX1hUp@rgit.acin.tuwien.ac.at/markus_projects/edith_msgs.git ~/catkin_ws/src/edith_msgs

# Run catkin build
RUN /bin/bash -c  '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws/; catkin build'

# source the catkin workspace
#RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

#WORKDIR /root

# setup entrypoint
#COPY ./ros_entrypoint.sh /
#RUN chmod +x /ros_entrypoint.sh
#ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
