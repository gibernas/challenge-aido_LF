# definition of  Evaluation Container
# removed slim
FROM duckietown/gym-duckietown-server

# This might not be strictly necessary
# switch on systemd init system in container
ENV INITSYSTEM off
# setup environment
ENV TERM "xterm"
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO kinetic
ENV QT_X11_NO_MITSHM 1


RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev \
         bzip2

ADD https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh miniconda.sh
RUN sh miniconda.sh -b -p /opt/conda2 && rm miniconda.sh

RUN mkdir /project

COPY log.py /project
COPY eval.py /project

COPY gym_simulation_launcher.py /project
COPY launch.sh /project

COPY requirements.txt /project
RUN /opt/conda2/bin/pip install -r /project/requirements.txt

WORKDIR /project


# ROS INSTALL -------------------------
# install packages
RUN apt-get update && apt-get install -q -y \
        dirmngr \
        gnupg2 \
        apt-utils \
        apt-file \
        lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116


RUN apt-get update
RUN apt-get install -y ros-kinetic-ros-base

RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

RUN apt-get install -q -y python-catkin-pkg
RUN apt-get install -q -y ros-kinetic-catkin

RUN apt-get install ros-kinetic-catkin
RUN apt-get install ros-kinetic-common-msgs
RUN apt-get install -y ros-kinetic-roslib
RUN apt-get install -y ros-kinetic-rosbag
RUN pip install rospkg
ENV PYTHONPATH="/opt/ros/kinetic/lib/python2.7/dist-packages/:${PYTHONPATH}"
# ------------------------------------------

CMD /opt/conda2/bin/python eval.py