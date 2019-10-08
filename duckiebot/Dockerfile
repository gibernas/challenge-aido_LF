# We start from a base ROS image
FROM duckietown/dt-core:daffy-amd64

RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python-tk \
         python-wheel \
         python-pip && \
     rm -rf /var/lib/apt/lists/*


WORKDIR /project
COPY . .
RUN pip install -r requirements.txt
RUN pip list

# For ROS Agent - Need to upgrade Pillow for Old ROS stack
RUN pip install pillow --user --upgrade

RUN /bin/bash -c "export PYTHONPATH="/usr/local/lib/python2.7/dist-packages:$PYTHONPATH""


CMD ["python", "-u", "duckiebot_bridge.py"]
