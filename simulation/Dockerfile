FROM duckietown/gym-duckietown-server-ros:v3

RUN mkdir /project

COPY requirements.txt /project
RUN /opt/conda2/bin/pip install -r /project/requirements.txt

WORKDIR /project



COPY log.py /project
COPY eval.py /project

COPY gym_simulation_launcher.py /project
COPY launch.sh /project


CMD /opt/conda2/bin/python eval.py
