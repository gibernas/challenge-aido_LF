FROM duckietown/gym-duckietown-server-python2-ros

RUN mkdir /project

COPY requirements.txt /project
RUN pip install -r /project/requirements.txt

WORKDIR /project


COPY log.py /project
COPY eval.py /project

COPY gym_simulation_launcher.py /project
COPY launch.sh /project


CMD python eval.py
