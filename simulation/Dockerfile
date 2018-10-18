FROM duckietown/gym-duckietown-server-python2-ros:aido1_lf1_r3-v3

RUN mkdir /project

COPY requirements.txt /project
RUN pip install -r /project/requirements.txt

WORKDIR /project


COPY . .


CMD python eval.py
