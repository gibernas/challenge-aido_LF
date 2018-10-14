FROM duckietown/gym-duckietown-server-python2-ros:more-exposure

RUN mkdir /project

COPY requirements.txt /project
RUN pip install -r /project/requirements.txt

WORKDIR /project


COPY . .


CMD python eval.py
