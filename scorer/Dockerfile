FROM ros:kinetic-ros-base-xenial

RUN apt-get update && apt-get install python-pip -y

COPY requirements.txt /project/requirements.txt
RUN pip install -r /project/requirements.txt

COPY eval.py /project/eval.py
COPY read_scores.py /project/read_scores.py
COPY log.pickle /example.pickle

WORKDIR /project

ENV PYTHONPATH=/project

CMD python /project/eval.py
