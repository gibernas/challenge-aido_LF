# definition of  Evaluation Container
#FROM python:2.7-alpine
FROM duckietown/gym-duckietown
COPY requirements.txt /project

RUN pip install -r /project/requirements.txt

COPY eval.py /project
WORKDIR /project
CMD eval.py
