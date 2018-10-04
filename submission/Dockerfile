# Definition of Submission container

FROM ubuntu:18.04

COPY solution.py /project

COPY trained_models /project/trained_models
COPY aido_model.py /project

WORKDIR /project
CMD solution.py
