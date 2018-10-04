#FROM python:2.7-alpine
FROM ubuntu:18.04

ARG CHALLENGE_EVALUATION=/challenge-evaluator

RUN apt-get update
RUN apt-get install -y git

RUN apt-get install -y python-pip

RUN echo CHALLENGE_EVALUATION=${CHALLENGE_EVALUATION}
RUN test -n "${CHALLENGE_EVALUATION}"
ENV CHALLENGE_EVALUATION ${CHALLENGE_EVALUATION}
 
COPY . ${CHALLENGE_EVALUATION}

RUN pip install -r ${CHALLENGE_EVALUATION}/requirements.txt

RUN chmod +x ${CHALLENGE_EVALUATION}/eval.py
CMD ${CHALLENGE_EVALUATION}/eval.py
