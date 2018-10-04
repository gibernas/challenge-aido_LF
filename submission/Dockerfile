# Definition of Submission container

FROM nvidia/cuda:9.1-runtime-ubuntu16.04

RUN apt-get update -y && apt-get install -y --no-install-recommends \
         git \
         bzip2 && \
     rm -rf /var/lib/apt/lists/*

ADD https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh miniconda.sh
RUN sh miniconda.sh -b -p /opt/conda && rm miniconda.sh
ENV PATH /opt/conda/bin:$PATH


RUN mkdir /project

COPY solution.py /project
COPY requirements.txt /project
COPY trained_models /project/trained_models
COPY model.py /project

RUN pip install -r /project/requirements.txt

WORKDIR /project

CMD python solution.py
