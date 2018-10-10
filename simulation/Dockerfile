# definition of  Evaluation Container
FROM duckietown/gym-duckietown-server:slim

RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev \
         bzip2 \
         curl \
         && rm -rf /var/lib/apt/lists/*

# RUN sudo apt-get install -y wget
# ADD https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh miniconda.sh
RUN curl -o miniconda.sh https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh

RUN sh miniconda.sh -b -p /opt/conda2 && rm miniconda.sh

RUN mkdir /project

COPY log.py /project
COPY eval.py /project

COPY gym_simulation_launcher.py /project
COPY launch.sh /project

COPY requirements.txt /project
RUN /opt/conda2/bin/pip install -r /project/requirements.txt

WORKDIR /project
CMD /opt/conda2/bin/python eval.py
