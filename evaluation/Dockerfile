# definition of  Evaluation Container
FROM duckietown/gym-duckietown-server
RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev

ADD https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh miniconda.sh
RUN sh miniconda.sh -b -p /opt/conda2 && rm miniconda.sh
RUN mkdir /project
COPY requirements.txt /project
COPY eval.py /project
RUN /opt/conda2/bin/pip install -r /project/requirements.txt
WORKDIR /project
CMD /opt/conda2/bin/python eval.py
