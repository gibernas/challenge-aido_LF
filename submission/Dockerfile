# Definition of Submission container

FROM tensorflow/tensorflow:latest-gpu

# from https://github.com/tensorflow/tensorflow/issues/10776
RUN cd /usr/local/cuda/lib64 \
    && mv stubs/libcuda.so ./ \
    && ln -s libcuda.so libcuda.so.1 \
    && ldconfig

RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python-tk && \
     rm -rf /var/lib/apt/lists/*

# ADD https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh miniconda.sh
# RUN sh miniconda.sh -b -p /opt/conda && rm miniconda.sh
# ENV PATH /opt/conda/bin:$PATH

RUN mkdir /project

COPY requirements.txt /project
RUN pip install -r /project/requirements.txt

COPY solution.py /project
COPY trained_models /project/trained_models
COPY model.py /project

WORKDIR /project

CMD python solution.py
