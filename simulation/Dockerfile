FROM duckietown/gym-duckietown-server-ros:v3

RUN mkdir /project

RUN apt install python3-dev -y

COPY requirements.txt /project

RUN /opt/conda/bin/conda install -c conda-forge ruamel.yaml ruamel.ordereddict
RUN /opt/conda/bin/pip install -r /project/requirements.txt

WORKDIR /project



COPY log.py /project
COPY eval.py /project
COPY test1.py /project
COPY environment.sh /project

COPY gym_simulation_launcher.py /project
COPY launch.sh /project

ENV PYTHONPATH=/project

ENTRYPOINT ["/project/environment.sh"]

CMD ["/opt/conda/bin/python", "/project/eval.py"]
