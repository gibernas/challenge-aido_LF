FROM duckietown/gym-duckietown-server-python3:aido2_lf_r1


WORKDIR /project


COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

RUN apt-get update && apt-get install -y libgtk2.0-dev
# RUN conda install -c conda-forge opencv

COPY . .

ENTRYPOINT ["/bin/bash", "/project/launch.sh"]
