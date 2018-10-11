#FROM ros:kinetic-ros-base-xenial
FROM duckietown/rpi-gui-tools:master18-more-deps

ENV CACHE=2
COPY requirements.txt /project/requirements.txt
RUN pip install -r /project/requirements.txt

COPY eval.py /project/eval.py
COPY read_scores.py /project/read_scores.py
COPY make_video.py /project/make_video.py
COPY example.bag /project/example.bag
COPY entrypoint.sh /project/entrypoint.sh

WORKDIR /project

ENV PYTHONPATH=/project

ENTRYPOINT ["/project/entrypoint.sh"]
CMD  ["/project/entrypoint.sh", "python" ,"/project/eval.py"]

