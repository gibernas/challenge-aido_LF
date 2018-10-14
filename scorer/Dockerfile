#FROM ros:kinetic-ros-base-xenial
FROM duckietown/rpi-gui-tools:master18-more-deps

ENV CACHE=2
WORKDIR /scorer

COPY requirements.txt  requirements.txt
RUN pip install -r  requirements.txt

COPY . .


ENV PYTHONPATH=/scorer

ENTRYPOINT ["/scorer/entrypoint.sh"]
CMD  ["/scorer/entrypoint.sh", "python" ,"/scorer/scorer.py"]

