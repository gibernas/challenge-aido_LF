FROM duckietown/rpi-gui-tools:master18-more-deps

WORKDIR /videos

COPY requirements.txt  requirements.txt
RUN pip install -r  requirements.txt

COPY . .

ENV PYTHONPATH=/videos

ENTRYPOINT ["/videos/entrypoint.sh"]
CMD  ["/videos/entrypoint.sh", "python" ,"/videos/videos.py"]

