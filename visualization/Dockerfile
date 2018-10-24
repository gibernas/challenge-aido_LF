FROM python:2.7

WORKDIR /visualizer

COPY requirements.txt  requirements.txt
RUN pip install -r  requirements.txt

COPY . .

ENV PYTHONPATH=/visualizer

ENTRYPOINT ["/visualizer/entrypoint.sh"]
CMD  ["/visualizer/entrypoint.sh", "python" ,"/visualizer/videos.py"]

