FROM python:2.7

WORKDIR /visualizer

COPY requirements.txt  requirements.txt
RUN pip install -r  requirements.txt

# TODO: put back in requirements.txt
RUN pip install plotly
RUN pip install duckietown-world==1.0.19

COPY . .

ENV PYTHONPATH=/visualizer

ENTRYPOINT ["/visualizer/entrypoint.sh"]
CMD  ["/visualizer/entrypoint.sh", "python", "/visualizer/visualizer.py"]

