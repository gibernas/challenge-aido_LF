

FROM python:3.7

WORKDIR /project

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt
RUN pip install PyGeometry
COPY . .
COPY experiment_manager.py .

ENTRYPOINT ["python3", "experiment_manager.py"]
