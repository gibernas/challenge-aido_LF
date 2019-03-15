FROM python:3.7
#FROM python:3.7-slim
#RUN apt-get update && apt-get install -y git
# really need gcc...

WORKDIR /project

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

COPY . .
COPY experiment_manager.py .

ENTRYPOINT ["python3", "experiment_manager.py"]
