FROM python:3.7

WORKDIR /project

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

COPY . .

ENTRYPOINT ["python3", "scenario_maker.py"]
