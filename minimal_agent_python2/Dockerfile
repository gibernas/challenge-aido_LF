FROM python:2.7

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt
RUN pip list

COPY . .


ENV DISABLE_CONTRACTS=1
ENTRYPOINT ["python", "-u", "minimal_agent_python2.py"]
