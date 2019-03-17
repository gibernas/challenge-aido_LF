FROM duckietown/aido2-base-python3:v4

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt

COPY . .

ENTRYPOINT ["python3", "minimal_agent.py"]
