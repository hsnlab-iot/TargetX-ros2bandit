# Flask app Dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

RUN apt update && apt install -y --no-install-recommends \
    graphviz ccze tmux busybox  && \
    busybox --install -s

COPY app/* /app

EXPOSE 5000
EXPOSE 1884

ENV PYTHONUNBUFFERED=1
CMD ["python", "main.py"]
