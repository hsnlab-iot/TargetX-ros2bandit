from multiprocessing import Process, set_start_method, Manager
from shared_memory import get_shared_resources
from broker import broker_main
import os
import sys
import subprocess
import io

PIPE_WEBAPI = 'webapi.pipe'
PIPE_BROKER = 'broker.pipe'

def ensure_fifo(pipe_name):
    try:
        os.mkfifo(pipe_name)
    except FileExistsError:
        pass  # already exists

class Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, msg):
        for s in self.streams:
            try:
                s.write(msg)
                s.flush()
            except Exception:
                pass  # ignore broken pipe or closed stream

    def flush(self):
        for s in self.streams:
            try:
                s.flush()
            except Exception:
                pass

def run_flask_app(shared, lock):
    from webapi import create_app
    with open(PIPE_WEBAPI, 'wb', buffering=0) as raw:
        f = io.TextIOWrapper(raw, write_through=True)
        sys.stdout = f
        sys.stderr = Tee(f, sys.__stderr__)
        app = create_app(shared, lock)
        app.run(host='0.0.0.0', port=5000)
        port = app.config.get("SERVER_PORT", "unknown")
        print(f"The application is running on port {port}")

def run_broker(shared, lock):
    with open(PIPE_BROKER, 'wb', buffering=0) as raw:
        f = io.TextIOWrapper(raw, write_through=True)        
        sys.stdout = f
        sys.stderr = Tee(f, sys.__stderr__)
        broker_main(shared, lock)

if __name__ == "__main__":
    set_start_method('spawn')
    shared, lock  = get_shared_resources()

    ensure_fifo(PIPE_WEBAPI)
    ensure_fifo(PIPE_BROKER)

    # Start broker
    broker_process = Process(target=run_broker, args=(shared, lock))
    broker_process.start()

    # Start Flask web API
    flask_process = Process(target=run_flask_app, args=(shared, lock))
    flask_process.start()

    subprocess.run(["tmux", "new-session", "-d", "-s", "broker", f"stdbuf -oL cat {PIPE_BROKER}; sleep inf" ])
    subprocess.run(["tmux", "new-session", "-d", "-s", "webapi", f"stdbuf -oL cat {PIPE_WEBAPI}; sleep inf" ])

    broker_process.join()
    print("Broker process has finished.")
    flask_process.join()
    print("Flask process has finished.")
