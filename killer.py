import os
import time

os.makedirs("/tmp/killer")

while True:
    for file in os.listdir("/tmp/killer"):
        try:
            if time.mktime(time.strptime("%Y-%m-%d_%H:%M:%S.pid")) < time.time():
                pid = None
                with open("/tmp/killer/" + file) as f:
                    pid = int(f.read())
                os.remove("/tmp/killer/" + file)
                os.kill(pid)
