
import subprocess
import signal
import time
import os
print("This comes from the script")
server = subprocess.Popen(["python3", "server.py"])
print("This comes from the script too")
time.sleep(20)
print("This comes from the script three")
os.killpg(os.getpgid(server.pid), signal.SIGTERM)
#lsof -i :8080 
