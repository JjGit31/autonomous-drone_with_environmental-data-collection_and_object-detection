import subprocess
import time

# Iterate focus values from 0 to 255 in steps of 10
for i in range(0, 256, 10):
    cmd = f"v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute={i}"
    subprocess.run(cmd, shell=True)
    print(f"Focus set to {i}")
    time.sleep(2)  # Wait to observe the effect

