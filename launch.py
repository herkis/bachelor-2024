import subprocess

subprocess.run('ros2 launch launch/sensor.launch.py',shell=True)
subprecess.run('ros2 run sensor_oxygen oxygen_sensor talker',shell=True)