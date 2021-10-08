# Navigation for Blue ROV2

Sensor-drivers and connection to ROS2 architecture [ROS2 Galactic](https://docs.ros.org/en/ros2_documentation/galactic/index.html).


## Sensor suite

Sensors used in this project:

* Barometer: Blue Robotics BAR30
* Thermometer: Blue Robotics Celsius Fast-Response

## Barometer

Heavily based on the MS5837 library mentioned under [Libraries](#libraries).
Remember to enable I2C on the Raspberry Pi and connect on the correct pins!

## Thermometer

Heavily based on the TSYS01 library mentioned under [Libraries](#libraries).
Remember to enable I2C on the Raspberry Pi and connect on the correct pins!

## Libraries

* [BlueRobotics MS5837 Python Library](https://github.com/bluerobotics/ms5837-python)
* [BlueRobotics TSYS01 Python Library](https://github.com/bluerobotics/tsys01-python)


## License
[MIT](https://choosealicense.com/licenses/mit/)
