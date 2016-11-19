# Snake Vision Robot testing branch

NOTE: ALL MOTOR CONTROL has been removed from this branch, this sketch is used to just test the sensors, and see the logic in the serial monitor 
It was also used to add the sensor calibrate function, and make sure having two of the MLX90614 sensors would not cause problems.

The idea is that the robot is attracted to heat, and will move toward it.
The setup is to have the MLX90614 sensors set at an angle from each other, 
IF one sees more heat than the other, the robot will turn in that direction, and will attempt to move to the source.

The MLX90614 sensors were provided by IC Station for this experiment.
http://www.icstation.com/mlx90614esf-human-body-infrared-temperature-sensor-contact-temperature-module-p-9911.html

The sensors are I2C, and a MUX will be used to switch between the two sensors

## Parts

Heat Seeking robot based on 2 IC Station MLX90614ESF Human Body Infrared Temperature Sensors
Using a L9110 Motor Driver, and 2 CD74HC4067 MUX boards. 
With an Arduino Nano for it's brains.


## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request

## Credits

LeRoy Miller (2016).

## License

This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses>
