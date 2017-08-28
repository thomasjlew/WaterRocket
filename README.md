# WaterRocket
## Description
Simple parachute deployment for a water rocket.
Was tested on the 29 August and used during the Swiss Water Rockets Championship 2017.
## Authors
Assembly and design: Paul Megevand <br />
Electronics and Programming: Thomas Lew
## How to use it?
Compile the project on your computer with the Arduino standard IDE: <br />
https://www.arduino.cc/en/Main/Software <br />
and upload it to your microcontroller from this program.

For the Arduino pro mini such as the one used here, use an USB adapter (such as the FT232RL )
## Hardware
For the brain of the project, an arduino pro mini was used. Any board would work, just don't forget to set the pins right in the code.

To connect it to a computer for programming it, we used a FT232RL. <br />
![alt text](https://github.com/thomasjlew/WaterRocket/blob/master/images/ft232rl.jpg)

Finally, you need an Inertial Measurement Unit to detect takeoff. We used the MPU6050. <br />
![alt text](https://github.com/thomasjlew/WaterRocket/blob/master/images/mpu6050.jpeg)

## Sample water rocket with electronics mounted
![alt text](https://github.com/thomasjlew/WaterRocket)

## Deployment
Right now, it works with a timer set to 5 seconds. Once the water rocket takes off, it is detected which starts the timer to realease the parachute after 5 seconds.

