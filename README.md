Do not mistake me for an engineer
==

Awful quadrotor code

Stabilization code cannot be finished - sensors establish absolute direction and the yaw axis drifts too much for this to trust its measurements to correct for this. Robocopter MKII will include a magnetometer (instead of a barometer lol I'm an idiot).  
But! This /can/ get off the ground in spite of this! It just crashes really hard and really fast. If you can't win, redefine success!

The flight controller is an arduino mega 2560r3 with a BMP085 barometer, a ITG3200 gyroscope and an ADXL345 accelerometer. LCD's purpose changes depending on what problem I'm having.

Sparkfun links:
- https://www.sparkfun.com/products/10121
- https://www.sparkfun.com/products/10121
- https://www.sparkfun.com/products/9054

'Quotient of two vectors' means little to me so I swiped the quaternion/orientation code from each sensor's respective buildr tutorial:
- http://bildr.org/2011/06/bmp085-arduino/
- http://bildr.org/2012/03/stable-orientation-digital-imu-6dof-arduino/

Radio receiver is a Spektrum AR600 with a DX6i transmitter.

It's made out of aluminum I cut, drilled, hammered, bent and abused in my basement.
