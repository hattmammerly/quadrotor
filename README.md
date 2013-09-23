Awful quadrotor code

The flight controller is an arduino mega 2560r3 with a BMP085 barometer, a ITG3200 gyroscope and an ADXL345 accelerometer. LCD's purpose changes depending on what problem I'm having.

Sparkfun links:
- https://www.sparkfun.com/products/10121
- https://www.sparkfun.com/products/10121
- https://www.sparkfun.com/products/9054

The barometer code and 6DOF IMU code were taken from their respective bildr tutorials:
- http://bildr.org/2011/06/bmp085-arduino/
- http://bildr.org/2012/03/stable-orientation-digital-imu-6dof-arduino/

When I can do quaternion math I'll replace them with my own code.

Radio receiver is a Spektrum AR600 with a DX6i transmitter.

Frame is hand-cut aluminum; roughly nine-inch aluminum arms, with 10x4.7 propellers.

If I remember more details, I'll write them in. I'm at college so this code is unfinished for a little while.

I apologize for this droning readme (pardon the pun). When I get back into this project I'll improve it.

##### TODO
- make sure the pitch and roll axes aren't mixed up. Currently, the stabilization code makes it flop upside-down.

Apart from the above list, it should be done!
