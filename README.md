# LSM9DS1-AHRS

Mahony AHRS and Tilt Compensated Compass for the LSM9DS1 sensor, written and tested for the Adafruit LSM9DS1 breakout board, using I2C connection.
Because of the way the Adafruit board is constructed and marked, it is convenient to take the marked Y axis as pointing True North.  Correct for local magnetic declination is included in the code.

Note: The standard sensor orientation for AHRS Tait-Bryan angles is X North Y West and Z Up (NWU system)

The Sparkfun LSM9DS1 Arduino library is required. Only the default settings and raw sensor data are used.

The currently implemented AHRS algorithm is the standard Madgwick/Mahony scheme found in other repositories. However, new features have been added, such as code to simplify accurate calibration of the accelerometer, magnetometer and gyro. The magnetometer and accelerometer axes are realigned so that the output orientation is meaningful, and to simplify overall operation. 

SENSOR CALIBRATION
Magnetometer and accelerometer calibration is required. Approaches are described in this excellent blog article:

    http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html 
    
I also strongly recommend this blog post as a general guide to magnetometer/accelerometer calibration

   https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

Magneto is a fair sized C program that runs on a PC or a Mac (suggest to use Code::Blocks IDE on Windows). For convenience, I’ve collected all of the basic parts of magneto, described in the sailboatinstruments link, into one file that can be compiled and run on a desktop.

The magneto approach fits an ellipsoid to the data, avoiding statistical problems associated with the min/max approach, rotates the ellipsoid to align with the coordinate axes, scales the axial dimensions to a sphere, and rotates back. The result is a set of offsets and a nine element matrix correction that must be applied to the raw data.

The magneto program was modified to add measurement rejection criteria and to publish data initialization statements that can be incorporated directly into the AHRS code. 

A detailed example of using the above procedures in a particularly difficult case, with severe "hard iron" distortion can be found at 

https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315
