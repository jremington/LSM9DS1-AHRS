# LSM9DS1-AHRS

Mahony AHRS and Tilt Compensated Compass for Arduino and the LSM9DS1 sensor, written and tested for the Adafruit LSM9DS1 breakout board, using I2C connection.

Standard orientation for the tilt-compensated compass: marked Y axis points True North.  Correct for local magnetic declination is included in the code. Change this by changing the facing vector p in the code.

Standard sensor orientation for AHRS Tait-Bryan angles is X North (yaw=0) Y West and Z Up (NWU system). Note that X is remapped to form a right handed system. North is OPPOSITE to the direction of the "X arrow" on the Adafruit breakout board.

The Sparkfun LSM9DS1 Arduino library is required. Only the default settings and raw sensor data are used.

The currently implemented AHRS algorithm is the standard Madgwick/Mahony scheme found in other repositories. However, new features have been added, such as code to simplify accurate calibration of the accelerometer, magnetometer and gyro. The magnetometer and accelerometer axes are realigned so that the output orientation is meaningful, and to simplify overall operation. 

SENSOR CALIBRATION

Gyro, magnetometer and accelerometer calibration is required, for each sensor individually. The Arduino program LSM9DS1_cal_data.ino collects gyro data and calculates the gyro offset (while the sensor is held still), then collects about 300 accelerometer and magnetometer data points, while the user slowly and carefully rotates the sensor in 3D.

You need to cut/paste/copy the output data on the serial monitor, then create two separate comma separated value (.csv) file, one each for the magnetometer and accelerometer x,y,z values, and use them for the final calibration steps.

General magnetometer calibration approaches are described in this excellent blog article:

    http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html 
    
I also strongly recommend this blog post as a general guide to magnetometer/accelerometer calibration

   https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

Magneto is a fair sized C program that runs on a PC or a Mac (suggest to use Code::Blocks IDE on Windows). For convenience, I’ve collected all of the basic parts of magneto, described in the sailboatinstruments link, into one file that can be compiled and run on a desktop.

The magneto approach fits an ellipsoid to the data, avoiding statistical problems associated with the min/max approach, rotates the ellipsoid to align with the coordinate axes, scales the axial dimensions to a sphere, and rotates back. The result is a set of offsets and a nine element matrix correction that must be applied to the raw data.

The magneto program was modified to add measurement rejection criteria and to publish data initialization statements that can be incorporated directly into the AHRS code. 

A detailed example of using the above procedures in a particularly difficult case, with severe "hard iron" distortion can be found at 

https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315
