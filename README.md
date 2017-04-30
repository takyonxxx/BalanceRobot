Balance Robot Arduino Due or Mega

The Balance Robot uses:

18650 battery holder,

Arduino DUE / MEGA,

2 x 12V 37mm 350 Rpm 30:1 DC GearMotor with Encoder

MPU6050 (6-axis motion-tracking device that combines a 3-axis gyroscope and a 3-axis accelerometer)
With MPU6050 , you can get stable angle when the Kalman filter is used.

L298P motor driver:
is a high voltage (50V), high current (2A) dual channel full-bridge driver.
It can drive inductive loads such as relay, DC, and stepping motors.

Serial Bluetooth Module HC-06

Acrylic board

Video:
https://vimeo.com/212365258

Calibrating your PID Controller

Create some way in which you can change the PID constant of your robot while it is running.
One option is to use a potentiometer or some other analogue input to be able to increase or decrease the PID constant. 
I personally used the USB connection and the serial monitor to send new PID values.
This is important as you can then see straightaway how well the new PID values are working, and you won’t have to re-upload the code hundreds of times!
Set all PID constants to zero. This is as good a place to start as any…
Slowly increase the P-constant value.
While you are doing this, hold the robot to make sure it doesn’t fall over and smash into a million pieces!
You should increase the P-constant until the robot responds quickly to any tilting, and then just makes the robot overshoot in the other direction.
Now increase the I-constant.
This component is a bit tricky to get right.
You should keep this relatively low, as it can accumulate errors very quickly.
In theory, the robot should be able to stabilise with only the P and I constants set, but will oscillate a lot and ultimately fall over.
Raise the D-constant.
A lot. The derivative components works against any motion, so it helps to dampen any oscillations and reduce overshooting.

