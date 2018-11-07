# UKF-Robot
Unscented Kalman Algorithm for localizing a differential drive robot

# Using KalmanFilter robot.py:
  In the gb.run function (line 501), the algorithm is run. It takes different control parameters, that can modify the experiment:
  withSmoothing: If true, the simple sanity checking filter is implemented, constraining the estimates to the boundary space.
  initialKnown: If true, then the unscented kalman filter is initialized with perfect knowledge of the initial state of the robot
  withAnimation: If true, the sequence of inputs will play out in an animation, before plotting the evalutaion graphs.

  The generateInputs() function can be modified to specify the input path for the robot. Be warned, the robot currently (11/7/18) does not support boundary checking, and a given input sequence might run the robot off screen.
  
  The noise parameters are contained in the Robot class, and can be modified to adjust the accuracy of the sensors and servos.
