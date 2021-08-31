# Line-Follower-Bot-
The line follower robot is a mobile machine that can detect and follow the line drawn on the floor. 
Generally, the path is predefined and is visible like a black line on a white surface with a high contrasted color.

The bot uses the OpenCV module of python for image processing, which detects the balck path and also calculates the deviation from the path (i.e., the error)
The PID controller which has been implemented on the arduino (microcontroller) through Arduino IDE (uses a Lang similar to C/C++) uses this error to calculate the motor speeds,
thus making a "turn right", "turn left" or "go straight" decision. The PID tuning is the greatest challenge.
P.S: PID tuning differs system to system.
