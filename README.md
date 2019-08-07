# RoverGUI
Rover GUI, run in another Raspberry Pi and connected via local area network

To run ROS in another "ROS" enable hardware, do this following script in terminal:

export ROS_MASTER_URI=http://rosHardwareIPAddress:11311
export ROS_IP=yourIPAddress

to check your IP Address:
hostname -I
