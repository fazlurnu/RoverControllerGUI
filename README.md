# RoverGUI
Rover GUI, run in another Raspberry Pi and connected via local area network

To run ROS in another "ROS" enable hardware, do this following script in terminal (make sure both are connected through local network, or any network):


$   export ROS_MASTER_URI=http://rosHardwareIPAddress:11311

$   export ROS_IP=yourIPAddress

To check your IP Address:
$   hostname -I

$   export ROS_MASTER_URI=http://192.168.43.234:11311 //In this case
--------------------------
$   export ROS_IP=192.168.43.135 //In this case
--------------------------
