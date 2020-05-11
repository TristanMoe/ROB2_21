%% Initiate 
clear
rosshutdown

setenv('ROS_MASTER_URI','http://192.168.203.130:11311')
setenv('ROS_IP','84.238.65.133')
rosinit('http://192.168.203.130:11311','NodeHost','84.238.65.133');

% For test on local file
%image2 = imread('C:\Users\Holme\Desktop\6_semester\ROB2\NyeRepo\GreenCicle\green.JPG');

cameraSub = rossubscriber('/camera/rgb/image_raw');
laserSub = rossubscriber('/scan');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');

isFound = findGreenCircle(cameraSub, velMsg, velPub);