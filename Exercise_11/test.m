%% Initiate 
clear
rosshutdown

setenv('ROS_MASTER_URI','http://192.168.80.128:11345')
setenv('ROS_IP','192.168.1.107')
rosinit('http://192.168.80.128:11311','NodeHost','192.168.1.107');

% For test on local file
%image2 = imread('C:\Users\Holme\Desktop\6_semester\ROB2\NyeRepo\GreenCicle\green.JPG');

cameraSub = rossubscriber('/camera/rgb/image_raw');
laserSub = rossubscriber('/scan');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');

odomSub = rossubscriber('/odom');

robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);

isFound = findGreenCircle(cameraSub, velMsg, velPub, laserSub);

 odomdata = receive(odomSub, 2); 
 pose = odomdata.Pose.Pose;
 quat = pose.Orientation;
 angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

 tic;
 while toc < 3
    velmsg.Linear.X = 0; 
    velmsg.Angular.Z = 1;
    send(robot, velmsg);
 end
 
 tic;
 while toc < 2
    velmsg.Linear.X = 0.6; 
    velmsg.Angular.Z = 0;
    send(robot, velmsg);
 end

