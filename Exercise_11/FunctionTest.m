rosshutdown
close all

%% 
% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
setenv('ROS_MASTER_URI','http://192.168.80.128:11345')
setenv('ROS_IP','192.168.1.107')
rosinit('http://192.168.80.128:11311','NodeHost','192.168.1.107');

cameraSub = rossubscriber('/camera/rgb/image_raw');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');
odomSub = rossubscriber('/odom');

laserSub = rossubscriber('/scan');
robot = rospublisher('/mobile_base/commands/velocity');

%% A TO B
start = [1105 567]; 
goal = [150 220];

pixelToMeterRatio = 22.5;

estimatedPose = DStarWithObstacleAvoidance(start, goal, pixelToMeterRatio, 0);


findGreenCircle(cameraSub, velMsg, velPub, laserSub);
%%

 tic;
 while toc < 2
 end
 
 tic;
 while toc < 3
    velMsg.Linear.X = 0; 
    velMsg.Angular.Z = 1;
    send(robot, velMsg);
 end
 
 tic;
 while toc < 2
    velMsg.Linear.X = 0.6; 
    velMsg.Angular.Z = 0;
    send(robot, velMsg);
 end
 

 
 odomdata = receive(odomSub, 2); 
 pose = odomdata.Pose.Pose;
 quat = pose.Orientation;
 angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
  
%% B TO C
goal = [670 70];

estimatedPose = DStarWithObstacleAvoidance(...
    round(estimatedPose(1:2)*pixelToMeterRatio), ...
    goal, pixelToMeterRatio, angles(1));
%%
findGreenCircle(cameraSub, velMsg, velPub, laserSub);
