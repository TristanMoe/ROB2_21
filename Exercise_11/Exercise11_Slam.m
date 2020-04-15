%% Initiate 
rosshutdown

% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
setenv('ROS_MASTER_URI','http://127.0.0.1:11345')
setenv('ROS_IP','192.168.87.106')
rosinit('http://192.168.124.128:11311','NodeHost','192.168.87.106');

% Substribe to odometer and laser scanner. 
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

% Create publisher for sender velocity commands 
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

% Receive laser scan and odometry message.
scanMsg = receive(laserSub);
odompose = odomSub.LatestMessage;
scan = lidarScan(scanMsg); 

% Subscribe to velocity  
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);


%% Load map 
load('officemap.mat');
show(map);

start = [0 0]; 
goal = [5 0]; 

dx = DXform(map);
dx.plan(goal);
path = dx.query(start, 'animate'); 

% Set movement properties
goalRadius = 1; 

distanceToGoal = norm(start - goalRadius); 

%% Set Controller 
Set controller
controller = robotics.PurePursuit;
controller.Waypoints = path; 
controller.DesiredLinearVelocity = 0.2; 
controller.MaxAngularVelocity = 1; 
controller.LookaheadDistance = 0.2;

%% Example 
maxLidarRange = 8; % meters
mapResolution = 20; % cells per meters
slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);

% Tradeoffs - see documentation 
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

% Loop - scan, drive.
for i=1:10
    % Get scan
    scanMsg = receive(laserSub);
    scan = lidarScan(scanMsg); 
    
    % Drive a bit 
    velmsg.Linear.X = 0.3; 
    velmsg.Angular.Z = 0.2;  
    send(robot, velmsg); 
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end    
end

% create new map for path algoritm. 
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');



