%% Initiate 
rosshutdown
%% 
% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
setenv('ROS_MASTER_URI','http://192.168.80.128:11345')
setenv('ROS_IP','192.168.1.149')
rosinit('http://192.168.80.128:11311','NodeHost','192.168.1.149');
%% 
% Substribe to odometer and laser scanner. 
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
[pub,msg] = rospublisher('/mobile_base/commands/reset_odometry','std_msgs/Empty');
send(pub,msg);

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


%% Create Occupancy Map from Shannon
image = imread('shannon.png');
imshow(image)

%% Convert to grayscale and black and white image
grayimage = rgb2gray(image);
bwimage = grayimage < 220;

%% 

entreM2 = 9.9;
pixelWidth = 57;
pixelLength = 139;
res = 139/57;
WidthInMeter = sqrt(9.9/res);
pixelToMeterRatio = pixelWidth/WidthInMeter;
%%

% Lab is 52.2 m2 and image-lab is 99px x 265 px
% Entire image = 1540px x 701px
% 265 / 99 = 2.67
% 1x * 2.45x = 55.2 m2, x = 4.74m
% cells to meter ratio = 99px / 4.74m = 20.88
gridWidth = 10;
map = robotics.BinaryOccupancyGrid(bwimage, 20.88);
grid = flipud(getOccupancy(map));
se = strel('square',gridWidth);
gridAfterDialate = imdilate(grid,se);
imshow(gridAfterDialate);



%grid = flipud(getOccupancy(map));
%se = strel('square',gridWidth);
%gridAfterDialate = imdilate(grid,se);

%%


start = [1119 581]; 
goal = [100 300]; 

dx = DXform(gridAfterDialate);
dx.plan(goal);
path = dx.query(start, 'animate'); 
path = path/map.Resolution; 
% Set movement properties
goalRadius = 1; 

distanceToGoal = norm(start/map.Resolution - goalRadius); 

%% Set Controller 
controller = robotics.PurePursuit;
controller.Waypoints = path; 
controller.DesiredLinearVelocity = 0.2; 
controller.MaxAngularVelocity = 1; 
controller.LookaheadDistance = 0.2;


% Loop - scan, drive.
while(distanceToGoal >= goalRadius)
    % Get turtlebot pose 
    odomdata = receive(odomSub, 2); 
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    %theta = rad2deg(angles(1));
    poseVector = [pose.Position.X+start(1)/map.Resolution, pose.Position.Y+start(2)/map.Resolution, angles(1)];
    
    % Get new values for robot
    [linVel, angVel, targetDir] = controller(poseVector);   
    % Pass new information to turtlebot. 
    velmsg.Linear.X = linVel; 
    velmsg.Angular.Z = angVel;       
    send(robot, velmsg); 
    distanceToGoal = norm(poseVector(1:2) - goal/map.Resolution);  
end




