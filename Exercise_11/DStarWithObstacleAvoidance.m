function [estimatedPose] = DStarWithObstacleAvoidance(startPixel, stopPixel, pixelToMeter, startTheta)
[checkScanForObstacle, avoidObstacle, setUpVFHController, getPoseVector, runAMCL] = ...
    obstacleFunctions();
    % Substribe to odometer and laser scanner. 
    
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
[pub,msg] = rospublisher('/mobile_base/commands/reset_odometry','std_msgs/Empty');
send(pub,msg);

% Create publisher for sender velocity commands 
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');


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

% Lab is 52.2 m2 and image-lab is 99px x 265 px
% Entire image = 1540px x 701px
% 265 / 99 = 2.67
% 1x * 2.45x = 55.2 m2, x = 4.74m
% cells to meter ratio = 99px / 4.74m = 20.88
gridWidth = 25;
grid = flipud(bwimage);
se = strel('square',gridWidth);
gridAfterDialate = imdilate(grid,se);
imshow(gridAfterDialate);

%%
map = robotics.BinaryOccupancyGrid(bwimage, pixelToMeter); 

start = startPixel; 
goal = stopPixel; 

dx = DXform(gridAfterDialate);
dx.plan(goal);
path = dx.query(start, 'animate'); 
path = path/map.Resolution; 
% Set movement properties
goalRadius = 1; 

distanceToGoal = norm(start/map.Resolution - goalRadius); 

%% localizationSetup

startInMeters = start/map.Resolution;

initialPose = [startInMeters(1) startInMeters(2) startTheta]; 

amcl = setupAMCL(map, initialPose);

%% Setup Helper for Vizualization and Driving Turtlebot
visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Set PurePuresuitController 
controller = robotics.PurePursuit;
controller.Waypoints = path; 
controller.DesiredLinearVelocity = 0.6; 
controller.MaxAngularVelocity = 1.5; 
controller.LookaheadDistance = 1.5;

%% Set VFHController
vfhController = setUpVFHController();
minRangeObstacleDetect = 0;
maxRangeObstacleDetect = 1.2;
angleIntervalThreshold = 0.15;

% Loop - scan, drive.
i=0;
while(distanceToGoal >= goalRadius)
    poseVector = getPoseVector(odomSub, startInMeters);
    scanMsg = receive(laserSub);
    scan = lidarScan(scanMsg);
    [estimatedPose, estimatedCovariance, i] = runAMCL(amcl, scan,poseVector, i, visualizationHelper);
    % Get new values for robot
    
    isObstacle = checkScanForObstacle(scan, minRangeObstacleDetect, maxRangeObstacleDetect, ...
    angleIntervalThreshold);

    if(isObstacle)
        isObstacle
        [steerDir, numberOfIterations] = avoidObstacle(vfhController, ...
            estimatedPose(3), 20, laserSub, robot, velMsg,...
            odomSub, startInMeters, amcl,i, visualizationHelper);
        i = numberOfIterations;
    else
        % Pass new information to turtlebot.
        [linVel, angVel, targetDir] = controller(estimatedPose); 
        velmsg.Linear.X = linVel; 
        velmsg.Angular.Z = angVel;
        send(robot, velmsg);
    end 
            
    distanceToGoal = norm(estimatedPose(1:2) - goal/map.Resolution);  
end
end



function [amcl] = setupAMCL(map, initialPose)

odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% Link: https://se.mathworks.com/help/nav/ref/likelihoodfieldsensormodel.html
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
% Min, Max of sensor readings.
rangeFinderModel.SensorLimits = [0.45 10];
% Occupancy map used for computing likelihood field.
rangeFinderModel.Map = map;

% Transform laser readings from camera frame to base frame of turtlebot. 
% !!! Sensor model is only compatible with sensors fixed on robot's frame
% Query tansformation tree (tf tree) in ROS 
tftree = rostf; 
% Projekteres op til 'verdens koordinatsystem' frem for robottens.
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;

% Assign MotionModel and SensorModel 
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

% Update particles when robot's movement exceeds UpdateThresholds. 
% Using larger numbers leads to slower particle depletion at the price of
% slower particle convergence as well. 
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

% Lower and upper bound on number of partiblecs generated during resample process. 
% Increasing increases change of true pose, but impact computation speed. 
amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
% Initial pose estimate should be obtained according to our setup
% This helper function retrieves thoe robot's current true pose from Gazebo
amcl.InitialPose = initialPose;
amcl.InitialCovariance = eye(3)*0.5;

end 