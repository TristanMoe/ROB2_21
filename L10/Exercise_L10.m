%% Initiate 
rosshutdown

% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
setenv('ROS_MASTER_URI','http://127.0.0.1:11345')
setenv('ROS_IP','192.168.1.107')
rosinit('http://192.168.80.128:11311','NodeHost','192.168.1.107');

% Substribe to odometer and laser scanner. 
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

% Create publisher for sender velocity commands 
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

%% Load & Show map
load officemap.mat
show(map)

%% Setup Lasor Sensor Model & TurtleBot Motion Model
% Motion is estimated using odometry data - noice defines uncertainty in
% robot's rotational and linear motion, can be adjusted
% Link: https://se.mathworks.com/help/nav/ref/odometrymotionmodel.html
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];


% Link: https://se.mathworks.com/help/nav/ref/likelihoodfieldsensormodel.html
rangeFinderModel = likelihoodFieldSensorModel;
% Min, Max of sensor readings.
rangeFinderModel.SensorLimits = [0.45 8];
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

%% Initialize AMCL Object. 
% Link: https://se.mathworks.com/help/nav/ref/montecarlolocalization-system-object.html
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
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3)*0.5;

%% Setup Helper for Vizualization and Driving Turtlebot
visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Turtlebot is driven using the below class, which drives the robot inside 
% the environment while avoiding obstacles using the controllerVFG class
% Link to controllerVFG: https://se.mathworks.com/help/nav/ref/controllervfh-system-object.html
wanderHelper = ...
    ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

%% Localization Procedure 
numUpdates = 60;
i = 0;
while i < numUpdates
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    % !!! Perhaps add more particles to check that it might be other places
    % in the map... Sensor outage, disconnection, moving robot ... just to ensure errors. !!! 
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scan);
    
    % !!! Use estimate in intervals to check if pose / position is correct !!!
    % !!! Use odometer until threshold between actual odometer position and
    % estimate is too large, then change using estimate odometer position !!! 
    % !!! USE ESTIMATE POSE, NOT ODEMETER !!!
    % Drive robot to next pose.
    wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
    end    
end

%% Shutdown sequence
stop(wanderHelper);
rosshutdown

%% Function 
function [isUpdated, estimatedPose, estimatedCovariance] = setupACML(map, initialePose, ...
    rosLaserSubscriber, rosOdometerSubscriber)

odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% Link: https://se.mathworks.com/help/nav/ref/likelihoodfieldsensormodel.html
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
% Min, Max of sensor readings.
rangeFinderModel.SensorLimits = [0.45 8];
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
amcl.InitialPose = initialePose;
amcl.InitialCovariance = eye(3)*0.5;

end 

function [isUpdated, estimatedPose, estimatedCovariance, numberOfIterations] = runACML(scanMsg, pose,numberOfIterations)
scan = lidarScan(scanMsg);
[isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scan);
if isUpdated
    numberOfIterations = numberOfIterations + 1;
    plotStep(visualizationHelper, amcl, estimatedPose, scan, numberOfIterations);
end    
end


