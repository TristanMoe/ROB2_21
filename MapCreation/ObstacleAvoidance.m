%% Initiate 
rosshutdown

% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
setenv('ROS_MASTER_URI','http://127.0.0.1:11345')
setenv('ROS_IP','192.168.87.106')
rosinit('http://192.168.124.128:11311','NodeHost','192.168.87.106');

% Substribe to odometer and laser scanner. 
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

% Subscribe to bumper 
bumpSub = rossubscriber('/mobile_base/sensors/bumper_pointcloud', 'BufferSize', 5);

% Create publisher for sender velocity commands 
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity');

%% Set up controller for VFH
vfh = setUpVFHController(); 
% Avoid obstacle and optain current steering direction
targetDir = 0; 
currentSteerDir = avoidObstacle(vfh, targetDir, 5, laserSub, velPub, velMsg); 
rosshutdown

% If anything is registrered in range interval, function returns 1 else 0. 
% Activate VFH if there is obstacle within 1 meter. 
function [isObstacle] = checkScanForObstacle(lidarScan, minRange, maxRange, minAngle, maxAngle)
    scanLimited = removeInvalidData(lidarScan, 'RangeLimits', [minRange, maxRange], 'AngleLimits', [minAngle, maxAngle]); 
    if (scanLimited.Count == 0) 
        isObstacle = 1;
    else 
        isObstacle = 0;
    end
end 

% Setup the controller with adjusted parameters. 
function [vfh] = setUpVFHController()
% https://www.mathworks.com/help//ros/ug/obstacle-avoidance-with-turtlebot-and-vfh.html
vfh = controllerVFH;
vfh.UseLidarScan = true;
% Set this weight to find most optimal path. 

% Limit for range readings. Used to ignore obstacles that are too far from
% the vehicle. 
vfh.DistanceLimits = [0.5 1];

% Radius of actual robot! 
vfh.RobotRadius = 0.3;

vfh.MinTurningRadius = 0.1;

% Meters around the robot which should be avoided! 
vfh.SafetyDistance = 0.1;
end 

% Calculate obstacle free steering direction. 
function [steerDir] = avoidObstacle(controllerVFH, targetDir, length, laserSub, velPub, velMsg) 
    % https://www.mathworks.com/help//nav/ref/ratecontrol.html
    % Loop frequency. 
    rate = rateControl(10);

    while rate.TotalElapsedTime < length
        
        % Get laser scan data
        laserScan = receive(laserSub);
        ranges = double(laserScan.Ranges);
        angles = double(laserScan.readScanAngles);        
 
        % Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);     
        
        steerDir = controllerVFH(scan, targetDir);           
    
        % Calculate velocities
        if ~isnan(steerDir) % If steering direction is valid
            desiredV = 0.2;
            w = exampleHelperComputeAngularVelocity(steerDir, 1);
        else % Stop and search for valid direction
            desiredV = 0.0;
            w = 0.5;
        end

        % Assign and send velocity commands
        velMsg.Linear.X = desiredV;
        velMsg.Angular.Z = w;
        velPub.send(velMsg);
    end   
end 


