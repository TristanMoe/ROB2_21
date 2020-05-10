function [checkScanForObstacle, avoidObstacle, setUpVFHController, getPoseVector, runAMCL] = obstacleFunctions()
    setUpVFHController = @setUpVFHController;
    checkScanForObstacle = @checkScanForObstacle;
    avoidObstacle = @avoidObstacle;
    getPoseVector = @getPoseVector;
    runAMCL = @runAMCL;
end

function [isObstacle] = checkScanForObstacle(lidarScan, minRange, maxRange, angleInterval)
    scanLimited = removeInvalidData(lidarScan, 'RangeLimits', [minRange, maxRange], ...
    'AngleLimits', [-angleInterval, angleInterval]); 
    figure(10);
    plot(scanLimited);
    hold on 
    plot(lidarScan);
    hold off
    if (scanLimited.Count == 0) 
        isObstacle = 0;
    else 
        isObstacle = 1;
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
vfh.DistanceLimits = [0.05 1.5];

% Radius of actual robot! 
vfh.RobotRadius = 0.3;

vfh.MinTurningRadius = 0.1;

% Meters around the robot which should be avoided! 
vfh.SafetyDistance = 0.5;
end 

% Calculate obstacle free steering direction. 
function [steerDir, numberOfIterations] = avoidObstacle(controllerVFH, targetDir,...
length, laserSub, velPub, velMsg, odomSub, startInMeters, amcl, ...
numberOfIterations, visualizationHelper) 
    % https://www.mathworks.com/help//nav/ref/ratecontrol.html
    % Loop frequency. 
    rate = rateControl(10);

    while rate.TotalElapsedTime < length
        % Updating pose for acml
        poseVector = getPoseVector(odomSub, startInMeters);
        % Get laser scan data
        laserScan = receive(laserSub);
        ranges = double(laserScan.Ranges);
        angles = double(laserScan.readScanAngles);        
 
        % Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);   
        
        [estimatedPose, estimatedCovariance,numberOfIterations] = runAMCL(amcl, scan, poseVector,numberOfIterations, visualizationHelper);
        
        steerDir = controllerVFH(scan, targetDir-estimatedPose(3)); 
        figure(4);
        show(controllerVFH);
    
        % Calculate velocities
        if ~isnan(steerDir) % If steering direction is valid
            desiredV = 0.2;
            w = exampleHelperComputeAngularVelocity(steerDir, 0.5);
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

function [poseVector] = getPoseVector(odomSub, startInMeters)
    % Get turtlebot pose 
    odomdata = receive(odomSub, 2); 
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    %theta = rad2deg(angles(1));
    poseVector = [pose.Position.X+startInMeters(1), pose.Position.Y+startInMeters(2), angles(1)];
end

function [estimatedPose, estimatedCovariance, numberOfIterations] = ...
runAMCL(amcl, lidarScan, pose,numberOfIterations, visualizationHelper)

[isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, lidarScan);
if isUpdated
    numberOfIterations = numberOfIterations + 1;
    plotStep(visualizationHelper, amcl, estimatedPose, lidarScan, numberOfIterations);
end    
end