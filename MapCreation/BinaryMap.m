% Create Occupancy Map 
image = imread('shannon.png');

% Convert to grayscale and black and white image
grayimage = rgb2gray(image);
bwimage = grayimage < 220;

grid = robotics.BinaryOccupancyGrid(bwimage);
map = flipud(grid.getOccupancy());

%% Create Shortest Path

start = [1111; 600]; 
goal = [1100; 550]; 

dx = DXform(map);
dx.plan(goal);
path = dx.query(start, 'animate'); 


goalRadius = 0.5; 
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

robotCurrentPose = [robotCurrentLocation 0];
robot = ExampleHelperDifferentialDriveRobot(robotCurrentPose); 

controller = robotics.PurePursuit; 
controller.Waypoints = path;
controller.DesiredLinearVelocity = 5;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 1;

distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

maxLidarRange = 8; % meters
mapResolution = 20; % cells per meters
slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);

% Tradeoffs - see documentation 
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

while( distanceToGoal > goalRadius )
     % Get scan
    scanMsg = receive(laserSub);
    scan = lidarScan(scanMsg); 
    
    [v, w] = controller(robot.CurrentPose);
    drive(robot, v, w); 
    robotCurrentPose = robot.CurrentPose; 
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal); 
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
  
end
