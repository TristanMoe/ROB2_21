%% Initiate 
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
setenv('ROS_IP','192.168.1.100')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.50');

%% Makemap 
%map = openfig('map.fig');
map = makemap();

%% Subscribe to robot position 
odom = rossubscriber('/odom');

%% Subscribe to velocity  
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);

%% Get robots start position
odomdata = receive(odom, 2); 
pose = odomdata.Pose.Pose;
start = [round(pose.Position.X)+1, round(pose.Position.Y)+1]; 

goal = [round(pose.Position.X)+10,round(pose.Position.Y)+10];
dx = DXform(map);
dx.plan(goal); 
path = dx.query(start, 'animate'); 
%path = path(2:2:end,:); 

%%
% Set controller
controller = robotics.PurePursuit;
controller.Waypoints = path; 
controller.DesiredLinearVelocity = 0.2; 
controller.MaxAngularVelocity = 2; 
controller.LookaheadDistance = 0.2;

% Set movement properties
goalRadius = 0.5; 
distanceToGoal = norm(start - goalRadius); 

% Robot control loop
while(distanceToGoal > goalRadius)
    % Get turtlebot pose 
    odomdata = receive(odom, 2); 
    pose = odomdata.Pose.Pose;
    theta = pose.Orientation.W;
    poseVector = [round(pose.Position.X+1) round(pose.Position.Y+1) 0]; 
    
    % Get new values for robot
    [linVel, angVel, targetDir] = controller(poseVector);   
    
    % Pass new information to turtlebot. 
    velmsg.Linear.X = linVel; 
    velmsg.Angular.Z = angVel;       
    send(robot, velmsg);        
end

%% Path simulering 
goal = [120,50];
start = [45,10]; 
dx = DXform(map);
dx.plan(goal); 
path = dx.query(start);

%% Turtlebot simulering
goalRadius = 0.5; 
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

robotCurrentPose = [robotCurrentLocation 0];
robot = ExampleHelperDifferentialDriveRobot(robotCurrentPose); 

controller = robotics.PurePursuit; 
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;


distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

while( distanceToGoal > goalRadius )
    [v, w] = controller(robot.CurrentPose);
    drive(robot, v, w); 
    robotCurrentPose = robot.CurrentPose; 
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal); 
end













