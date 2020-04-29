%% Initiate 
rosshutdown

% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!c
setenv('ROS_MASTER_URI','http://127.0.0.1:11345')
setenv('ROS_IP','http://192.168.1.107')
rosinit('http://192.168.80.129:11311','NodeHost','192.168.1.107');


%% Makemap 
%map = openfig('map.fig');
map = makemap(10); %MAKE SURE TO CHECK map.fig if the line above
%does not work and create map with the same look
%% Subscribe to robot position and resets odometer
odom = rossubscriber('/odom');
[pub,msg] = rospublisher('/mobile_base/commands/reset_odometry','std_msgs/Empty');
send(pub,msg);
rostopic list

%% Subscribe to velocity  
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);


%% Get robots start position and plans with DXForm
odomdata = receive(odom, 2); 
pose = odomdata.Pose.Pose;

%% Setting path with either DXForm or with hardcoded path
useHardCodedPath = true; %change to false to use DXForm algorithm
start = [0 0]; 
goal = [30 80]; 
dx = DXform(map);
dx.plan(goal); 
path = dx.query(start, 'animate'); 
% Set movement properties
goalRadius = 0.1; 

if(useHardCodedPath)
    path = [3 3; 4 3; 5 3; 5 4;5 5; 4 5; 3 5; 3 4;3 3;3 2]; %% comment this out if using DXForm and uncomment the line above
    goal = [3 2]; %goal is 3,3 but goal radius is 1, so robot stops in 3,3
    goalRadius = 1;  
end

distanceToGoal = norm(start - goalRadius); 
%% 
% Set controller
controller = robotics.PurePursuit;
controller.Waypoints = path; 
controller.DesiredLinearVelocity = 0.2; 
controller.MaxAngularVelocity = 1; 
controller.LookaheadDistance = 0.2;

% Robot control loop
while(distanceToGoal >= goalRadius)
    % Get turtlebot pose 
    odomdata = receive(odom, 2); 
    pose = odomdata.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    %theta = rad2deg(angles(1));
    poseVector = [pose.Position.X+start(1), pose.Position.Y+start(2), angles(1)];
    
    % Get new values for robot
    [linVel, angVel, targetDir] = controller(poseVector);   
    % Pass new information to turtlebot. 
    velmsg.Linear.X = linVel; 
    velmsg.Angular.Z = angVel;       
    send(robot, velmsg); 
    distanceToGoal = norm(poseVector(1:2) - goal); 
end

%% Path simulering 
goal = [120,80];
start = [45,10]; 
dx = Dstar(map);
%% Simulering med DXForm (can be run independent of above code)
map = makemap(100);
goal = [10,10];
start = [50,50]; 
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













