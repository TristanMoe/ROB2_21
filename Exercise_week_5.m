%% Initiate 
roslaunch turtlebot_bringup 3dsensor.launch

setenv('ROS_MASTER_URI','http://192.168.1.200:11311')
% assuming your own ip is 192.168.1.100
setenv('ROS_IP','192.168.1.50')
rosinit('http://192.168.1.200:11311','NodeHost','192.168.1.50');

%% Makemap 
map = makemap();

%% Subscribe to robot position 
odom = rossubscriber('/odom');

%% Subscribe to velocity  
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);

%% Set goal and find path 
goal = [120,80];
start = [45,10]; 
dx = DXform(map);
dx.plan(goal); 
path = dx.query(start); 

% Set controller
controller = robotics.PurePursuit;
controller.Waypoints = path; 
controller.DesiredLinearVelocity = 0.3; 
controller.MaxAngularVelocity = 0.6; 
controller.LookaheadDistance = 0.5;

% Set movement properties
goalRadius = 0.1; 
distanceToGoal = norm(start - goalRadius); 

% Robot control loop
while( distanceToGoal > goalRadius )
    % Get turtlebot pose 
    odomdata = receive(odom, 2); 
    pose = odomdata.Pose.Pose;
    theta = odomdata.Pose.Orientation; 
    poseVector = [pose.Position.X, pose.Position.Y, pose.Orientation]; 
    
    % Get new values for robot
    [linVel, angVel, targetDir] = controller(poseVector);   
    
    % Pass new information to turtlebot. 
    velsmg.Linear.X = linVel; 
    velsmg.Angular.Z = angVel;       
    send(robot, velsmg);    
end



%% Show map
dx = DXform(map) 
dx.plot(map) 

%% Set goal and find path 
goal = [120,80];
start = [45,10]; 
dx.plan(goal); 
dx.query(start, 'animate'); 

%% D-star
ds = Dstar(map);
tic, ds.plan(goal); toc
ds.plot();
ds.query(start, 'animate');

%%







