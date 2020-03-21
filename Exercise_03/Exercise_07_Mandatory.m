% %% Initiate 
% rosshutdown
% 
% % !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
% setenv('ROS_MASTER_URI','http://192.168.87.107:11311')
% setenv('ROS_IP','192.168.87.106')
% rosinit('http://192.168.87.107:11311','NodeHost','192.168.87.106');
% 
% %% Subscribe to Laser
% laser = rossubscriber('/scan');
% % Load scan, timeout 3 seconds
% scan = receive(laser,3);
% plot(scan)

%% Simulation
exampleHelperROSLoadMessages
scan;
plot(scan);

%% Extract cartesian from scan-plot
minRange = 0.1; 
maxRange = 10; 

range = scan.Ranges; 
anglesRange = readScanAngles(scan); 

[cart, angles] = readCartesian(scan, 'RangeLimit', [minRange, maxRange]);

figure(); 
plot(cart(:,2), cart(:,1), '.'); 
axis equal 

figure(); 
plot(scan);

%% Hough Transform & get lines based on peaks. 
% Find overall shortest distance for all lines.
lines = getObstacleInformation(cart);

for k = 1:length(lines)
    line = lines(k); 
    % Find XY_RHO - Create perpendicular triangle and calculate XY for
    % rho-coordinate point.
    theta_rho = 90 - abs(lines(k).theta); 
    dist_rho = lines(k).rho; 
    
    x_rho = (sind(theta_rho)*dist_rho) / sind(90); 
    y_rho = (sind(180-theta_rho-90)*dist_rho)/sind(90);
    
    xy_rho = [x_rho, y_rho];    
    
    % Check if distance is correct 
    X = [0,0];
    distLine = pdist([X;xy_rho], 'euclidean'); 
    
    % Find distance to P1 
    X = [0,0];
    P1 = [lines(k).point1];
   
    distP1 = pdist([X;P1], 'euclidean'); 
    
    % Find distance to P2 
    X = [0, 0];
    P2 = [lines(k).point2];
    distP2 = pdist([X;P2], 'euclidean');
    
    % Check if x_rho, y_rho is with P1 & P2
    % Find largest x-value
    P_points = [P1(1) P2(1)];
    [x_lar, maxIndex] = max([P_points(1) P_points(2)]); 
    [x_min, minIndex] = min([P_points(1) P_points(2)]); 

    if(and(P_points(minIndex) > x_rho,P_points(minIndex) < x_rho))
        distLineShortest(k).dist = dist_rho; 
        distLineShortest(k).angle = lines(k).theta;  
    else
        X = [0,0];
        P_dist = [distP1; distP2]; 
        P_minPoints = [P1; P2];
        
        [minDist, indexMin] = min(P_dist); 
        distLineShortest(k).dist = minDist;    
        
        P_min = P_minPoints(indexMin,:);         
        distLineShortest(k).angle = atan2d(P_min(1), P_min(2)); 
    end
end

[distShortest, index] = min([distLineShortest(:).dist]);
angleShortest = distLineShortest(index).angle;

%%
% Function to get distance to wall
% Robot angle between wall and x-axis 
function lines = getObstacleInformation(cart)
[H, theta, rho] = hough(cart);
minPeakThreshold = 0.1;
minLineThreshold = 5; 

numpeaks = 15;
hPeaks = houghpeaks(H, numpeaks, 'Threshold', minPeakThreshold); 

% Get lines 
lines = houghlines(cart,theta,rho,hPeaks,'FillGap',5,'MinLength',minLineThreshold);

% Show lines
%imshow(cart), hold on,
%for k = 1:length(lines)
%   xy = [lines(k).point1; lines(k).point2];
%   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%end
end

%% TEST: Find range between two points
%X = [0,0];
%Y = [0.6237, 1.185];
%XY = [X;Y];
%angle = atan2d(Y(1), Y(2)); 
%dist = pdist(XY, 'euclidean'); 


