%% Initiate 
rosshutdown

% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
setenv('ROS_MASTER_URI','http://192.168.87.107:11311')
setenv('ROS_IP','192.168.87.106')
rosinit('http://192.168.87.107:11311','NodeHost','192.168.87.106');

%% Subscribe to Laser
laser = rossubscriber('/scan');
% Load scan, timeout 3 seconds
scan = receive(laser,3);
plot(scan)

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

%% Resize grid & data 
mat = scale(cart); 
imshow(mat);
%% Hough Transform & Get lines
lines = getObstacleInformation(mat);

%% Find overall shortest distance for all lines.
for k = 1:length(lines)
    % Find XY_RHO - Create perpendicular triangle and calculate XY for
    % rho-coordinate point.
    theta_rho = 90 - abs(lines(k).theta); 
    dist_rho = lines(k).rho; 
    
    x_rho = (sin(theta_rho)*dist_rho)/sin(90); 
    y_rho = (sin(180-theta_rho-90)*dist_rho)/sin(90);
    
    xy_rho = [x_rho, y_rho];    
    
    % Find distance to P1 
    X = [0,0];
    P1 = [lines(k).point1];
    distP1 = pdist(X, 'euclidean'); 
    
    % Find distance to P2 
    P2 = [lines(k).point2];
    distP2 = pdist(X, 'euclidean');
    
    % Find linear function for P1, P2 ???
    coeff = polyfit([P1], [P2], 1);   
    a = coeff(1);
    b = coeff(2);
    y_pos_rho = a*x_rho+b; 
end



%% Find range between two points
X = [0,0;0.6,1.18];
dist = pdist(X, 'euclidean'); 

%% Testing stuff 
exampleHelperROSLoadMessages
scan;
plot(scan) 

% Read cart
[cart, angles] = readCartesian(scan, 'RangeLimit', [minRange, maxRange]);
[H, T, R] = hough(cart); 
P = houghpeaks(H, 5, 'threshold', ceil(0.3*max(H(:))));

lines = houghlines(cart, T, R, P, 'FillGap', 5, 'MinLength', 7);

for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
    plot(xy(:,1),xy(:,2), 'LineWidth', 2, 'Color', 'green'); 
end

%% Function to extract range-angle coordinate
% Robot range to wall 
% Angle between robot heading & wall 

function [mat] = scale(cart)
    gridSize = 100; 
    x = cart(:,2); % x-position
    d = cart(:,1); % depth

    xoffset = min(x); 
    xscale = (max(x)-min(x)); 

    xi = (x - xoffset)/xscale; % normalize data 0-1;
    xi = round(xi*(gridSize-1))+1; 

    doffset = min(d); 
    dscale = (max(d)-min(d));

    di = (d - doffset)/dscale; 
    di = round(di*(gridSize-1))+1; 
    ind = sub2ind([gridSize gridSize], xi, di); 

    mat = zeros(gridSize); 
    mat(ind) = 1; 
end

% Function to get distance to wall
% Robot angle between wall and x-axis 
function lines = getObstacleInformation(mat)
[H, theta, rho] = hough(mat);
minPeakThreshold = 0.1;
minLineThreshold = 5; 

numpeaks = 10;
hPeaks = houghpeaks(H, numpeaks, 'Threshold', minPeakThreshold); 

% Get lines 
lines = houghlines(mat,theta,rho,hPeaks,'FillGap',5,'MinLength',minLineThreshold);

% Show lines
imshow(mat), hold on,
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
end
end




