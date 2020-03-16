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

%% Extract cartesian from scan-plot
minRange = 0.1; 
maxRange = 7; 

range = scan.Ranges; 
anglesRange = readScanAngles(scan); 

[cart, angles] = readCartesian(scan, 'RangeLimit', [minRange, maxRange]);

figure(); 
plot(cart(:,2), cart(:,1), '.'); 

figure(); 
plot(scan);

%% Resize grid & data 
mat = scale(cart); 
imshow(mat);
%% Hough Transform & Peaks
[distanceToWall, robAngle, minimumLine] = getObstacleInformation(cart);

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
function [distanceToWall, robAngle, minimumLine] = getObstacleInformation(mat)
[H, theta, rho] = hough(mat);
minPeakThreshold = 0.1;
minLineThreshold = 15; 

numpeaks = 100;
hPeaks = houghpeaks(H, numpeaks, 'Threshold', minPeakThreshold); 

% Get lines 
lines = houghlines(mat,theta,rho,hPeaks,'FillGap',5,'MinLength',minLineThreshold);

% Show lines
imshow(mat), hold on,
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
end

% Find shortest distance to all line obstacles. 
[distanceToWall, index] = min([lines.rho]); 

% Distance to pendicular point on wall (Hough-line).
% Angle will always be 90 degrees on Hough-line.
minimumLine = lines(index);

% Angle between wall and x-axis. 
robAngle = minimumLine.theta; 
end







