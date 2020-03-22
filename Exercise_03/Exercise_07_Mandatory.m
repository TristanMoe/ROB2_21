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
% plot(scan);


%% Extract cartesian from scan-plot
minRange = 0.1; 
maxRange = 10; 

range = scan.Ranges; 
anglesRange = readScanAngles(scan); 

[cart, angles] = readCartesian(scan, 'RangeLimit', [minRange, maxRange]);
figure(); 
plot(cart(:,2), cart(:,1), '.'); 
axis equal 

% figure(); 
% plot(scan);

% scale to image and scale back to get coordinates of lines then find rho
% and theta from these points

lines = getObstacleInformation(cart);
hold on
% taken from houghline docs
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
   
   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
hold off

%% Hough Transform & get lines based on peaks. 
% Find overall shortest distance for all lines.
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

[distShortest, index] = min([distLineShortest(:).dist])
angleShortest = distLineShortest(index).angle

%%
% Function to get distance to wall
% Robot angle between wall and x-axis 
function lines = getObstacleInformation(cart)

[mat, xscale, xoffset, dscale, doffset] = scale(cart);
[H, theta, rho] = hough(mat);
minPeakThreshold = 2;
minLineThreshold = 15; 

numpeaks = 15;
hPeaks = houghpeaks(H, numpeaks, 'Threshold', minPeakThreshold); 

% Get lines 
lineScaled = houghlines(mat,theta,rho,hPeaks,'FillGap',5,'MinLength',minLineThreshold);
lines = reScaleCoor(lineScaled,xscale, xoffset, dscale, doffset);
end

%%from class demo
function [mat, xscale, xoffset, dscale, doffset] = scale(cart)
x = cart(:,2);
d = cart(:,1);
N = 100; % grid size

xoffset = min(x);
xscale = (max(x)-min(x));
xi = (x - xoffset)/xscale; % scale to 0-1
xi = round(xi*(N-1))+1; % scale and integer 1 to N
doffset = min(d);
dscale = (max(d)-min(d));
di = (d - doffset)/dscale; % scale to 0-1
di = round(di*(N-1))+1;
ind = sub2ind([N N], xi, di); % matrix indices

mat = zeros(N); % NxN matrix
mat(ind) = 1; % with points..
%imshow(mat), colorbar()  
end

function lines = reScaleCoor(lines,xscale, xoffset, dscale, doffset)
N = 100; % grid size
for k = 1:length(lines)
    line = lines(k);
    line.point1 = flip(line.point1, 2);
    line.point2 = flip(line.point2, 2);
    line.point1(1) = ((line.point1(1)-1)/(N-1))*xscale+xoffset;
    line.point1(2) = ((line.point1(2)-1)/(N-1))*dscale+doffset;
    
    line.point2(1) = ((line.point2(1)-1)/(N-1))*xscale+xoffset;
    line.point2(2) = ((line.point2(2)-1)/(N-1))*dscale+doffset;
    
    a = (line.point2(2)-line.point1(2))/(line.point2(1)-line.point1(1));
    
    %% taken from https://se.mathworks.com/matlabcentral/answers/70287-to-find-intersection-point-of-two-lines
    %line1
    x1  = [line.point1(1) line.point2(1)];
    y1  = [line.point1(2) line.point2(2)];
    %line2
    x2 = [-10 0 10];
    y2 = [10/a 0 -10/a];
    %fit linear polynomial
    p1 = polyfit(x1,y1,1);
    p2 = polyfit(x2,y2,1);
    if(line.point1(1) == line.point2(1) )
        line.rho = line.point2(1);
        line.theta = 0;
    elseif(line.point1(2) == line.point2(2) )
        line.rho = line.point2(2);
        line.theta = 90;
    else
        x_intersect = fzero(@(x) polyval(p1-p2,x),3);
        y_intersect = polyval(p1,x_intersect);
        line.rho = hypot(x_intersect,y_intersect);
        line.theta = atan2d(x_intersect, y_intersect);
    end
    lines(k) = line;
end
  
end




