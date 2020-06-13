%% 
% !!! REMEMBER TO CHANGE IP BASED ON HOST !!!
rosshutdown
setenv('ROS_MASTER_URI','http://192.168.124.128:11345')
setenv('ROS_IP','192.168.87.106')
rosinit('http://192.168.124.128:11311','NodeHost','192.168.87.106');

% Subscribe to image % turtlebot
tbot = turtlebot('192.168.124.128');
colorImg = getColorImage(tbot); 

% Remove all non-green color
greenImg = 2*colorImg(:,:,2)-colorImg(:,:,3)-colorImg(:,:,1);
logImg = logical(greenImg);

% Remove small blobs / noise
imgReduced = bwpropfilt(logImg, 'Area', [100 200000]); 

% Remove high eccentricity 
imgEcc = bwpropfilt(imgReduced, 'Eccentricity', 1, 'smallest');
imagesc(imgEcc); 

s = regionprops(logImg, {'Centroid','Area','EquivDiameter'});