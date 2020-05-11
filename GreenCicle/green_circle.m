%% Initiate 
clear
rosshutdown

setenv('ROS_MASTER_URI','http://192.168.203.130:11311')
setenv('ROS_IP','84.238.65.133')
rosinit('http://192.168.203.130:11311','NodeHost','84.238.65.133');

% For test on local file
image2 = imread('C:\Users\Holme\Desktop\6_semester\ROB2\NyeRepo\GreenCicle\green.JPG');

cameraSub = rossubscriber('/camera/rgb/image_raw');
laserSub = rossubscriber('/scan');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');
laserScan = receive(laserSub);

driveAround = true;
lookingAtGreenCircle = false;
while driveAround
    velMsg.Linear.X = 0;
    velMsg.Angular.Z = 0.5;
    velPub.send(velMsg);
    image = readImage(receive(cameraSub));
    imageProps = getImageProps(image);  % Needs filtering

    if ~isempty(imageProps)
        for i = 1 : size(imageProps, 1)
            [greenCircleDetected, xCoordinate] = detectGreenCircle(imageProps);
            if greenCircleDetected
                if xCoordinate > 100 && xCoordinate < 400
                    disp("Green circle is detected")
                    driveAround = false;
                    break;
                end
            end
        end
    end
end

if not(driveAround)
    disp("I need to use scan to judge distance from target")
    distanceOk = false;
    %distanceOk = getDistance();
    if distanceOk
        %We are looking at the green circle at an appropriate distance
        %Tristan and Martin takes over
    end
end

%% functions
function imgProps = getImageProps(image)
    greenImg = 2*image(:,:,2)-image(:,:,3)-image(:,:,1); %Remove all non-green color
    logImg = logical(greenImg);

    % Remove small blobs / noise
    imgReduced = bwpropfilt(logImg, 'Area', [300 10000]); 
%     % Remove high eccentricity 
    imgEcc = bwpropfilt(imgReduced, 'Eccentricity', 1, 'smallest');
    
    imagesc(imgEcc)
 
    imgProps = regionprops(imgEcc, 'Eccentricity', 'Centroid', 'Area', 'Circularity');
end



function image = getImageFromCamera(camSub)
    msg = readImage(receive(camSub))

    %Get msg details
    imgData = msg.Data;
    imgH = msg.Height;
    imgW = msg.Width;

    %Create blank image output with required size
    img = zeros(msg.Width,msg.Height,3);
    img_rot = imrotate(img, 90);

    %Reshape image data
    imgR = reshape(imgData(1:3:end),imgW,imgH)';
    imgG = reshape(imgData(2:3:end),imgW,imgH)';
    imgB = reshape(imgData(3:3:end),imgW,imgH)';
    img_rot(:,:,1) = imgR;
    img_rot(:,:,2) = imgG;
    img_rot(:,:,3) = imgB;
    
    image = img_rot;
end

function [greenCircleDetected, xCoordinate] = detectGreenCircle(m1prop)

    greenCircleDetected = false;
    if m1prop.Eccentricity(1) < 0.6 && m1prop.Circularity > 0.8
       greenCircleDetected = true;
    end
    center = m1prop.Centroid;
    
    xCoordinate = int32(center(1,1));
    yCoordinate = int32(center(1,2));
    disp(xCoordinate + "," + yCoordinate)
end

function m1lab = filter(image)
    m1g = rgb2gray(image);
    figure
    imagesc(m1g), colorbar
    
    m1b = m1g < 100;
    figure
    imagesc(m1b)
    
    m1bd = imclose(m1b, strel('disk', 5)); % TODO: Test and tune
    figure
    imagesc(m1bd)
    
    m1be = bwpropfilt(m1bd, 'Area', [0 2000000]); % TODO: Test and tune
    figure
    imagesc(m1be)
    
    m1lab = bwlabel(m1be);
end

function distanceOk = getDistance()
    %Implement laserscan
end

% Wont be used - use laser scan instead
%function markerDistance = getDistance(pixelHeight)  % Input should be height of circle in pixels
%    k = 533;
%    actualHeight = 200;  % What is actual height?
%    markerDistance = (k * circleSize) / pixelHeight;
%end
