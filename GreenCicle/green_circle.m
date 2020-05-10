%% Initiate 
clear
rosshutdown

setenv('ROS_MASTER_URI','http://192.168.203.128:11311')
setenv('ROS_IP','84.238.65.133')
rosinit('http://192.168.203.128:11311','NodeHost','84.238.65.133');

% For test on local file
image = imread('C:\Users\Holme\Desktop\6_semester\ROB2\NyeRepo\GreenCicle\green.JPG');

cameraSub = rossubscriber('/camera/rgb/image_raw');

msg = receive(cameraSub);

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
figure
imagesc(img_rot), colorbar

% Check bumper med simulering

%% do the thing - DOES NOT WORK WITH ROS IMAGE FOR SOME REASON
figure
imshow(img_rot)

%m1lab = filterImage(img_rot); % Filtering does not work properly with ROS image
m1lab = img_rot;

m1prop = regionprops(m1lab, 'Eccentricity', 'Centroid'); %TODO: This is not good..

xCoordinate = 0;  % Of circle center
pixelsOnXAxis = 640; % Resolution is 640*480
middleOfXAxis = pixelsOnXAxis/2;

if ~isempty(m1prop)
    for i = 1 : size(m1prop, 1)
        [greenCircleDetected, xCoordinate] = detectGreenCircle(m1prop(i), img_rot);
        if greenCircleDetected
            if xCoordinate < middleOfXAxis
                disp("I need to turn left")
            elseif xCoordinate > middleOfXAxis
                disp("I need to turn right")
            else
                disp("I need to use scan to judge distance from target")
            end
            break;
        end
    end
end

function [greenCircleDetected, xCoordinate] = detectGreenCircle(m1prop, image)
    isCircle = false;
    greenCircleDetected = false;
    if m1prop.Eccentricity < 0.6
       isCircle = true;
       disp("this is a circle")
    end
    center = m1prop.Centroid;
    
    xCoordinate = int32(center(1,1));
    yCoordinate = int32(center(1,2));
    disp(xCoordinate + "," + yCoordinate)
    
    % Check if center of circle is green
    red = image(yCoordinate,xCoordinate,1);
    green = image(yCoordinate,xCoordinate,2);
    blue = image(yCoordinate,xCoordinate,3);
        
    if(green > blue && green > red && isCircle)
        greenCircleDetected = true;
        disp("Green cicle detected")
    else
        disp("Circle was not green")
    end
end

function m1lab = filterImage(image)
    m1g = rgb2gray(image);
    figure
    imagesc(m1g), colorbar
    
    m1b = m1g < 130;
    figure
    imagesc(m1b)
    
    m1bd = imclose(m1b, strel('disk', 10)); % TODO: Test and tune
    figure
    imagesc(m1bd)
    
    m1be = bwpropfilt(m1bd, 'Area', [0 2000000]); % TODO: Test and tune
    figure
    imagesc(m1be)
    
    m1lab = bwlabel(m1be);
end


% Wont be used - use laser scan instead
function markerDistance = getDistance(pixelHeight)  % Input should be height of circle in pixels
    k = 533;
    actualHeight = 200;  % What is actual height?
    markerDistance = (k * circleSize) / pixelHeight;
end
