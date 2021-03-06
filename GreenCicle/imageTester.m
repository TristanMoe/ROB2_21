%% Initiate 
clear
rosshutdown

setenv('ROS_MASTER_URI','http://192.168.80.128:11345')
setenv('ROS_IP','192.168.1.107')
rosinit('http://192.168.80.128:11311','NodeHost','192.168.1.107');

% For test on local file
%image = imread('C:\Users\Holme\Desktop\6_semester\ROB2\NyeRepo\GreenCicle\green.JPG');

cameraSub = rossubscriber('/camera/rgb/image_raw');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');

while true
    velMsg.Linear.X = 0.2;
    velMsg.Angular.Z = 0.2;
    velPub.send(velMsg);
    image = readImage(receive(cameraSub));
    imshow(image);
    %m1lab = filter(image); % Filtering does not work properly with ROS image
    m1lab = image;
    
   % Green ball thresholds 
    greenBall.greenMax = 120; 
    greenBall.darkMin = 30; 

    imSize = size(image); 
    heightThresh = imSize(1)*0.5; % Pixel height as a function of image size

    greenImg = 2*image(:,:,2)-image(:,:,3)-image(:,:,1); %Remove all non-green color
    greenThresh = greenImg < greenBall.greenMax; 
    imshow(greenImg) 
    
    grayscale = logical(greenImg);
    imagesc(grayscale)
    m1prop = regionprops(grayscale, 'Eccentricity', 'Centroid'); %TODO: This is not good with turtlebot
    imshow(m1prop)

    
    
    
    
    xCoordinate = 0;  % Of circle center
    pixelsOnXAxis = 640; % Resolution is 640*480
    middleOfXAxis = pixelsOnXAxis/2;

    if ~isempty(m1prop)
        for i = 1 : size(m1prop, 1)
            [greenCircleDetected, xCoordinate] = detectGreenCircle(m1prop, image);
            if greenCircleDetected
                %Break the while loop
                break;
            end
        end
    end
end

if xCoordinate < middleOfXAxis
    disp("I need to turn left")
elseif xCoordinate > middleOfXAxis
    disp("I need to turn right")
else
    disp("I need to use scan to judge distance from target")
    distanceOk = getDistance();
    if distanceOk
        %We are looking at the green circle at an appropriate distance
        %Tristan and Martin takes over
    end
end

%% functions
function image = getImageFromCamera(camSub)
    msg = receive(camSub);

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

function m1lab = filter(image)
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

%function distanceOk = getDistance()
    %Implement laserscan
%end

% Wont be used - use laser scan instead
function markerDistance = getDistance(pixelHeight)  % Input should be height of circle in pixels
    k = 533;
    actualHeight = 200;  % What is actual height?
    markerDistance = (k * circleSize) / pixelHeight;
end