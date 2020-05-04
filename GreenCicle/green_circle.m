%% Gray scale object detection example
clear

image = imread('green.JPG');
imshow(image)

m1lab = filterImage(image); % TODO: Fix filtering
imshow(m1lab)

m1prop = regionprops(m1lab, 'Eccentricity', 'Centroid'); 

xCoordinate = 0;  % Of circle center
pixelsOnXAxis = 640; % Resolution is 640*480
middleOfXAxis = pixelsOnXAxis/2;

if ~isempty(m1prop)
    for i = 1 : size(m1prop, 1)
        [greenCircleDetected, xCoordinate] = detectGreenCircle(m1prop(i), image);
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
    if m1prop.Eccentricity < 0.5
       isCircle = true;
    end
    center = m1prop.Centroid;
    disp(center)
    
    xCoordinate = int32(center(1,1));
    yCoordinate = int32(center(1,2));
    
    red = image(yCoordinate,xCoordinate,1);
    green = image(yCoordinate,xCoordinate,2);
    blue = image(yCoordinate,xCoordinate,3);
        
    if(green > blue && green > red || isCircle)
        greenCircleDetected = true;
        disp('Green cicle detected with center: ')
        disp(xCoordinate + "," + yCoordinate)
    end
end

function m1lab = filterImage(image) %TODO: This needs to be fixed
    m1g = rgb2gray(image);
    m1b = m1g < 130; % Image Segmenter APP
    m1bd = imclose(m1b, strel('disk', 20)); % fill holes in disc
    m1be = bwpropfilt(m1bd, 'Area', [1000 200000]); % remove small blobs..
    m1lab = bwlabel(m1be);
    m1prop = regionprops(m1lab, 'Area', 'Eccentricity') % find features..
    m1bf = bwpropfilt(m1be, 'Eccentricity', 1, 'smallest');
    m1lab = m1b;
end


% Wont be used - use laser scan instead
function markerDistance = getDistance(pixelHeight)  % Input should be height of circle in pixels
    k = 533;
    actualHeight = 200;  % What is actual height?
    markerDistance = (k * circleSize) / pixelHeight;
end
