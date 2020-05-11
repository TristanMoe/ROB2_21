function [xCoordinate] = findGreenCircle(cameraSub, velMsg, velPub, laserSub)
    driveAround = true;
    while driveAround
        velMsg.Linear.X = 0;
        velMsg.Angular.Z = 0.5;
        
        scan = receive(laserSub);
        image = readImage(receive(cameraSub));
        imageProps = getImageProps(image);
        
        if ~isempty(imageProps)
            for i = 1 : size(imageProps, 1)
                [greenCircleDetected, xCoordinate] = detectGreenCircle(imageProps);
                [closeEnough, distance] = detectDistance(scan);
                if (greenCircleDetected || distance < 1)
                        disp("Green circle is detected")
                            if(xCoordinate < 320)
                                velMsg.Angular.Z = 0.2;
                             elseif (xCoordinate > 320)
                                velMsg.Angular.Z = -0.2;
                            end
                            velMsg.Linear.X = 0.1;
                            
                            if(closeEnough)
                                return;
                            end
                            break;
                end
            end
        end        
        velPub.send(velMsg);        
    end
end

function imgProps = getImageProps(image)
    greenImg = 2*image(:,:,2)-image(:,:,3)-image(:,:,1); %Remove all non-green color
    logImg = logical(greenImg);

    % Remove small blobs / noise
    imgReduced = bwpropfilt(logImg, 'Area', [300 35000]); 
%     % Remove high eccentricity 
    imgEcc = bwpropfilt(imgReduced, 'Eccentricity', 1, 'smallest');
    
    imagesc(imgEcc)
 
    imgProps = regionprops(imgEcc, 'Eccentricity', 'Centroid', 'Area', 'Circularity', 'BoundingBox');
end

function [greenCircleDetected, xCoordinate] = detectGreenCircle(m1prop)

    greenCircleDetected = false;
    if m1prop.Eccentricity(1) < 0.8 && m1prop.Circularity > 0.8
       greenCircleDetected = true;
    end
    center = m1prop.Centroid;
    
    xCoordinate = int32(center(1,1));
    yCoordinate = int32(center(1,2));
    disp(xCoordinate + "," + yCoordinate)
end
