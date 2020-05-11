function [lookingAtGreenCircle] = findGreenCircle(cameraSub, velMsg, velPub)
    driveAround = true;
    lookingAtGreenCircle = false;
    while driveAround
        velMsg.Linear.X = 0;
        velMsg.Angular.Z = 0.5;
        velPub.send(velMsg);
        image = readImage(receive(cameraSub));
        imageProps = getImageProps(image);

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
    lookingAtGreenCircle = true;
end

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
