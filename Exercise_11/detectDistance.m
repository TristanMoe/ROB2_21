function [closeEnough, distance] = detectDistance(scan)
    robotRadius = 0.3;
    if(~isnan(scan.Ranges(319)))
        distance = scan.Ranges(319);
    else
        closeEnough = false;
        disp("laserscan can't detect anything in the center")
        distance = 0;
    end
    
    if(distance>0.4+robotRadius)
        closeEnough = false;
    elseif(distance<0.4+robotRadius)
        closeEnough = true;
    elseif(distance == 0.4+robotRadius)
        closeEnough = true;
    end
end
   
