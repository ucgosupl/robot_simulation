% Calculate velocity direction based on position and angle
% alfa - current robot orientation
% wheel_now - wheel coordinates x,y from this iteration
% wheel_last - wheel coordinates x,y from last iteration
% dir - 1 - forward -1 - backward
function dir = direction(alfa, wheel_now, wheel_last)
    if alfa < 0
        alfa = alfa + 360;
    end
    
    angle = mod(alfa, 360);
    
    if (angle >= 45) && (angle < 135)
        if wheel_now(2) >= wheel_last(2)
            dir = 1;
        else
            dir = -1;
        end
    elseif (angle >= 135) && (angle < 225)
        if wheel_now(1) <= wheel_last(1)
            dir = 1;
        else
            dir = -1;
        end
    elseif  (angle >= 225) && (angle < 315)
        if wheel_now(2) <= wheel_last(2)
            dir = 1;
        else
            dir = -1;
        end
    else
        if wheel_now(1) >= wheel_last(1)
            dir = 1;
        else
            dir = -1;
        end
    end
end