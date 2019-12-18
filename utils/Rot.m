function R = Rot(theta,axis)
axis = abs(axis);
if isnumeric(theta)
    if axis(1) == 1 && axis(2) == 0 && axis(3) == 0
        R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
    elseif axis(1) == 0 && axis(2) == 1 && axis(3) == 0
        R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    elseif axis(1) == 0 && axis(2) == 0 && axis(3) == 1
        R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    else
    %     should return error
        R  = [];
    end
else
    if axis(1) == 1 && axis(2) == 0 && axis(3) == 0
        R = [sym([1 0 0]); 
             sym(0) cos(theta) -sin(theta); 
             sym(0) sin(theta) cos(theta)];
    elseif axis(1) == 0 && axis(2) == 1 && axis(3) == 0
        R = [cos(theta) sym(0) sin(theta); 
             sym([0 1 0]); 
             -sin(theta) sym(0) cos(theta)];
    elseif axis(1) == 0 && axis(2) == 0 && axis(3) == 1
        R = [cos(theta) -sin(theta) sym(0); 
             sin(theta) cos(theta) sym(0); 
             sym([0 0 1])];
    else
    %     should return error
        R  = [];
    end
end