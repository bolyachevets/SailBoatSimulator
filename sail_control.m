% sail control function from "Modeling, control and state-estimation for an
% autonomous sailboat", Jon M
function  sail_angle = sail_control(input)
    delta_sail_min = pi/32;
    delta_sail_max = pi/5.2;
    
    % true wind characteristics from input
    a_true = input(1); % speed
    psi_true = input(2); % direction in n-frame
    velocity = input(3);
    heading = input(4);
    
    wind_apparent = [a_true*cos(psi_true - velocity) - heading; a_true*sin(psi_true - velocity)];
    psi_apparent = atan2(wind_apparent(2),wind_apparent(1)); 
    
    sail_angle = -sign(psi_apparent)*(((delta_sail_min - delta_sail_max)/pi)*abs(psi_apparent) + delta_sail_max);  
end