% variables: x, y (position), theta (heading), omega (rotational speed) (in
% North-East-Up reference frame, n-frame), v (velocity)
% control is given by delta_r (rudder angle) and delta_s (sail angle)    
function  dydt = boat_dynamics(y)

    global p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11
    p1 = 0.03; % drift  coefficient
    p2 = 40; % tangential friction
    p3 = 6000; % angular friction
    p4 = 200; % sail lift
    p5 = 1500; % rudder lift
    p6 = 0.5; % distance to sail center of effort
    p7 = 0.5; % distance to mast
    p8 = 2; % distance to rudder
    p9 = 300; % mass of boat
    p10 = 400; % moment of inertia
    p11 = 0.2; % rudder break coefficient

    y1 = y(1); % x
    y2 = y(2); % y
    y3 = y(3); % heading
    y4 = y(4); % velocity
    y5 = y(5); % rotational speed
    delta = [y(6) y(7)]; % control for (rudder angle, sail angle)
    wind_true = [y(8) y(9)]; % input for true wind speed and direction

    % controls from input
    delta_r = delta(1);
    delta_s = delta(2);

    % true wind characteristics from input
    a_true = wind_true(1); % speed
    psi_true = wind_true(2); % direction in n-frame
    

    % apparent wind characteristics
    wind_apparent = [a_true*cos(psi_true - y3) - y4; a_true*sin(psi_true - y3)];
    psi_apparent = atan2(wind_apparent(2),wind_apparent(1));   
    a_apparent = norm(wind_apparent);
    
    % "the sheet is flexible and therefore the sail cannot
    % hold against the wind and stall the boat"
    sigma=cos(psi_apparent)+cos(delta_s);
    if (sigma<0), delta_s=pi+psi_apparent;  
    elseif(sigma>=0), delta_s=-sign(sin(psi_apparent))*delta_s;  
    end
    % rudder force
    fr = p5*y4*sin(delta_r);  
    % sail force
    fs = p4*a_apparent*sin(delta_s - psi_apparent);
    
    
    % Model from Modeling, control and state-estimation
    % for an autonomous sailboat, Jon Melin, 2015
    dy1dt = y4*cos(y3) + p1*a_true*cos(psi_true); % horizontal coordinate
    dy2dt = y4*sin(y3) + p1*a_true*sin(psi_true); % vertical coordinate
    dy3dt = y5;                                       % heading
    dy4dt = (fs*sin(delta_s) - fr*p11*sin(delta_r)-p2*y4^2)/p9; % velocity
    dy5dt = (fs*(p6 - p7*cos(delta_s)) - fr*p8*cos(delta_r) - p3*y5*y4)/p10; % rotational speed
       
    dydt = ([dy1dt; dy2dt; dy3dt; dy4dt; dy5dt]);
    
    fig = get(groot,'CurrentFigure');
    if isempty(fig)
        figure(1);
    end
    draw(y, 1);
    hold off;

        
end