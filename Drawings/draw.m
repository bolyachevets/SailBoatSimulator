function draw(y, fig)

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
a_true = wind_true(1) % speed
psi_true = wind_true(2) % direction in n-frame    
       
hull=[-1    1    2    2    1   -1   -1   -1;
      -0.5 -0.5 -0.25 0.25 0.5  0.5 -0.5 -0.5 ;
       1    1    1    1    1    1    1    1] ;
sail=[-2 0;
       0 0;
       1 1];
rudder=[-0.5 0;
         0 0;
         1 1];
R=[cos(y3),-sin(y3),y1;sin(y3),cos(y3),y2;0 0 1];
hull=R*hull;
Rdeltas=[cos(delta_s),-sin(delta_s),1;sin(delta_s),cos(delta_s),0;0 0 1];
Rdeltar=[cos(delta_r),-sin(delta_r),-1;sin(delta_r),cos(delta_r),0;0 0 1];
sail=R*Rdeltas*sail;
rudder=R*Rdeltar*rudder;
        
%% Boat
draw_arrow(0,0,psi_true,5*a_true,'blue'); % wind direction centered at the origin
hold on;
plot(hull(1,:),hull(2,:),'black');   
hold on;
plot(sail(1,:),sail(2,:),'red'); 
hold on;
plot(rudder(1,:),rudder(2,:),'red');
hold on;
        
end