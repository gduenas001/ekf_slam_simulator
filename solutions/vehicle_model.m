function xv= vehicle_model(xv, V,G, WB,dt)
%
% INPUTS:
%   xv - vehicle pose [x;y;phi]
%   V - velocity
%   G - steer angle (gamma)
%   WB - wheelbase
%   dt - change in time
%
% OUTPUTS:
%   xv - new vehicle pose

 
%% using the rear wheel as ref point
xv= [xv(1) + V*dt*cos(xv(3)); 
     xv(2) + V*dt*sin(xv(3));
     pi_to_pi(xv(3) + V*dt*tan(G)/WB)];

% %% using the front wheel as ref point
% xv= [xv(1) + V*dt*cos(G+xv(3,:));
%     xv(2) + V*dt*sin(G+xv(3,:));
%     pi_to_pi(xv(3) + V*dt*sin(G)/WB)];
