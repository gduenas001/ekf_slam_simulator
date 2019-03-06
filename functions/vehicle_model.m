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

%% Notes for student:
% Using the bicycle model explained in class, this function must return
% x(k+1).

