function [x,P]= predict (x,P,v,g,Q,WB,dt)
%function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)
%
% Inputs:
%   x, P - SLAM state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   xn, Pn - predicted state and covariance

%% Notes for student: 
% This function implements the EKF prediction step. The estimate mean is
% updated using the same update as in the "vehicle_model.m" function, but
% now with the estimate instead of the true robot pose.
% You must compute the two Jacobian explained in class and apply the
% equations to propagate the cov. matrix. Note that the covariance
% componenets affecting only the landmarks are not updated so avoid as many
% multiplications by zero as possible by only updating the necessary parts
% of the cov. matrix.
% return the new estimate mean (x) and cov. matrix (P)