function [x,P]= update(x,P,z,R,idf)
% function [x,P]= update(x,P,z,R,idf)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - landmark index for each z
%
% Outputs:
%   x, P - updated state and covariance

%% Notes for student
% This function implements the EKF update step.
% First, you should create the Jacobian H and the innovation vector gamma.
% Use the order provided in idf to compute them. As an example, if 
% idf = [4 3 1], this means that the first extracted landmark in z
% is associated to landmark 4 in the state vector, which are the elements
% (3 + 3*2 + 1) : (3 + 3*2 + 2) in x. The second detected landmark
% corresponds to landmark 3 in the state vector, which are the elements
% (3 + 2*2 + 1) : (3 + 2*2 + 2) in x, and so on...
% return the state vector (x) and cov. matrix (P) after the udpate step.

