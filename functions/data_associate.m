function [zf,idf, zn]= data_associate(x,P,z,R, gate1, gate2)
% function [zf,idf, zn]= data_associate(x,P,z,R, gate1, gate2)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances
%   gate1 - maximum threshold for the mahalanobis distance to accept an
%           association
%   gate2 - minimum threshold for the mahalanobis distance to create a new
%           landmark
%
% Outputs:
%   zf - same format as z, but with the non-associated measurements removed
%   idf - association vector for the landmark detections in zf
%   zn - same format as z, but with the landmark detections that will
%        create a new landmark in the state vector later using function
%        augment.m

%% Notes for student
% Simple gated nearest-neighbour data-association.
% Note that if a measurement from z is not associated and it does not
% create a new landmark, i.e. the Mahalanobis distance falls in between
% the two thresholds, it does not go to zf nor to zn and it won't be used.

