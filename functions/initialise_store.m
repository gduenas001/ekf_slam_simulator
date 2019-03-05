function data= initialise_store(x,P, xtrue)
% data - a data structure containing:
%   data.true: the vehicle 'true'-path (ie, where the vehicle *actually* went)
%   data.path: the vehicle path estimate (ie, where SLAM estimates the vehicle went)
%   data.state(k).x: the SLAM state vector at time k
%   data.state(k).P: the diagonals of the SLAM covariance matrix at time k


% offline storage initialisation
data.i=1;
data.path= x;
data.true= xtrue;
data.state(1).x= x;
data.state(1).P= diag(P);

