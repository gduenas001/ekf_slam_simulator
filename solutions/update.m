function [x,P]= update(x,P,z,R,idf)
% function [x,P]= update(x,P,z,R,idf)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%
% Outputs:
%   x, P - updated state and covariance

lenz= size(z,2); % number of landmarks detected
lenx= length(x); % number of states (including landmarks)
H= zeros(2*lenz, lenx); % each landmark detection provides two measurements, thus 2*lenz number of rows
gamma= zeros(2*lenz, 1); % innovation vector
RR= zeros(2*lenz); % augmented covariance matrix

% create innovation and Jacobians
for i=1:lenz
    ii= 2*i + (-1:0);
    [zp,H(ii,:)]= observe_model(x, idf(i));
    
    gamma(ii)= [z(1,i) - zp(1);
                pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii)= R;
end
        
% apply the EKF equations
if lenz > 0
    Y= H*P*H' + RR;
    L= P*H'/Y;
    x= x + L*gamma;
    P= P - L*H*P;
end

end

function [z,H]= observe_model(x, idf)
%function [z,H]= observe_model(x, idf)
%
% INPUTS:
%   x - state vector
%   idf - index of feature order in state
%
% OUTPUTS:
%   z - predicted observation
%   H - observation Jacobian
%
% Given a feature index (ie, the order of the feature in the state vector),
% predict the expected range-bearing observation of this feature and its Jacobian.

Nxv= 3; % number of vehicle pose states
fpos= Nxv + idf*2 - 1; % position of xf in state
H= zeros(2, length(x));

% auxiliary values
dx= x(fpos)  -x(1); 
dy= x(fpos+1)-x(2);
d2= dx^2 + dy^2;
d= sqrt(d2);
xd= dx/d;
yd= dy/d;
xd2= dx/d2;
yd2= dy/d2;

% predict z
z= [d;
    atan2(dy,dx) - x(3)];

% calculate H
H(:,1:3)        = [-xd -yd 0; yd2 -xd2 -1];
H(:,fpos:fpos+1)= [ xd  yd;  -yd2  xd2];
end