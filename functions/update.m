function [x,P]= update(x,P,z,R,idf)
% function [x,P]= update(x,P,z,R,idf, batch)
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
