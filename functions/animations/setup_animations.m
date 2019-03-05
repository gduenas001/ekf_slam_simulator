function h= setup_animations()
h.xt= patch(0,0,'b'); % vehicle true
h.xv= patch(0,0,'r'); % vehicle estimate
h.pth= plot(0,0,'k.','markersize',2); % vehicle path estimate
h.obs= plot(0,0,'r'); % observations
h.xf= plot(0,0,'r+'); % estimated features
h.cov= plot(0,0,'r'); % covariance ellipses
