
% NOTES:
%   This program is a SLAM simulator. To use, create a set of landmarks and 
%   vehicle waypoints (ie, waypoints for the desired vehicle path). The program
%   'frontend.m' may be used to create this simulated environment - type
%   'help frontend' for more information.
%       The configuration of the simulator is managed by the script file
%   'configfile.m'. To alter the parameters of the vehicle, sensors, etc
%   adjust this file. There are also several switches that control certain
%   filter options.


% ** USE THIS FILE TO CONFIGURE THE EKF-SLAM **
configfile; 

% this script initialize variables
initialization;

% main loop 
while iwp ~= 0
    
    % compute true data
    [G,iwp, NUMBER_LOOPS]=...
        compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt, NUMBER_LOOPS);
    xtrue= vehicle_model(xtrue, V,G, WHEELBASE,dt); %%%% BY STUDENT
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    
    % ------------------ EKF predict step ------------------ 
    [x,P]= predict (x,P, Vn,Gn,QE, WHEELBASE,dt);   %%%% BY STUDENT
    % -------------------------------------------------------
    
    % ------------------- EKF update step -------------------
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        
        % simulate measurements
        [z,ftag_visible]= get_observations(xtrue, lm, ftag, MAX_RANGE);
        z= add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
        
        % data association
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table]= data_associate_known(x,z,ftag_visible, da_table);
        else
            [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT); %%%% BY STUDENT
        end

        % EKF update
        [x,P]= update(x,P,zf,RE,idf);  %%%% BY STUDENT
        
        % add new landmarks to the state vector
        [x,P]= augment(x,P, zn,RE); 
    end
    % -------------------------------------------------------

    % offline data store
    data= store_data(data, x, P, xtrue);
    
    % ------------------------- plots -------------------------
    xt= transformtoglobal(veh,xtrue);
    xv= transformtoglobal(veh,x(1:3));
    set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:))
    set(h.xv, 'xdata', xv(1,:), 'ydata', xv(2,:))
    set(h.xf, 'xdata', x(4:2:end), 'ydata', x(5:2:end))
    ptmp= make_covariance_ellipses(x(1:3),P(1:3,1:3));
    pcov(:,1:size(ptmp,2))= ptmp;
    if dtsum==0
        set(h.cov, 'xdata', pcov(1,:), 'ydata', pcov(2,:)) 
        pcount= pcount+1;
        if pcount == 15
            set(h.pth, 'xdata', data.path(1,1:data.i), 'ydata', data.path(2,1:data.i))    
            pcount=0;
        end
        if ~isempty(z)
            plines= make_laser_lines (z,x(1:3));
            set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:))
            pcov= make_covariance_ellipses(x,P);
        end
    end
    drawnow
    % -------------------------------------------------------
end

data= finalise_data(data);
set(h.pth, 'xdata', data.path(1,:), 'ydata', data.path(2,:))    

