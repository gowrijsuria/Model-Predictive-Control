function plot_path(x_pos,y_pos,obs_x,obs_y,obs_r,r,no_obs)
    angle = 0:0.01:2*pi;
    
    figure(1);
    cla;
    hold on
    axis equal;
    hold on
    plot(x_pos,y_pos,'b');
    hold on;
    
    for iter_obs= 1:no_obs
        fill(obs_x(1,iter_obs)+ ((obs_r(1,iter_obs)+r)*cos(angle)), obs_y(1,iter_obs) + ((obs_r(1,iter_obs)+r)*sin(angle)),'c');
    end
    
    pause(0.2);
    title('Optimal Trajectory with Configuration Space of Obstacles')
end