function plot_final(x_pos,y_pos,obs_x,obs_y,obs_r,r,no_obs,no_of_steps)

    angle = 0:0.01:2*pi;
    figure(2);
    cla;
    hold on
    axis equal;
    hold on
    plot(x_pos,y_pos,'b');
    title('Motion along final trajectory with obstacles')
    hold on;
    
    for m= 1:no_obs
        fill(obs_x(1,m)+ ((obs_r(1,m))*cos(angle)), obs_y(1,m) + ((obs_r(1,m))*sin(angle)),'r');
    end
    
hold on

for i=1:no_of_steps
    h=fill(x_pos(i,1)+r*cos(angle),y_pos(i,1)+r*sin(angle),'k');
    drawnow;
    pause(0.2);
    delete(h)
end
    
end