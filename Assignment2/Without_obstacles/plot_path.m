function plot_path(x_pos,y_pos)
    
    
    figure(1);
    cla;
    hold on
    axis equal;
    hold on
    plot(x_pos,y_pos,'b');
    hold on;
    
    pause(0.2);
    title('Computing Optimal Trajectory without obstacles')
end