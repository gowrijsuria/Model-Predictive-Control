function plot_final(x_pos,y_pos,r,no_of_steps)

    angle = 0:0.01:2*pi;
    figure(2);
    cla;
    hold on
    axis equal;
    hold on
    plot(x_pos,y_pos,'b');
    title('Motion along final trajectory without obstacles')
    hold on;

    
hold on

for i=1:no_of_steps
    h=fill(x_pos(i,1)+r*cos(angle),y_pos(i,1)+r*sin(angle),'k');
    drawnow;
    pause(0.2);
    delete(h)
end
    
end