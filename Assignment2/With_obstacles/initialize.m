function [A, Q ,C]=initialize(wpt,wpx,wpy,del_t,no_of_steps)

x_init=0;
y_init=0;

%initialization of A,Q,C matrices
arr_size = 2*(no_of_steps-1);

A = zeros(arr_size,arr_size);
Q = zeros(arr_size,1);
C = 0;

%no of waypoints 
no_of_wps = size(wpt,2);

for way_pnt_iter = 1 : no_of_wps 
    
    curr_step = wpt(1,way_pnt_iter);
    curr_step_x = wpx(1,way_pnt_iter);
    curr_step_y = wpy(1,way_pnt_iter);
    
    rem = no_of_steps-1-curr_step;
    
    zero_arr = zeros(rem,1);
    ones_arr = ones(curr_step,1);
    
    x_t = [(x_init - curr_step_x); ones_arr; zero_arr];
    y_t = [(y_init - curr_step_y); ones_arr; zero_arr];
    
    zero_arr = zeros((no_of_steps-1),1);
    s_t_x = [x_t(2:end,1); zero_arr]*del_t;
    s_t_y = [zero_arr; y_t(2:end,1)]*del_t;
    
    %initial values of x_t(0) and y_t(0)
    x_t_0 = x_t(1,1);
    y_t_0 = y_t(1,1);
    
    s_t=[x_t_0 *x_t(2:end); y_t_0 *y_t(2:end)]*del_t; %(x0-xg)*del_t 
    
    %Updating the values of A, Q and C matrices
    A = A + s_t_x*s_t_x' + s_t_y*s_t_y';
    Q = Q + 2*s_t;
    C = C + x_t_0*x_t_0  + y_t_0*y_t_0;
    
    
    
end


end

