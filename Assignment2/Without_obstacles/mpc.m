close all;
clear;
clc;

x_init=0;
y_init=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     initilization of co-ordinates and controls
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

initial_vx=0;
initial_vy=0;
final_vx=0;
final_vy=0;
robot_radius=0.5;

%initialization of waypoints along with their time stamps
wpx=[2,5,6,10];
wpy=[4,8,9,10];
wpt=[10,15,30,60];

%initilization of metadata: number of steps and time_interval for each step
dt=0.1;
n_steps=61;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     initilization of velocity and acceleration controls
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

min_accel=-2;
max_accel=2;

min_vel=0;
max_vel=10;

prev_cost = 0;
del_cost = 2;
init_guess = 0.1*abs(rand(2*(n_steps-1),1));

iter=1;
max_no_iter=20;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     initlization of obstacles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[A,Q,C] = initialize(wpt,wpx,wpy,dt,n_steps);


%start of optimization 
    
    %computation of the x and y co-ordinates over the n_steps
    x_guess = double(init_guess(1:n_steps-1))*dt;
    y_guess = double(init_guess(n_steps:end))*dt;
    
    estimate_x = cumsum([x_init; x_guess]);
    estimate_y = cumsum([y_init; y_guess]);
    estimate_x = estimate_x(2:end);
    estimate_y = estimate_y(2:end);
    

     
    [cvx_optval,optimal_vel]=cvx_optim(init_guess,A,Q,C,min_vel,max_vel,min_accel,max_accel,dt,n_steps,initial_vx,initial_vy,final_vx,final_vy);
   
    fprintf('Iteration : %0d => cost: %4.4f \n', iter, cvx_optval);
    
    
    %Calculating actual co-ordinates for plotting the trajectory
    x_var = cumsum([initial_vx;double(optimal_vel(1:n_steps-1))*dt]);
    y_var = cumsum([initial_vy;double(optimal_vel(n_steps:end))*dt]);
    
    plot_path(x_var,y_var);
   
   

disp("Optimal Trajectory found!");

plot_final(x_var,y_var,robot_radius,n_steps)
   
    
      
   
