%close all;
%clear;
%clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     initilization of co-ordinates and controls
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

initial_vx=0;
initial_vy=0;
final_vx=0;
final_vy=0;
robot_radius=0.5;

%initialization of waypoints along with their time stamps
wpx=[5,10];
wpy=[4,10];
wpt=[25,60];

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

obs_x = [2,4,7,10];
obs_y = [2,4,4,8];
obs_r = [0.8, 0.4,0.5,0.6];
num_obs = size(obs_x,2);
[A,Q,C] = initialize(wpt,wpx,wpy,dt,n_steps);


%start of optimization 
while( iter < max_no_iter && del_cost > 0.01)
    
    %computation of the x and y co-ordinates over the n_steps
    x_guess = double(init_guess(1:n_steps-1))*dt;
    y_guess = double(init_guess(n_steps:end))*dt;
    
    estimate_x = cumsum([x_init; x_guess]);
    estimate_y = cumsum([y_init; y_guess]);
    estimate_x = estimate_x(2:end);
    estimate_y = estimate_y(2:end);
    
    %computation of constraints for collision avoidance
    if(num_obs > 0)
        constraint_obsx = (repmat(estimate_x,1,num_obs)- repmat(obs_x,n_steps-1,1));
        constraint_obsy = (repmat(estimate_y,1,num_obs)- repmat(obs_y,n_steps-1,1));
        constraint_obsr  = (robot_radius + obs_r );
    end
     
    [cvx_optval,optimal_vel]=cvx_optim(init_guess,A,Q,C,min_vel,max_vel,min_accel,max_accel,dt,n_steps,initial_vx,initial_vy,final_vx,final_vy,constraint_obsx,constraint_obsy,constraint_obsr,num_obs);
   
    fprintf('Iteration : %0d => cost: %4.4f \n', iter, cvx_optval);
    
    if(iter > 1) 
        del_cost = abs( prev_cost- cvx_optval);
    end
   
    init_guess = double(optimal_vel);
    prev_cost = cvx_optval;
    
    
    iter=iter+1;
    %Calculating actual co-ordinates for plotting the trajectory
    x_var = cumsum([initial_vx;double(optimal_vel(1:n_steps-1))*dt]);
    y_var = cumsum([initial_vy;double(optimal_vel(n_steps:end))*dt]);
    
    plot_path(x_var,y_var,obs_x,obs_y,obs_r,robot_radius,num_obs);
   
   
end

disp("Optimal Trajectory found!");

plot_final(x_var,y_var,obs_x,obs_y,obs_r,robot_radius,num_obs,n_steps)
   
    
      
   
